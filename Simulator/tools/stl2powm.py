#!/usr/bin/env python3
"""
Convert the robot's binary STL link meshes into the compact form the simulator
loads in the browser.

The STLs come from the project's Gazebo model (model.sdf), one per link and
already expressed in that link's own frame — the same frames POL.h uses — so no
repositioning is needed here. What this does is:

  1. weld the STL's duplicated vertices into an indexed mesh,
  2. decimate with quadric edge collapse to a triangle budget,
  3. quantise positions to int16 over the mesh's bounding box,
  4. write a small binary the loader reads directly into a BufferGeometry.

The visual STLs run ~38k triangles and 1.9 MB each; at the default budget a link
lands around 90 kB with roughly a quarter of a millimetre of mean surface error.
Run with --report to print the measured error instead of guessing at it.

Usage:
    python3 tools/stl2powm.py meshes_src/*.stl -o meshes [--faces 6000] [--report]

Requires: numpy, fast-simplification  (pip install numpy fast-simplification)
"""

import argparse
import os
import struct
import sys

import numpy as np

MAGIC = b"POWM"
VERSION = 1


def read_binary_stl(path):
    """Return the STL's triangle corners as an (n*3, 3) float32 array."""
    with open(path, "rb") as f:
        header = f.read(80)
        if header[:5] == b"solid" and b"facet" in f.read(256):
            raise SystemExit(f"{path}: ASCII STL is not supported, re-export as binary")
        f.seek(80)
        (count,) = struct.unpack("<I", f.read(4))
        raw = np.frombuffer(f.read(count * 50), dtype=np.uint8)
    if len(raw) != count * 50:
        raise SystemExit(f"{path}: truncated, expected {count} triangles")
    # each 50-byte record is normal[3], v0[3], v1[3], v2[3] as float32, then u16
    return raw.reshape(count, 50)[:, 12:48].copy().view(np.float32).reshape(count * 3, 3)


def weld(corners, tol=1e-6):
    """Merge coincident corners into an indexed mesh."""
    keys = np.round(corners / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    return corners[first].astype(np.float32), inverse.reshape(-1, 3).astype(np.int32)


def point_triangle_distance(P, A, B, C):
    """Closest-point distance from points to triangles (Ericson), broadcast."""
    AB, AC, AP = B - A, C - A, P - A
    d1 = (AB * AP).sum(-1)
    d2 = (AC * AP).sum(-1)
    BP = P - B
    d3 = (AB * BP).sum(-1)
    d4 = (AC * BP).sum(-1)
    CP = P - C
    d5 = (AB * CP).sum(-1)
    d6 = (AC * CP).sum(-1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2

    denom = np.where(va + vb + vc == 0, 1e-30, va + vb + vc)
    Q = A + AB * (vb / denom)[..., None] + AC * (vc / denom)[..., None]

    def pick(mask, value):
        return np.where(mask[..., None], value, Q)

    Q = pick((d1 <= 0) & (d2 <= 0), A)
    Q = pick((d3 >= 0) & (d4 <= d3), B)
    Q = pick((d6 >= 0) & (d5 <= d6), C)

    den = np.where(d1 - d3 == 0, 1, d1 - d3)
    Q = pick((vc <= 0) & (d1 >= 0) & (d3 <= 0), A + AB * (d1 / den)[..., None])
    den = np.where(d2 - d6 == 0, 1, d2 - d6)
    Q = pick((vb <= 0) & (d2 >= 0) & (d6 <= 0), A + AC * (d2 / den)[..., None])
    den = np.where((d4 - d3) + (d5 - d6) == 0, 1, (d4 - d3) + (d5 - d6))
    Q = pick(
        (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0),
        B + (C - B) * ((d4 - d3) / den)[..., None],
    )
    return np.sqrt(((P - Q) ** 2).sum(-1))


def surface_error(sample, V, F, chunk=64):
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    out = np.empty(len(sample))
    for i in range(0, len(sample), chunk):
        P = sample[i : i + chunk][:, None, :]
        out[i : i + chunk] = point_triangle_distance(P, A[None], B[None], C[None]).min(1)
    return out


def write_powm(path, V, F):
    """
    POWM v1, little-endian:
        magic 'POWM' | u16 version | u16 flags | u32 verts | u32 tris
        f32 bbox_min[3] | f32 bbox_max[3]
        i16 positions[3 * verts]   (bbox-normalised)
        u8  pad[0 or 2]            (see below)
        u32 indices[3 * tris]
    Positions quantise to about 4 microns over a 250 mm part, far below the
    decimation error, so the quantisation is free in practice.

    The pad matters: an odd vertex count leaves the index block on a 2-byte
    boundary, and a Uint32Array view cannot start there — the loader throws
    outright. Two bytes of padding keep every index block 4-byte aligned.
    """
    lo, hi = V.min(0), V.max(0)
    span = np.where(hi - lo < 1e-9, 1.0, hi - lo)
    q = np.rint((V - lo) / span * 65535.0 - 32768.0).astype(np.int16)
    with open(path, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<HHII", VERSION, 0, len(V), len(F)))
        f.write(lo.astype("<f4").tobytes())
        f.write(hi.astype("<f4").tobytes())
        f.write(q.astype("<i2").tobytes())
        if (len(V) * 6) % 4:
            f.write(b"\0\0")
        f.write(F.astype("<u4").tobytes())
    return os.path.getsize(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="binary STL files")
    ap.add_argument("-o", "--outdir", default="meshes")
    ap.add_argument("--faces", type=int, default=6000, help="triangle budget per link (default 6000)")
    ap.add_argument("--report", action="store_true", help="measure surface error against the original")
    args = ap.parse_args()

    try:
        import fast_simplification
    except ImportError:
        raise SystemExit("needs fast-simplification:  pip install fast-simplification")

    os.makedirs(args.outdir, exist_ok=True)
    rng = np.random.default_rng(0)
    total = 0

    for src in args.inputs:
        V, F = weld(read_binary_stl(src))
        original = (V.copy(), F.copy())

        if len(F) > args.faces:
            V, F = fast_simplification.simplify(V, F, 1.0 - args.faces / len(F))
            V = V.astype(np.float32)
            F = F.astype(np.int32)

        name = os.path.splitext(os.path.basename(src))[0] + ".powm"
        size = write_powm(os.path.join(args.outdir, name), V, F)
        total += size

        note = ""
        if args.report:
            V0, F0 = original
            sample = V0[rng.choice(len(V0), min(1500, len(V0)), replace=False)].astype(np.float64)
            d = surface_error(sample, V.astype(np.float64), F)
            note = f" | erreur surface {d.mean() * 1000:.3f} mm moy, {d.max() * 1000:.3f} mm max"

        print(f"{name:38s} {len(F):6d} faces  {size / 1024:7.1f} kB{note}")

    print(f"{'total':38s} {'':6s}        {total / 1024:7.1f} kB")


if __name__ == "__main__":
    main()
