# Vendored dependencies

Three.js r169, taken unmodified from the `three@0.169.0` npm package:

| file                  | source in the package                        |
| --------------------- | -------------------------------------------- |
| `three.module.min.js` | `build/three.module.min.js`                  |
| `OrbitControls.js`    | `examples/jsm/controls/OrbitControls.js`      |
| `LICENSE`             | `LICENSE` (MIT)                              |

They are checked in rather than pulled from a CDN so the simulator opens with
no network access, and keeps working if a CDN URL moves. `index.html` maps the
bare `three` specifier onto these files through an import map.
