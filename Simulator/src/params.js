/**
 * Physical parameters of the POW wheeled bipedal robot.
 *
 * Every value here is transcribed from the firmware of the original project
 * (SeungbinOh/Pow_WBR_Project), file ESP32/WBR_Control/Params.h, so that the
 * simulator runs on exactly the same numbers as the real robot.
 *
 * Units are SI: metres, kilogrammes, kg*m^2, radians (unless a name says deg).
 */

const MM = 1e-3;
const G = 1e-3;
const GMM2 = 1e-9;

const mm = (x, y, z) => [x * MM, y * MM, z * MM];
const gmm2 = (m) => m.map((v) => v * GMM2);

// Linkage geometry (Params.h :: createDefaultProperties)
export const GEOM = {
  a: 0.075 * Math.cos(Math.PI / 6), // hip pivot offset, along the leg plane u axis
  b: 0.075 * Math.sin(Math.PI / 6), // hip pivot offset, along the leg plane w axis
  l1: 0.106, // passive thigh link  A -> D
  l2: 0.077, // active thigh link   B -> C
  l3: 0.05, // knee coupler        D -> C
  l4: 0.137, // calf, D -> wheel axle (main component)
  l5: 0.008, // calf, D -> wheel axle (offset component)
  L: 0.123, // half track width
  R: 0.0725, // wheel radius
};

// Rigid bodies. Order matters: it is the order POL.h uses for p_vecs / c_vecs.
// `p` is the joint origin in the body frame, `c` the CoM in the link frame,
// `I` the inertia about that CoM, expressed in the link frame.
// The two thigh links carrying an index 5/6 get their `p` from forward
// kinematics at run time, so it is left null here.
export const BODIES = [
  {
    name: 'Body',
    m: 1524.76209213 * G,
    p: [0, 0, 0],
    c: mm(13.71923256, -0.22808627, 34.91864017),
    I: gmm2([
      4274811.10362144, 21823.60087554, 202865.50474913,
      21823.60087554, 7103674.50655196, 7275.19023018,
      202865.50474913, 7275.19023018, 8108785.33349067,
    ]),
  },
  {
    name: 'ThighActiveRight',
    m: 42.41994494 * G,
    p: [-0.064951905284, -0.086, 0.0375],
    c: mm(-48.49050768, -4.61247326, 2.04421032),
    I: gmm2([
      5008.68060146, -6441.33591254, -848.79062342,
      -6441.33591254, 48822.95563592, -404.74966887,
      -848.79062342, -404.74966887, 49903.76341241,
    ]),
  },
  {
    name: 'ThighActiveLeft',
    m: 42.41994494 * G,
    p: [-0.064951905284, 0.086, 0.0375],
    c: mm(-48.49051712, 4.61247327, 2.04421822),
    I: gmm2([
      5008.67928627, 6441.33404437, -848.78989403,
      6441.33404437, 48822.93805935, 404.75116121,
      -848.78989403, 404.75116121, 49903.74714712,
    ]),
  },
  {
    name: 'ThighPassiveRight',
    m: 38.26139565 * G,
    p: [0, -0.081, 0],
    c: mm(-77.93299656, -10.41097168, -3.75891919),
    I: gmm2([
      5119.23827329, -5810.74900488, 2235.15477431,
      -5810.74900488, 58048.93720431, 778.21403278,
      2235.15477431, 778.21403278, 58325.14232056,
    ]),
  },
  {
    name: 'ThighPassiveLeft',
    m: 38.26139565 * G,
    p: [0, 0.081, 0],
    c: mm(-77.93299656, 10.41097168, -3.75891919),
    I: gmm2([
      5119.23827939, 5810.74900473, 2235.15477477,
      5810.74900473, 58048.93720347, -778.21403282,
      2235.15477477, -778.21403282, 58325.14231362,
    ]),
  },
  {
    name: 'CalfRight',
    m: 319.23782393 * G,
    p: null, // from forward kinematics
    c: mm(172.54753946, 3.72875356, 7.27718211),
    I: gmm2([
      101347.04479182, 5354.04268934, -27779.76191434,
      5354.04268934, 703157.66050555, 252.37915614,
      -27779.76191434, 252.37915614, 676771.26272286,
    ]),
  },
  {
    name: 'CalfLeft',
    m: 319.23782393 * G,
    p: null, // from forward kinematics
    c: mm(172.54736867, -3.72877629, 7.27850779),
    I: gmm2([
      101346.64715298, -5353.69484487, -27779.40548575,
      -5353.69484487, 703160.42663078, -258.16665555,
      -27779.40548575, -258.16665555, 676774.26748659,
    ]),
  },
];

// Wheels are not part of the "body" aggregation: they spin, so POL keeps their
// inertia separate and constant in the body frame.
export const WHEELS = {
  right: {
    m: 214.11770281 * G,
    I: gmm2([
      312911.5850843, -0.00692278, -0.00051598,
      -0.00692278, 598903.109559, 0.00005987,
      -0.00051598, 0.00005987, 312911.64073952,
    ]),
  },
  left: {
    m: 237.11770281 * G,
    I: gmm2([
      352917.56444663, -0.00684532, -0.00133537,
      -0.00684532, 676120.35437132, 0.00006188,
      -0.00133537, 0.00006188, 352917.58100268,
    ]),
  },
};

// Operating limits (Params.h)
export const LIMITS = {
  HEIGHT_MIN: 0.07,
  HEIGHT_MAX: 0.2,
  PHI_MIN: -15, // deg
  PHI_MAX: 15, // deg
  VEL_MAX: 1, // m/s
  YAW_MAX: 1, // rad/s
  MAX_TORQUE: 0.75, // N*m per wheel
};

export const DT = 0.008; // firmware sampling time
export const GRAVITY = 9.80665;

// Hip servo travel, in degrees (HRController.h). Used to flag commands the
// real hardware would clip.
export const HIP_SERVO = {
  right: { min: 20, max: 120, center: 90 },
  left: { min: 60, max: 160, center: 90 },
};

/**
 * LQR gain schedule (VYBController.h). One 2x4 gain per 10 mm of body height,
 * the first entry at HEIGHT_MIN. Rows are [tau_RW; tau_LW], columns are the
 * state [theta, theta_dot, v, psi_dot].
 */
export const LQR_GAINS = [
  [1.35316904, 0.14009245, 0.23358589, -0.12161902, -1.36517013, -0.14243867, -0.23405192, -0.1217983],
  [1.40047216, 0.14638508, 0.23369175, -0.12160757, -1.41285639, -0.1488545, -0.2340732, -0.12184087],
  [1.44602994, 0.15279748, 0.23382116, -0.12158976, -1.45881851, -0.15539539, -0.23413544, -0.12186834],
  [1.4896865, 0.15931475, 0.2339735, -0.12156787, -1.50289679, -0.16204545, -0.23423617, -0.12188368],
  [1.53149557, 0.16592279, 0.23414424, -0.12154222, -1.54514525, -0.16879039, -0.23436934, -0.12188799],
  [1.57156576, 0.17260877, 0.23432853, -0.12151281, -1.58567342, -0.17561742, -0.23452873, -0.121882],
  [1.61001973, 0.17936153, 0.23452203, -0.12147964, -1.62460463, -0.18251548, -0.23470875, -0.12186644],
  [1.64697959, 0.18617179, 0.23472118, -0.12144289, -1.66206114, -0.18947534, -0.23490463, -0.12184212],
  [1.68256117, 0.19303219, 0.23492316, -0.1214029, -1.69815838, -0.19648966, -0.23511244, -0.12180996],
  [1.71687199, 0.19993741, 0.23512588, -0.12136017, -1.73300286, -0.20355303, -0.23532897, -0.12177102],
  [1.75001117, 0.20688435, 0.23532789, -0.12131532, -1.76669221, -0.21066222, -0.23555173, -0.1217264],
  [1.7820715, 0.21387275, 0.2355283, -0.12126915, -1.79931713, -0.2178167, -0.23577883, -0.12167735],
  [1.81314599, 0.22090653, 0.23572687, -0.12122289, -1.83096725, -0.22501991, -0.23600886, -0.12162555],
  [1.84334794, 0.22799757, 0.23592427, -0.12117958, -1.86174765, -0.23228239, -0.23624048, -0.12157472],
];
