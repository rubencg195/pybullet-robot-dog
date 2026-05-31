# PyBullet robot dog

This repo is a [SpotMicro](https://github.com/michaelkubina/SpotMicroESP32)-style quadruped in [PyBullet](https://pybullet.org/). **V0** is one leg on a test stand built from primitives (cylinders/spheres) with a **Watt six-bar linkage knee drive** (ternary link, crank + coupler) matching the V1 CAD layout (60 mm femur/tibia)—this is the **working sim**. **V1** is the mechanical design reference: **Onshape** part STLs in `V1_test_stand/cad/stl/` (12 bodies) and drawings under `images/V1_test_stand/`. V1 does **not** ship sim meshes; use V0 for PyBullet. A mesh URDF stub (`leg_test_stand_cad.urdf`) and launchers exist for later. Everything after that—full dog, gaits, hardware—is sketched as V2+.

There’s a real build on the bench too—ESP32, servos, aluminium extrusion—so the sim is where we mess with poses without stripping gears.

---

## Try this first

Coronal camera (you’re looking along +X, abduction swings toward you) plus a GIF saved next to the other samples:

```bash
bash scripts/run_test_stand.sh --record recordings/README_v0_test_stand.gif --fps 15 --camera coronal
```

When the window opens, use the **Params** sliders on the right—they’re in **degrees**. Move the leg around; the recorder samples the **same** view you see (including if you orbit with the mouse), using the same idea as [interactive_robot_arm.py](https://github.com/rubencg195/aws-pybullet-environment/blob/main/scripts/interactive_robot_arm.py) in the aws-pybullet-environment repo. Stop with **Ctrl+C** or by closing the window so Pillow can flush the GIF. You’ll need Pillow and a PyBullet that actually imports—if that’s painful on your machine, jump to [Getting it running](#getting-it-running).

Still PNG when you quit:

```bash
bash scripts/run_test_stand.sh --snapshot recordings/README_v0_test_stand.png --camera coronal
```

### What’s in `recordings/`

We only commit a few files from that folder (see `.gitignore`). Right now the README uses:

| GIF | PNG |
|:---:|:---:|
| ![PyBullet test stand session](recordings/README_v0_test_stand.gif) | ![PyBullet still](recordings/PYB-SIM.png) |

**Sister project** — browser-based [robot-dog-simulator](https://github.com/rubencg195/robot-dog-simulator) (Three.js; live at [robotdogsim.rubenchevez.com](https://robotdogsim.rubenchevez.com)):

![Web simulator demo](recordings/README_robot_dog_simulator.gif)

### V1 CAD (Onshape export, v1.1)

Reference renders from the V1 test stand CAD live under `images/V1_test_stand/` (same idea as `references/spot-micro/images/` for the full [SpotMicro](https://www.thingiverse.com/thing:3445283) Thingiverse pack—reference art kept out of sim code). The design was migrated from Fusion 360 to **Onshape** in v1.1; the **shoulder servo holder** piece was added. Printable part geometry is the 12 STLs in `V1_test_stand/cad/stl/` (one file per Onshape body—femur, tibia, servo housings, Watt six-bar links, …).

| Mechanism (front view, v1.0) | Dimensions (front view, v1.0) |
|:---:|:---:|
| ![V1 leg mechanism — Fusion v1.0 front view](images/V1_test_stand/front-view-leg-mechanism-v1-0.png) | ![V1 leg dimensions — Fusion v1.0](images/V1_test_stand/front-view-leg-mechanism-dimensions-v1-0.png) |

| Mechanism (perspective, v1.1) |
|:---:|
| ![V1 leg mechanism — Onshape v1.1 perspective view with shoulder servo holder](images/V1_test_stand/perspective-view-mechanism-v1-1.png) |

The mechanism is a **Watt six-bar linkage** with a **ternary link** driving the tibia (MG996R housings, servo horn geometry, crank + coupler) on a fixed test-stand base—the same class of leg actuation described for Iron Dog Mini ([Rahman et al., 2023](https://doi.org/10.3390/robotics12010028)). **V0** simulates this linkage with primitives; the CAD is the mechanical source of truth. Compare femur/tibia/shoulder-offset lengths to `LegConfig` in `common/kinematics.py` (**L1** 55 mm, **L2** 60 mm, **L3** 60 mm).

---

## What’s in the box

**V0** — `V0_test_stand/urdf/leg_test_stand.urdf` is the working leg: hip abduction + flexion, Watt six-bar linkage knee drive (`knee_drive` crank, passive knee + coupler + ternary link, loop closed by a PyBullet constraint), V1 dimensions (60 mm links), base at 0.35 m. Sliders: `hip_abduction`, `hip_flexion`, `knee_drive`. `test_stand.py` and `ik_demo.py` live here.

**V1** — mechanical reference only: `V1_test_stand/cad/stl/` (12 Onshape part STLs), `images/V1_test_stand/` (drawings), and `urdf/leg_test_stand_cad.urdf` (placeholder for a future mesh sim—**not wired up yet**). There is no `meshes/` content in the repo. For PyBullet today, use **V0**.

Shared pieces: `common/kinematics.py` (FK/IK, `LegConfig`), `common/debug_visualizer.py`, `common/view_capture.py`. Shell helpers: `check_v0_env.sh`, `run_test_stand.sh`, `run_ik_demo.sh`, plus `run_test_stand_v1.sh` / `run_ik_demo_v1.sh`.

---

## Architecture

Folders are versioned so the working V0 sim and the V1 CAD reference stay separate, and later quadruped work can share `common/`.

```mermaid
flowchart LR
  subgraph project["PyBullet Robot Dog"]
    COMMON["common/\nKinematics\nVisualisation"]
    V0["V0_test_stand/\nPrimitives\nworking now"]
    V1["V1_test_stand/\nOnshape CAD\n+ drawings"]
    V2["V2 (next)\nFull quad\nbody + legs"]
    V3["V3\nGaits"]
    V4["V4\nReal robot"]
  end
  COMMON --> V0
  COMMON --> V1
  COMMON --> V2
  COMMON --> V3
  COMMON --> V4
  V0 -->|"same leg layout"| V1
  V1 -->|"four legs"| V2
  V2 -->|"locomotion"| V3
  V3 -->|"ESP32 / PWM"| V4
```

### Sim data flow — V0 (working today)

V0 loads `leg_test_stand.urdf` (primitives + Watt six-bar). Kinematics and debug draw live in `common/`.

```mermaid
flowchart TB
  subgraph user["User Input"]
    SLIDERS["GUI Joint Sliders\nhip_abduction\nhip_flexion\nknee_drive"]
    CLI["CLI Arguments\n--path circle|line|step\n--record  --speed  --loops\n--urdf optional"]
  end

  subgraph scripts["Test stand scripts"]
    TS["test_stand.py\nInteractive FK Explorer\nSliders → joints → trail"]
    IK["ik_demo.py\nIK Path Tracing\nTarget path → IK → drive"]
  end

  subgraph common_lib["common/ Library"]
    KIN["kinematics.py\n─────────────────\nLegConfig dataclass\nforward_kinematics()\nforward_kinematics_full()\ninverse_kinematics()"]
    VIZ["debug_visualizer.py\n─────────────────\nDebugVisualizer class\nTrails · Skeleton · Markers\nHUD text · Path drawing"]
  end

  subgraph model["URDF model"]
    URDF["leg_test_stand.urdf\n(primitives + Watt 6-bar)\nbase z=0.35 m"]
  end

  subgraph engine["PyBullet Engine"]
    LOADER["URDF Loader\nloadURDF · fixedBase"]
    CTRL["Position Control\nsetJointMotorControl2"]
    PHYS["Physics Stepping\nGravity · Dynamics"]
    RENDER["GUI Renderer\nOpenGL viewport"]
    PBFK["getLinkState()\nFK verification"]
  end

  subgraph output["Visual Output"]
    TRAIL["Foot Trail\nGreen persistent path"]
    SKEL["Leg Skeleton\nYellow overlay lines"]
    MARKERS["Joint Markers\nColoured crosses at each joint"]
    HUD["Debug HUD\nAngles · Foot pos · FK error"]
    TARGET["Target Path\nOrange reference trajectory"]
    GIF["GIF/PNG\nview_capture.py\n--record · --snapshot"]
  end

  SLIDERS --> TS
  CLI --> IK
  TS --> KIN
  TS --> VIZ
  IK --> KIN
  IK --> VIZ
  TS --> LOADER
  IK --> LOADER
  URDF --> LOADER
  LOADER --> PHYS
  KIN -.->|"FK angles"| CTRL
  CTRL --> PHYS
  PHYS --> RENDER
  PBFK -.->|"verification"| HUD
  VIZ --> TRAIL
  VIZ --> SKEL
  VIZ --> MARKERS
  VIZ --> HUD
  VIZ --> TARGET
  RENDER --> GIF
```

### Leg chain

```mermaid
flowchart LR
  BASE["Test Stand\n(fixed base)\nz = 0.35 m"]
  BASE -->|"q1: Hip Abduction\naxis X · ±31°"| SH["Shoulder\nL1 = 55 mm"]
  SH -->|"q2: Hip Flexion\naxis Y"| FEM["Femur\nL2 = 60 mm"]
  FEM -->|"knee (passive)\nWatt 6-bar driven"| TIB["Tibia\nL3 = 60 mm"]
  SH -->|"knee_drive\ncrank 29 mm"| CRANK["Crank + coupler\n+ ternary link\n60 mm push-rod"]
  CRANK -.->|"closes loop"| TIB
  TIB --> FOOT["Foot"]
```

### IK pipeline (geometric)

```mermaid
flowchart TD
  INPUT["Target foot position\n(px, py, pz) in hip frame"]
  INPUT --> CHECK{"r² − L1² ≥ 0 ?"}
  CHECK -->|No| FAIL["Return None\n(unreachable)"]
  CHECK -->|Yes| Q3["Knee angle q3\ncos q3 = (d² − L2² − L3²) / 2·L2·L3\nq3 = knee_sign · acos(cos q3)"]
  Q3 --> Q3CHECK{"│cos q3│ ≤ 1 ?"}
  Q3CHECK -->|No| FAIL
  Q3CHECK -->|Yes| Q2["Hip flexion q2\nq2 = atan2(−px, D) − atan2(B, A)\nA = L2 + L3·cos q3\nB = L3·sin q3"]
  Q2 --> Q1["Hip abduction q1\nq1 = atan2(py, −pz)\n     − atan2(side·L1, √(py²+pz²−L1²))"]
  Q1 --> RESULT["Return (q1, q2, q3)"]
```

---

## Leg numbers (V1 CAD)

Dimensions from the dimensioned drawing (v1.0 Fusion, v1.1 Onshape); used in V0 primitives and `LegConfig`:

- **L1** 55 mm — shoulder offset (abduction axis to flexion axis)
- **L2** 60 mm — femur
- **L3** 60 mm — tibia
- **Ground link** 33 mm — hip pivot to knee-drive pivot (on shoulder)
- **Crank** 29 mm — knee servo arm
- **Coupler** 60 mm — push-rod (crank tip to knee)
- **Ternary link** — three-joint coupler in the Watt six-bar loop (see `Leg - TibiaTernary.stl` in `cad/stl/`)

Straight leg reaches **120 mm** below the hip flexion axis. FK/IK in `common/kinematics.py` still use the three-link model (L1–L3); the Watt six-bar linkage maps crank angle to knee angle in the URDF.

Axes: **X** forward, **Y** left, **Z** up. Base plate sits at **z = 0.35 m**. With all active joints at zero (right leg, `side_sign = -1`), the foot is near **(0, −0.055, −0.120)** in the hip frame (Watt six-bar linkage settles the passive knee).

```
      Z (up)
      │
      │       (platform at z = 0.35 m)
      ╰──────→ X (forward)
     ╱
    Y (left)
```

---

## Repo layout

```
pybullet-robot-dog/
├── README.md
├── requirements.txt
├── images/
│   └── V1_test_stand/          # CAD reference renders (PNG)
├── recordings/                 # mostly ignored; README_* and PYB-SIM.png are tracked
├── references/
│   ├── documentation/          # Iron Dog Mini Watt six-bar paper (PDF)
│   └── spot-micro/             # Thingiverse reference STLs + images (see README.txt)
├── scripts/
│   ├── check_v0_env.sh
│   ├── run_test_stand.sh
│   ├── run_ik_demo.sh
│   ├── run_test_stand_v1.sh
│   └── run_ik_demo_v1.sh
├── common/
│   ├── kinematics.py
│   ├── debug_visualizer.py
│   └── view_capture.py
├── V0_test_stand/
│   ├── urdf/leg_test_stand.urdf
│   ├── test_stand.py
│   └── ik_demo.py
└── V1_test_stand/
    ├── cad/stl/                # Onshape part export — 12 STLs (v1.1)
    ├── urdf/leg_test_stand_cad.urdf   # placeholder mesh URDF (future)
    ├── test_stand.py           # stub — exits until mesh sim exists
    └── ik_demo.py
```

---

## Getting it running

You want Python 3.10+ (older might work; we use `X | None` in a few places) and something that can open an OpenGL window. On WSL that often means WSLg, VcXsrv, or a remote desktop from [aws-pybullet-environment](https://github.com/rubencg195/aws-pybullet-environment).

**venv + pip** — on many Linux installs PyBullet compiles from source, so you need a compiler:

```bash
cd pybullet-robot-dog
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Miniconda** — if `g++` isn’t there or pip keeps dying, conda-forge ships a binary:

```bash
conda install -y -c conda-forge pybullet numpy pillow
# if conda nags about ToS on defaults:
# conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
# conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

Smoke test:

```bash
bash scripts/check_v0_env.sh
```

### Test stand

```bash
bash scripts/run_test_stand.sh
# or, explicitly:
bash scripts/run_test_stand.sh --record recordings/README_v0_test_stand.gif --fps 15 --camera coronal
python -u V0_test_stand/test_stand.py   # if your venv is already active
```

The **Params** panel sliders are PyBullet “user parameters”—they’re UI, not part of the physics. They set joint **targets** in degrees (we clamp to the URDF). **Clear Trail** just nukes the green path.

Cameras: **`stand`** is the side/profile rig shot (default before we cared about naming); **`iso`** is the old 45° corner; **`coronal`** is the face-on +X view used in the gallery GIF.

Green = foot trace, yellow = skeleton overlay, HUD = angles and a quick FK check against `getLinkState`.

### IK demo

```bash
bash scripts/run_ik_demo.sh
bash scripts/run_ik_demo.sh --path line
bash scripts/run_ik_demo.sh --path step --loops 2 --record recordings/ik_step.gif
```

Orange = commanded path, red dot = current target, green = where the foot actually went, yellow sticks = leg.

If you have two Pythons fighting, force one: `export PY_ROBOT_DOG=/path/to/python`.

### V1 (CAD reference — not a sim yet)

V1 is committed as **CAD and drawings**, not as a runnable PyBullet model:

- `V1_test_stand/cad/stl/` — 12 Onshape part STLs (print/manufacture/reference)
- `images/V1_test_stand/` — mechanism and dimension renders

There are **no sim meshes** in V1. `run_test_stand_v1.sh` and `V1_test_stand/test_stand.py` are stubs that tell you to use V0:

```bash
bash scripts/run_test_stand.sh --camera coronal   # use this
# bash scripts/run_test_stand_v1.sh               # not ready — no mesh URDF yet
```

### Recording notes

`--record` and `--snapshot` pull from the **debug** camera matrices, so what you record is what you framed in the GUI. GIF cadence follows `--fps` on the wall clock, not one frame per physics step—same trick as the Kuka script linked above. Pillow is required.

---

## Kinematics (quick reference)

**FK** for a right leg (`side_sign = -1`), foot in hip frame:

```
x = −L₂ sin(q₂) − L₃ sin(q₂ + q₃)

D = L₂ cos(q₂) + L₃ cos(q₂ + q₃)

y = side · L₁ cos(q₁) + D sin(q₁)
z = side · L₁ sin(q₁) − D cos(q₁)
```

`forward_kinematics_full()` also returns hip / shoulder / knee points for drawing.

**IK** is geometric: knee from law of cosines, then hip flexion, then abduction. It hands back `None` if the point is past full extension, inside the shoulder sphere, or otherwise silly. Details are in `common/kinematics.py`.

---

## Roadmap

```mermaid
flowchart LR
  V0["V0\nPrimitives"] --> V1["V1\nCAD stand"]
  V1 --> V2["V2\nQuadruped"]
  V2 --> V3["V3\nGaits"]
  V3 --> V4["V4\nHardware"]
```

| Phase | What | Status |
|-------|------|--------|
| **V0** | Single leg, Watt six-bar knee URDF (V1 dims), sliders, IK demo, GIF/PNG capture | in good shape |
| **V1** | Onshape v1.1 CAD in `cad/stl/` + reference images in `images/V1_test_stand/` | CAD in repo; **no mesh sim** |
| **V2** | Full body URDF, four legs, stand/sit poses, foot placement from body pose | not started |
| **V3** | Scheduled gaits—trot, crawl, turns, maybe rough terrain hooks | not started |
| **V4** | Talk to the real rig—ESP32, PWM calibration, later IMU if we need it | not started |

**V0 loose ends** (optional polish on the primitive leg):

| # | Task |
|---|------|
| 0.10 | Plot reachable workspace |
| 0.11 | Velocity / torque limits in IK |
| 0.12 | Jacobian / singularities |

**V1 mesh sim (future):** if we add a mesh URDF later, it would mean merging `cad/stl/` parts into coarse link STLs aligned to V0 joint frames—not dropping the 12 part files straight into PyBullet. `leg_test_stand_cad.urdf` and the V1 launchers are placeholders for that work.

---

## Troubleshooting

### It dies on import

Run `bash scripts/check_v0_env.sh`. Typical failures:

- **`No module named numpy`** — install deps in an active venv: `pip install -r requirements.txt`.
- **`No module named pybullet`** — pip tried to compile Bullet and you don’t have `g++`. Install `build-essential` + `python3-dev`, or grab PyBullet from conda-forge (see above). The error `x86_64-linux-gnu-g++' failed` is exactly that.
- **Display / `Error 11`** — no GL context. Fix `DISPLAY`, use WSLg, or run on a machine with a real window.

On stripped-down Ubuntu/WSL, missing **build-essential** is the usual villain.

### IK returns `None`

Usually the target is out of workspace, too close to the hip line, or you mixed up hip frame vs world frame (remember the +0.35 m base height in Z).

### FK error not zero

Position control takes a moment to settle; a few mm at a step boundary is normal, then it should hug `getLinkState` once the motors catch up.

### GIF looks wrong

We still render captures with TinyRenderer; if it looks softer than the live GL view, bump `--width` / `--height` or orbit before you record.

### Sliders dead / UI stuck

One PyBullet connection at a time; if you’re debugging inside an IDE, try a plain terminal.

### URDF “not found”

Run from repo root (the scripts resolve paths from `__file__`, but it saves headaches).

### V1 launcher exits immediately

Expected. V1 has **no sim meshes**—only Onshape part STLs in `cad/stl/`. Use V0:

```bash
bash scripts/run_test_stand.sh
```

### Where we’re stuck in general

- **No CI** that opens a real GUI—run locally or on the DCV box.
- **PEP 668** distros: use a venv, don’t fight the system Python.

---

## References

- **In-repo:** `references/documentation/` — [Rahman et al. (2023)](references/documentation/A_Dynamic_Approach_to_Low-Cost_Design_Development_.pdf) (*Iron Dog Mini*; Watt six-bar linkage with ternary link for tibia actuation)
- **In-repo:** `references/spot-micro/` — original [Thingiverse SpotMicro](https://www.thingiverse.com/thing:3445283) STLs, photos, and `README.txt` (parts list, assembly videos). Not used directly by the sim; kept for comparison and future quadruped work.
- [Rahman et al. (2023)](https://doi.org/10.3390/robotics12010028) — DOI for the same Iron Dog Mini paper
- [robot-dog-simulator](https://github.com/rubencg195/robot-dog-simulator) — browser-based Three.js sister project ([live preview](https://robotdogsim.rubenchevez.com))
- [SpotMicro ESP32](https://github.com/michaelkubina/SpotMicroESP32)
- [SpotMicro AI](https://github.com/FlorianWilworeit/SpotMicroAI)
- [PyBullet quickstart](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit)
- [MIT Mini Cheetah software](https://github.com/mit-biomimetics/Cheetah-Software)
- [aws-pybullet-environment](https://github.com/rubencg195/aws-pybullet-environment) — remote GPU desktop we’ve used for PyBullet before
