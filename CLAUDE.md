# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

FUT-ROBO is a small-scale robot soccer system (CIR — Competição Interna de Robótica) made of four independent modules that talk to each other over UDP, plus a shared wire-format header:

- `vision/` — C++23 program: reads a camera, detects the ball and 4 robots (2 teams × 2 players) by HSV color segmentation, converts pixel coordinates to real field coordinates via a homography calibration, and publishes poses over UDP multicast.
- `strategy/` — C++23 program: listens to the multicast pose stream, runs a control law for one robot, and sends velocity commands over UDP unicast to that robot.
- `firmware/` — ESP-IDF (C) firmware for the ESP32 on each robot: receives velocity commands over UDP, converts them to differential-drive wheel PWM, and drives the motors.
- `ihm/` — Python/Tkinter desktop console: starts/stops the `vision` and `strategy` processes, configures/builds them via CMake, visualizes the live multicast telemetry on a drawn field, edits `vision/config.yaml`, manages calibration files, and can send stop commands directly to a robot's firmware.
- `common/include/common/protocol.h` — the packed C struct definitions that define the UDP wire format shared by all of the above.

## Data flow

```
camera --> vision (color segmentation + homography calibration)
             |  publishes FieldPoses (ball + 4 robot poses)
             v  UDP multicast 224.1.1.1:5007
         strategy (listens for one robot id, runs Controller)
             |  sends TwistCmd {v, w}
             v  UDP unicast to <robot host>:<port> (default robot.local:3333)
         firmware (ESP32, ESP-IDF)
             |  differential-drive kinematics -> PWM (LEDC)
             v  motors
```

`ihm` sits alongside this pipeline: it also joins the same multicast group to render live telemetry, and it is the process manager/config editor for `vision` and `strategy` (it shells out to CMake/the built binaries rather than linking against them).

### Wire protocol (`common/include/common/protocol.h`)

`#pragma pack(push, 1)` structs: `Vec2` (int16 x/y), `RobotPose` (id + `Vec2` + float angle), `FieldPoses` (ball `Vec2` + 4 `RobotPose`), `TwistCmd` (float v m/s, w rad/s). This header is the single source of truth for the C/C++ side (`vision`, `strategy`, `firmware`) and is mirrored by hand in `ihm/protocol.py` (`struct.Struct` format strings, currently `FIELD_POSES` = 40 bytes, `TWIST_CMD` = 8 bytes). **Any change to `protocol.h` must be mirrored in `ihm/protocol.py` and in `firmware/main/main.c`'s assumptions about `TwistCmd` size**, or the systems will silently desync.

Field coordinates are real centimeters centered at the origin: the field is 237 cm along x (image horizontal) by 165 cm along y, set by `field.sizeX_cm`/`field.sizeY_cm` in `vision/config.yaml` (read by `vision/src/Calibration.cpp` and `ihm/app.py`; `strategy/src/Controller.cpp` hardcodes the same half-sizes). `vision/data/calib.yaml` stores the 4 clicked corner pixels, and the homography is rebuilt from them and the configured size on every start, so changing the field size needs no re-clicking; moving the camera needs `--calibrar`. Both axes must use the true scale; a different scale per axis distorts every measured angle.

Robot IDs are assigned in `vision/src/main.cpp` as `id = team_index + (player_index << 1)`, giving IDs 0–3 across the two teams.

## Build & run

### vision (C++23, needs OpenCV `core imgproc highgui calib3d` + standalone/non-Boost Asio, both found via system packages — see `vision/flake.nix` for a Nix dev shell)

```bash
cd vision
cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/vision
```

On first run it walks through field calibration (7 points, clockwise from center) and color selection (click-to-sample) for each of `laranja`, `verde`, `azul`, `rosa`, `amarelo`; results are cached as YAML in `vision/data/`. Delete the relevant YAML file to force recalibration. Runtime tuning (ROI sizes, thresholds, camera id/resolution, window) lives in `vision/config.yaml`.

### strategy (C++23, needs Asio)

```bash
cd strategy
cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug
cmake --build build
FUT_ROBOT_ID=0 FUT_ROBOT_HOST=robot.local FUT_ROBOT_PORT=3333 ./build/strategy
```

`FUT_OBJETIVO=gol` (default) drives behind the ball facing the attacking goal (right, +x) with a polar-coordinate pose controller (`strategy/src/GoalApproach.cpp`); `FUT_OBJETIVO=bola` only drives to the ball. Control constants live in `strategy/include/strategy/ControlParams.hpp`. `FUT_TESTE_GIRO=1` and `FUT_TESTE_MOTOR=1` run calibration tests. `FUT_ROBOT_ID` must be 0–3 (clamped). If no valid `FieldPoses` frame arrives for 250 ms, `strategy` sends a zero `TwistCmd` and resets controller state.

### firmware (ESP-IDF, target `esp32` by default; a Linux simulator target is also supported per `firmware/README.md`)

```bash
cd firmware
idf.py build
idf.py -p PORT flash monitor
```

### ihm (Python 3 + Tkinter, standard library only — no external packages)

```bash
python ihm/app.py            # or ./ihm/iniciar_ihm.sh on Linux, iniciar_ihm.bat on Windows
./ihm/iniciar_ihm.sh --check # verify Python/Tkinter and project layout without opening the UI
```

The `ihm` folder is meant to sit at the repo root next to `vision`/`strategy`/`firmware`/`common`; both launcher scripts locate the project root relative to their own path. Linux needs `python3-tk` installed separately (`sudo apt install python3 python3-tk`).

Tests (Python `unittest`, covering the binary protocol codec and the conservative OpenCV-YAML config read/write):

```bash
python -m unittest discover -s ./ihm/tests -v
```

Run a single test, e.g.:

```bash
python -m unittest ihm.tests.test_protocol.ProtocolTests.test_decodes_field_packet
```

There is no test suite for the C++ modules (`vision`, `strategy`) or for `firmware`.

## Notes

- `vision` and `strategy` both compile with `-std=c++23` and expect Asio headers on the include path outside of CMake's own management (no `find_package(asio)`/FetchContent) — install it system-wide (e.g. `libasio-dev`) or use the Nix shell in `vision/flake.nix`.
- The top-level `README.md` describes an older MQTT/Mosquitto-based publisher; the current `Publisher` (`vision/include/futbot/Publisher.hpp`) sends `FieldPoses` over UDP multicast instead — trust the code and `strategy/README.md` over the top-level README for the current transport.
- `strategy/README.md` documents the controller's behavior and environment variables in more detail than this file.
