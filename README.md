# Engineering Projects Portfolio

This repository is a curated collection of academic term projects and personal engineering work.
The projects are organized as small, reviewable case studies with source files, configuration, and representative results.

## Projects

| Project | Stack | Summary |
| --- | --- | --- |
| [FPGA YOLOv5 UAV Tracking](projects/fpga-yolov5-uav-tracking) | Verilog, Vivado, FPGA, YOLOv5, UDP/Ethernet | Code-focused FPGA accelerator package for lightweight YOLOv5-style UAV target tracking on Xilinx VC707, with RTL, memory images, testbenches, and Vivado recreation script. |
| [AirSim FPGA Drone Tracking](projects/airsim-fpga-drone-tracking) | Python, AirSim, OpenCV, UDP, YOLO/FPGA | Drone tracking pipeline that combines AirSim simulation, bounding-box based control, FPGA inference communication, and tracking metrics. |
| [EKF Localization in MATLAB](projects/ekf-localization-matlab) | MATLAB | Extended Kalman Filter experiments for nonlinear CTRV motion, compared against a linear constant-velocity Kalman Filter. |
| [LTspice Decoder Design](projects/ltspice-decoder-design) | LTspice | CMOS decoder/predecoder and related circuit schematics with testbench files for simulation. |

## Repository Layout

```text
projects/
  fpga-yolov5-uav-tracking/
    hardware/
  airsim-fpga-drone-tracking/
    src/
    config/
    assets/
  ekf-localization-matlab/
    src/
    figures/
  ltspice-decoder-design/
    schematics/
```

## Curation Notes

Large archives, raw simulator outputs, cache files, and generated logs are intentionally excluded from Git.
Small source-level memory initialization files are kept when they are required to reproduce HDL simulations or synthesis.
This keeps the repository clean and makes it easier for reviewers to focus on the implementation and results.

No license has been selected yet.
