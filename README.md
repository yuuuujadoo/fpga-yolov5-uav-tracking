# FPGA YOLOv5 UAV Tracking Accelerator

FPGA에서 경량 YOLOv5 기반 표적 검출을 실행하기 위한 Verilog RTL 프로젝트입니다. 이 저장소는 공개 포트폴리오용으로 재구성한 코드 중심 패키지입니다.

## Overview

이 설계는 드론 또는 임베디드 비전 환경에서 입력 프레임을 FPGA 내부 연산 경로로 처리하는 것을 목표로 합니다. YOLOv5n-light 계열 네트워크를 대상으로 하며, 가중치, 양자화 파라미터, 중간 feature map을 BRAM/ROM 중심 구조에 배치해 외부 메모리 왕복을 줄이는 구조입니다.

핵심 아이디어는 모든 layer를 개별 하드웨어로 복제하지 않고, 하나의 NPU core가 convolution, pooling, vector, detection head engine을 시분할로 재사용하도록 만드는 것입니다. `sequencer_fsm`은 `layer_params.mem`의 layer micro-instruction을 순회하면서 각 engine을 순차적으로 활성화합니다.

## Features

- Target FPGA: Xilinx VC707 / `xc7vx485tffg1761-2`
- RTL: Verilog-2001 중심 구현
- Datapath: signed int8 activation/weight, int32 accumulation
- Quantization: dyadic requantization with per-channel parameters
- Activation LUTs: SiLU and sigmoid lookup tables
- Memory: shared 2048-bit global feature buffer
- Interface: UDP/BRAM-style 32-bit command backend
- Output: 96-bit bounding box stream
- Verification assets: simulation testbenches and memory initialization files

## Architecture

```text
Host / bridge
  -> UDP / BRAM command interface
  -> npu_eth_backend
  -> npu_top
      -> global_feature_buffer
      -> top_wrapper
          -> sequencer_fsm
          -> conv_engine
          -> pool_engine
          -> vec_engine
          -> head_engine
          -> weight_rom_array / param_rom_array
```

The command backend maps host-side writes and reads into image loading, inference start/status, and result collection:

| Address region | Direction | Purpose |
| --- | --- | --- |
| `0x0_______` | write | Image payload. 64 x 32-bit writes form one 2048-bit GFB word. |
| `0x1_______` | write | Control. bit 0 starts inference. |
| `0x2_______` | read | Status: box count, done flag, busy flag. |
| `0x3_______` | read | Result words: box count followed by 96-bit boxes split into three 32-bit reads. |

## Repository Layout

```text
hardware/
  constraints/              # VC707 XDC constraints
  ip/                       # Xilinx Gigabit Ethernet PCS/PMA IP configuration
  mem/                      # weights, layer parameters, and activation LUTs
  rtl/
    common/                 # reset synchronizer and common utility RTL
    ethernet/               # Ethernet/UDP RTL dependency
    npu/                    # YOLOv5 NPU RTL
    top/                    # board-level and backend integration RTL
  sim/                      # testbenches and 640-input verification data
  vivado/create_project.tcl # Vivado project recreation script
```

## Main RTL Blocks

- `hardware/rtl/npu/npu_top.v`: top-level NPU wrapper with load/run modes and bounding box output.
- `hardware/rtl/npu/top_wrapper.v`: integrates the sequencer, shared ROMs, and computation engines.
- `hardware/rtl/npu/sequencer_fsm.v`: drives layer execution from `layer_params.mem`.
- `hardware/rtl/npu/conv_engine.v`: shared 1x1/3x3 convolution execution path.
- `hardware/rtl/npu/pool_engine.v`: max-pooling execution path.
- `hardware/rtl/npu/vec_engine.v`: upsample, concat, and bypass-style vector operations.
- `hardware/rtl/npu/head_engine.v`: integer detection head path.
- `hardware/rtl/top/npu_eth_backend.v`: 32-bit host command interface.
- `hardware/rtl/top/vc707_eth_npu_top.v`: VC707 Ethernet/NPU integration top.

## Recreate Vivado Project

Required toolchain:

- Xilinx Vivado 2024.1 or a compatible 7-series Vivado release
- VC707 board files

From this project directory:

```powershell
vivado -mode batch -source hardware/vivado/create_project.tcl
vivado build/vivado/fpga_yolov5_uav_tracking/fpga_yolov5_uav_tracking.xpr
```

The script adds RTL, `.mem` initialization files, testbenches, constraints, and the `gig_ethernet_pcs_pma_0.xci` IP configuration. Generated Vivado output is intentionally ignored by Git.

## Simulation Notes

Testbenches are in `hardware/sim/`.

- `tb_full_infer_640.v`: full 640 x 640 inference and final AXI bounding box comparison.
- `tb_full_infer_640_diag.v`: diagnostic full inference testbench.
- `tb_cp_all_640.v`: checkpoint-oriented layer/output comparison.
- `tb_npu_eth_backend.v`: command backend interface testbench.

The full-inference testbench expects its input and golden files by relative filename. Use `hardware/sim/data` as the simulator working directory or copy the files in that directory into the Vivado simulation run directory before launching simulation.

## Privacy Scope

This repository is intentionally code-focused. It excludes personal identifiers, generated bitstreams, Vivado run directories, simulator caches, and synthesis logs.

## Notices

Third-party Ethernet RTL and Xilinx IP configuration files retain their original license terms. See `THIRD_PARTY_NOTICES.md` and `LICENSE.md`.
