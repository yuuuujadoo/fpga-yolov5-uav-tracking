# Third-Party Notices

This repository includes project-specific RTL and supporting third-party RTL/IP configuration files.

## Verilog Ethernet RTL

Files under `hardware/rtl/ethernet/` are Ethernet/UDP support RTL originally from the Verilog Ethernet project by Alex Forencich. The original MIT license headers are preserved in the source files.

If this repository is published publicly, keep those headers intact and mention this dependency in the README.

## Xilinx IP

`hardware/ip/gig_ethernet_pcs_pma_0/gig_ethernet_pcs_pma_0.xci` is a Vivado IP configuration for Xilinx Gigabit Ethernet PCS/PMA. Building or regenerating this IP requires a compatible Xilinx Vivado installation and is subject to Xilinx license terms.
