#============================================================================
#  vc707_eth_npu.xdc
#----------------------------------------------------------------------------
#  DDR3/MIG 제거판.  app 클럭은 MMCM 출력에서 Vivado 가 자동 유도하므로
#  수동 create_generated_clock 을 두지 않는다(원본 base.xdc 의 -divide_by 2
#  는 MMCM 실제 동작(x5/d10)과 불일치하는 잘못된 제약이었음).
#  분주비는 top 파라미터 APP_CLK_DIV 로만 바꾸면 파생 클럭이 자동 추종한다.
#============================================================================

#---- 200MHz 시스템 차동 클럭 (VC707, 뱅크 38 = 1.5V → DIFF_SSTL15) ----
set_property PACKAGE_PIN E19 [get_ports sys_diff_clock_clk_p]
set_property PACKAGE_PIN E18 [get_ports sys_diff_clock_clk_n]
set_property IOSTANDARD DIFF_SSTL15 [get_ports sys_diff_clock_clk_p]
set_property IOSTANDARD DIFF_SSTL15 [get_ports sys_diff_clock_clk_n]
create_clock -period 5.000 -name sys_diff_clock [get_ports sys_diff_clock_clk_p]

#---- reset / MDIO ----
set_property PACKAGE_PIN AV40 [get_ports reset]
set_property IOSTANDARD LVCMOS18 [get_ports reset]

set_property PACKAGE_PIN AH31 [get_ports mdio_mdc]
set_property IOSTANDARD LVCMOS18 [get_ports mdio_mdc]
set_property PACKAGE_PIN AK33 [get_ports mdio_mdio_io]
set_property IOSTANDARD LVCMOS18 [get_ports mdio_mdio_io]
set_property PACKAGE_PIN AJ33 [get_ports phy_rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports phy_rst_n]

#---- SGMII / GTX (SFP) ----
set_property BOARD_PART_PIN sgmii_mgt_clkp [get_ports gtrefclk_p]
set_property BOARD_PART_PIN sgmii_mgt_clkn [get_ports gtrefclk_n]
set_property PACKAGE_PIN AH8 [get_ports gtrefclk_p]
set_property PACKAGE_PIN AH7 [get_ports gtrefclk_n]
set_property BOARD_PART_PIN sgmii_rxp [get_ports rxp]
set_property BOARD_PART_PIN sgmii_rxn [get_ports rxn]
set_property BOARD_PART_PIN sgmii_txp [get_ports txp]
set_property BOARD_PART_PIN sgmii_txn [get_ports txn]
set_property LOC GTXE2_CHANNEL_X1Y1 [get_cells u_pcs_pma/inst/pcs_pma_block_i/transceiver_inst/gtwizard_inst/inst/gtwizard_i/gt0_GTWIZARD_i/gtxe2_i]
set_property PACKAGE_PIN AM7 [get_ports rxn]
set_property PACKAGE_PIN AM8 [get_ports rxp]
set_property PACKAGE_PIN AN1 [get_ports txn]
set_property PACKAGE_PIN AN2 [get_ports txp]

set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]

#---- sys_clk_ibuf BUFG 직결 (CLOCK_DEDICATED_ROUTE 완화) ----
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets sys_clk_ibuf]

#============================================================================
#  CDC: app 도메인(=sys_diff_clock 파생, MMCM app_clk 포함) vs 이더넷 GTX
#  도메인은 axis_async_fifo(gray-code)로 보호됨 -> 비동기 선언.
#  -include_generated_clocks 로 MMCM 파생 app 클럭이 자동 포함된다.
#============================================================================
set_clock_groups -asynchronous \
  -group [get_clocks -include_generated_clocks sys_diff_clock] \
  -group [get_clocks -include_generated_clocks -of_objects [get_pins -hier -filter {NAME =~ */gt0_GTWIZARD_i/gtxe2_i/TXOUTCLK}]] \
  -group [get_clocks -include_generated_clocks -of_objects [get_pins -hier -filter {NAME =~ */gt0_GTWIZARD_i/gtxe2_i/RXOUTCLK}]]

#---- async FIFO gray-pointer 2FF 동기화기 ASYNC_REG 패킹 ----
set_property ASYNC_REG true [get_cells -hier -quiet -filter {NAME =~ "*_sync1_reg_reg*"}]
set_property ASYNC_REG true [get_cells -hier -quiet -filter {NAME =~ "*_sync2_reg_reg*"}]
set_property ASYNC_REG true [get_cells -hier -quiet -filter {NAME =~ "*rst_sync*_reg_reg*"}]

#---- reset_sync 비동기 인가 경로 타이밍 예외 ----
set_false_path -to [get_pins -hier -quiet -filter {NAME =~ "*u_rst_sync*/r_sync_reg*/PRE"}]
