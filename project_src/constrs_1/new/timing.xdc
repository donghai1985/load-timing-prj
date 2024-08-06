create_clock -period 20.000 -name S_FPGA_CLK [get_ports S_FPGA_CLK]

create_clock -period 8.000 -name SFPGA_RGMII_RX_CLK [get_ports SFPGA_RGMII_RX_CLK]


set_clock_groups -name async_clk_group -asynchronous \
-group [get_clocks -include_generated_clocks {SFPGA_RGMII_RX_CLK}] \
-group [get_clocks -include_generated_clocks {S_FPGA_CLK}]
