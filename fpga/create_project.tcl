# SET PROJECT NAME
set  project_name Loongson_Soc
set  project_path ./project
set project_part xc7a200tfbg676-1
# CLEAR
file delete -force $project_path

create_project -force $project_name $project_path -part $project_part

# Add conventional sources
add_files -scan_for_includes ../rtl

# Add IPs
add_files -norecurse -scan_for_includes ../rtl/ip/xilinx/PLL_2019_2/clk_pll.xci
add_files -norecurse -scan_for_includes ../rtl/ip/xilinx/axi_crossbar/axi_crossbar.xci
add_files -norecurse -scan_for_includes ../rtl/ip/xilinx/x2h/x2h.xci
add_files -norecurse -scan_for_includes ../rtl/ip/xilinx/x2p/x2p.xci
add_files -norecurse -scan_for_includes ../rtl/ip/xilinx/x2xl/x2xl.xci

# Add simulation files
add_files -fileset sim_1 ../sim/

# Add constraints
add_files -fileset constrs_1 -quiet ./constraints

# Set global include
set_property is_global_include true [get_files  ../rtl/ip/dma_axi32/dma_axi32_defines.v]

set_property -name "top" -value "tb_top" -objects  [get_filesets sim_1]
set_property -name {xsim.simulate.log_all_signals} -value {true} -objects [get_filesets sim_1]
