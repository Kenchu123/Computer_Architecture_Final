# Import Design
read_file -format verilog  "./RISCV.v"

set DESIGN "RISCV"
current_design [get_designs RISCV]

link

source -echo -verbose ./RISCV_syn.sdc

# Compile Design
current_design [get_designs RISCV]
set high_fanout_net_threshold 0
uniquify
set_fix_multiple_port_nets -all -buffer_constants [get_designs *]

compile

# Output Design
current_design [get_designs RISCV]

remove_unconnected_ports -blast_buses [get_cells -hierarchical *]

set bus_inference_style {%s[%d]}
set bus_naming_style {%s[%d]}
set hdlout_internal_busses true
change_names -hierarchy -rule verilog
define_name_rules name_rule -allowed {a-z A-Z 0-9 _} -max_length 255 -type cell
define_name_rules name_rule -allowed {a-z A-Z 0-9 _[]} -max_length 255 -type net
define_name_rules name_rule -map {{"\\*cell\\*" "cell"}}
define_name_rules name_rule -case_insensitive
change_names -hierarchy -rules name_rule


#Save files
#####################################################
write -format ddc     -hierarchy -output ./Netlist/$DESIGN\_syn.ddc
write -format verilog -hierarchy -output ./Netlist/$DESIGN\_syn.v
write_sdf -version 2.1  ./Netlist/$DESIGN\_syn.sdf
write_sdc ./Netlist/$DESIGN\_syn.sdc -version 1.8

#####################################################  
report_design              >  ./Report/$DESIGN\.design
report_area     -hierarchy >  ./Report/$DESIGN\.area
report_resource -hierarchy >  ./Report/$DESIGN\.resource
report_power               >  ./Report/$DESIGN\.power
report_timing -delay min -max_paths 5 > ./Report/$DESIGN\.timing_min
report_timing -delay max -max_paths 5 > ./Report/$DESIGN\.timing_max
report_constraint -all_violators >  ./Report/$DESIGN\.constraint
write_parasitics -output ./Netlist/pd_MEMC.spef