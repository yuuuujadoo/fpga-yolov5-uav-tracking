set project_name "fpga_yolov5_uav_tracking"
set part_name "xc7vx485tffg1761-2"
set board_part_name "xilinx.com:vc707:part0:1.4"

set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ".." ".."]]
set build_dir [file join $repo_root "build" "vivado"]
set project_dir [file join $build_dir $project_name]

file mkdir $build_dir
create_project $project_name $project_dir -part $part_name -force

if {[catch {set_property board_part $board_part_name [current_project]} board_msg]} {
    puts "Warning: could not set board_part to $board_part_name: $board_msg"
}

proc add_globbed_files {fileset_name pattern} {
    set files [glob -nocomplain $pattern]
    if {[llength $files] > 0} {
        add_files -fileset $fileset_name -norecurse $files
    } else {
        puts "Warning: no files matched $pattern"
    }
}

add_globbed_files sources_1 [file join $repo_root "hardware" "rtl" "common" "*.v"]
add_globbed_files sources_1 [file join $repo_root "hardware" "rtl" "ethernet" "*.v"]
add_globbed_files sources_1 [file join $repo_root "hardware" "rtl" "npu" "*.v"]
add_globbed_files sources_1 [file join $repo_root "hardware" "rtl" "top" "*.v"]
add_globbed_files sources_1 [file join $repo_root "hardware" "mem" "*.mem"]

set ip_file [file join $repo_root "hardware" "ip" "gig_ethernet_pcs_pma_0" "gig_ethernet_pcs_pma_0.xci"]
if {[file exists $ip_file]} {
    read_ip $ip_file
    generate_target all [get_ips gig_ethernet_pcs_pma_0]
} else {
    puts "Warning: missing IP file $ip_file"
}

add_files -fileset constrs_1 -norecurse [file join $repo_root "hardware" "constraints" "vc707_eth_npu.xdc"]
add_globbed_files sim_1 [file join $repo_root "hardware" "sim" "*.v"]

set_property top vc707_eth_npu_top [get_filesets sources_1]
set_property top tb_full_infer_640 [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "Created Vivado project at $project_dir"
puts "Open: $project_dir/$project_name.xpr"
