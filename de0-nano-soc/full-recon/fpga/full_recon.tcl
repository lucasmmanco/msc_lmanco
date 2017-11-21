# Copyright (C) 2016  Intel Corporation. All rights reserved.
# Your use of Intel Corporation's design tools, logic functions 
# and other software and tools, and its AMPP partner logic 
# functions, and any output files from any of the foregoing 
# (including device programming or simulation files), and any 
# associated documentation or information are expressly subject 
# to the terms and conditions of the Intel Program License 
# Subscription Agreement, the Intel Quartus Prime License Agreement,
# the Intel MegaCore Function License Agreement, or other 
# applicable license agreement, including, without limitation, 
# that your use is for the sole purpose of programming logic 
# devices manufactured by Intel and sold by Intel or its 
# authorized distributors.  Please refer to the applicable 
# agreement for further details.

# Quartus Prime: Generate Tcl File for Project
# File: full_recon.tcl
# Generated on: Mon Oct 16 23:14:17 2017

# Load Quartus Prime Tcl Project package
package require ::quartus::project

set need_to_close_project 0
set make_assignments 1

# Check that the right project is open
if {[is_project_open]} {
	if {[string compare $quartus(project) "full_recon"]} {
		puts "Project full_recon is not open"
		set make_assignments 0
	}
} else {
	# Only open if not already open
	if {[project_exists full_recon]} {
		project_open -revision full_recon full_recon
	} else {
		project_new -revision full_recon full_recon
	}
	set need_to_close_project 1
}

# Make assignments
if {$make_assignments} {
	set_global_assignment -name PROJECT_IP_REGENERATION_POLICY ALWAYS_REGENERATE_IP
	set_global_assignment -name SEARCH_PATH work
    set_global_assignment -name VHDL_FILE blink_led.vhd -library work
	set_global_assignment -name CDF_FILE full_recon.cdf
    set_global_assignment -name QSF_FILE full_recon.qsf

    	# Commit assignments
	export_assignments

	# Close project
	if {$need_to_close_project} {
		project_close
	}
}
