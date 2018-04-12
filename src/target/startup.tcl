# Defines basic Tcl procs for OpenOCD target module

proc new_target_name { } {
	return [target number [expr [target count] - 1 ]]
}

global in_process_reset
set in_process_reset 0

# Catch reset recursion
proc ocd_process_reset { MODE } {
	global in_process_reset
	global arp_reset_mode
	if {$in_process_reset} {
		set in_process_reset 0
		return -code error "'reset' can not be invoked recursively"
	}

	set in_process_reset 1
	set arp_reset_mode $MODE
	set success [expr [catch {ocd_process_reset_inner $MODE} result]==0]
	set in_process_reset 0

	if {$success} {
		return $result
	} else {
		return -code error $result
	}
}


proc arp_is_tap_enabled { target } {
	if (![using_jtag]) {
		return 1
	}
	return [jtag tapisenabled [$target cget -chain-position]]
}

# duplicate of target_examine_one(), keep in sync
proc arp_examine_one { target } {
	if [arp_is_tap_enabled $target] {
		$target invoke-event examine-start
		set err [catch "$target arp_examine allow-defer"]
		if { $err } {
			$target invoke-event examine-fail
		} else {
			$target invoke-event examine-end
		}
	}
}


proc arp_reset_plan_no_srst { phase target } {
	global arp_reset_mode
	switch $phase {
		pre {
			arp_examine_one $target
		}
		middle {
			$target arp_reset trigger $arp_reset_mode
		}
		post {
			$target arp_reset post_deassert $arp_reset_mode
		}
	}
}

proc arp_reset_plan_srst_dbg_Working { phase target } {
	global arp_reset_mode
	switch $phase {
		pre {
			# srst_nogate: SRST is asserted now
			# srst_gates_jtag: SRST has not been asserted yet
		}
		middle {
			# SRST is asserted, target is responsive
			arp_examine_one $target
			$target arp_reset prepare $arp_reset_mode
		}
		post {
			$target arp_reset post_deassert $arp_reset_mode
		}
	}
}

proc arp_reset_plan_srst_dbg_gated { phase target } {
	global arp_reset_mode
	switch $phase {
		pre {
			# SRST has not been asserted yet
			# srst_nogate mode is not supported
			$target arp_reset prepare $arp_reset_mode
		}
		middle {
			# SRST is asserted, target debug gated
		}
		post {
			arp_examine_one $target
			$target arp_reset post_deassert $arp_reset_mode
		}
	}
}

proc arp_reset_plan_srst_dbg_cleared { phase target } {
	global arp_reset_mode
	switch $phase {
		pre {
			# target debug is going to be cleared
			# no point to pre-configure is
		}
		middle {
			# SRST is asserted, target debug not accessible
			if { [using_hla] } {
				# hla_target controls SRST on its own and even worse
				# an hl adapter may not export SRST control in API (ST-Link 1)
				# The only possible workaround is to let the target control SRST
				$target arp_reset assert $arp_reset_mode
			} else {
				$target arp_reset clear_internal_state $arp_reset_mode
			}
		}
		post {
			arp_examine_one $target
			$target arp_reset post_deassert $arp_reset_mode
		}
	}
}

proc arp_reset_default_handler { phase target } {
	if [arp_is_tap_enabled $target] {
	        set dbg_u_srst [$target cget -dbg-under-srst]
		if [reset_config_includes srst] {
			switch $dbg_u_srst {
				working {
					arp_reset_plan_srst_dbg_Working $phase $target
				}
				gated {
					arp_reset_plan_srst_dbg_gated $phase $target
				}
				cleared {
					arp_reset_plan_srst_dbg_cleared $phase $target
				}
				unknown {
					if [reset_config_includes srst_nogate] {
						arp_reset_plan_srst_dbg_Working $phase $target
					} else {
						arp_reset_plan_srst_dbg_gated $phase $target
					}
				}
			}
		} else {
			arp_reset_plan_no_srst $phase $target
		}
	}
}

proc arp_reset_halt_default_handler { target } {
	# Wait upto 1 second for target to halt.  Why 1sec? Cause
	# the JTAG tap reset signal might be hooked to a slow
	# resistor/capacitor circuit - and it might take a while
	# to charge
	catch { $target arp_waitstate halted 1000 }
}

# Utility to make 'reset halt' work as reset;halt on a target
# It does not prevent running code after reset
proc arp_reset_simple_halter { target } {
	$target arp_poll
	set st [$target curstate]
	if { $st eq "reset" } {
		# we assume running state follows
		# if reset accidentaly halt, waiting is useless
		catch { $target arp_waitstate running 1000 }
		set st [$target curstate]
	}
	if { $st eq "running" } {
		echo "$target: Ran after reset and before halt..."
		$target arp_halt
	}
	arp_reset_halt_default_handler $target
}

proc ocd_process_reset_inner { MODE } {
	set targets [target names]

	# If this target must be halted...
	set halt -1
	if { 0 == [string compare $MODE halt] } {
		set halt 1
	}
	if { 0 == [string compare $MODE init] } {
		set halt 1;
	}
	if { 0 == [string compare $MODE run ] } {
		set halt 0;
	}
	if { $halt < 0 } {
		return -code error "Invalid mode: $MODE, must be one of: halt, init, or run";
	}

	# Target event handlers *might* change which TAPs are enabled
	# or disabled, so we fire all of them.  But don't issue any
	# target "arp_*" commands, which may issue JTAG transactions,
	# unless we know the underlying TAP is active.
	#
	# NOTE:  ARP == "Advanced Reset Process" ... "advanced" is
	# relative to a previous restrictive scheme

	foreach t $targets {
		$t invoke-event reset-start
	}

	# If srst_nogate is set, check all targets whether they support it
	if {[reset_config_includes srst srst_nogate]} {
		foreach t $targets {
			set dbg_u_srst [$t cget -dbg-under-srst]
			if {$dbg_u_srst eq "gated"} {
				reset_config srst_gates_jtag
				echo "'srst_nogate' is not supported by at least target $t"
				echo "Reset config changed to 'srst_gates_jtag'"
				break;
			}
		}
	}
        set early_reset_init [expr {[reset_config_includes independent_trst]
				    || [reset_config_includes srst srst_nogate]}]

	if $early_reset_init {
		# We have an independent trst or no-gating srst

		# Use TRST or TMS/TCK operations to reset all the tap controllers.
		# TAP reset events get reported; they might enable some taps.
		init_reset $MODE
	}

	foreach t $targets {
		$t invoke-event reset-assert-pre
	}

	# Assert SRST
	reset_assert_final $MODE

	foreach t $targets {
		$t invoke-event reset-assert-post
	}

	foreach t $targets {
		$t invoke-event reset-deassert-pre
	}

	# Deassert SRST
	reset_deassert_initial $MODE
	if { !$early_reset_init } {
		if [using_jtag] { jtag arp_init }
	}

	foreach t $targets {
		$t invoke-event reset-deassert-post
	}

	# Pass 1 - Now wait for any halt (requested as part of reset
	# assert/deassert) to happen.  Ideally it takes effect without
	# first executing any instructions.
	if { $halt } {
		foreach t $targets {
			if {![arp_is_tap_enabled $t]} {
				continue
			}

			# don't wait for targets where examination is deferred
			# they can not be halted anyway at this point
			if { ![$t was_examined] && [$t examine_deferred] } {
				continue
			}

			$t invoke-event reset-halt

			# Did we succeed?
			if { [$t curstate] ne "halted" } {
				return -code error [format "TARGET: %s - Not halted" $t]
			}
		}
	}

	#Pass 2 - if needed "init"
	if { 0 == [string compare init $MODE] } {
		foreach t $targets {
			if {![arp_is_tap_enabled $t]} {
				continue
			}

			# don't wait for targets where examination is deferred
			# they can not be halted anyway at this point
			if { ![$t was_examined] && [$t examine_deferred] } {
				continue
			}

			$t invoke-event reset-init
		}
	}

	foreach t $targets {
		$t invoke-event reset-end
	}
}

proc using_jtag {} {
	set _TRANSPORT [ transport select ]
	expr { [ string equal "jtag" $_TRANSPORT ] != 0 || [ string equal "hla_jtag" $_TRANSPORT ] != 0 }
}

proc using_swd {} {
	set _TRANSPORT [ transport select ]
	expr { [ string equal "swd" $_TRANSPORT ] != 0 || [ string equal "hla_swd" $_TRANSPORT ] != 0}
}

proc using_hla {} {
	set _TRANSPORT [ transport select ]
	expr { [ string first "hla" $_TRANSPORT ] != -1 }
}

proc using_stlink_jtag {} {
	set _TRANSPORT [ transport select ]
	expr { [ string equal "stlink_jtag" $_TRANSPORT ] != 0 }
}

proc using_stlink_swd {} {
	set _TRANSPORT [ transport select ]
	expr { [ string equal "stlink_swd" $_TRANSPORT ] != 0 }
}

#########

# Temporary migration aid.  May be removed starting in January 2011.
proc armv4_5 params {
	echo "DEPRECATED! use 'arm $params' not 'armv4_5 $params'"
	arm $params
}

# Target/chain configuration scripts can either execute commands directly
# or define a procedure which is executed once all configuration
# scripts have completed.
#
# By default(classic) the config scripts will set up the target configuration
proc init_targets {} {
}

proc set_default_target_event {t e s} {
	if {[$t cget -event $e] == ""} {
		$t configure -event $e $s
	}
}

proc init_target_events {} {
	set targets [target names]

	foreach t $targets {
		set_default_target_event $t gdb-flash-erase-start "reset init"
		set_default_target_event $t gdb-flash-write-end "reset halt"
		set_default_target_event $t gdb-attach "halt"
		set_default_target_event $t reset-assert-pre "arp_reset_default_handler pre $t"
		set_default_target_event $t reset-assert-post "arp_reset_default_handler middle $t"
		set_default_target_event $t reset-deassert-post "arp_reset_default_handler post $t"
		set_default_target_event $t reset-halt "arp_reset_halt_default_handler $t"
	}
}

# Additionally board config scripts can define a procedure init_board that will be executed after init and init_targets
proc init_board {} {
}

# deprecated target name cmds
proc cortex_m3 args {
	echo "DEPRECATED! use 'cortex_m' not 'cortex_m3'"
	eval cortex_m $args
}

proc cortex_a8 args {
	echo "DEPRECATED! use 'cortex_a' not 'cortex_a8'"
	eval cortex_a $args
}
