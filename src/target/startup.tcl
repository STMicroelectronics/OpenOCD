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

proc arp_examine_all {} {
	set targets [target names]

	# Examine all targets on enabled taps.
	foreach t $targets {
		if {![using_jtag] || [jtag tapisenabled [$t cget -chain-position]]} {
			$t invoke-event examine-start
			set err [catch "$t arp_examine allow-defer"]
			if { $err == 0 } {
				$t invoke-event examine-end
			}
		}
	}
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

	global arp_reset_halting
	set arp_reset_halting $halt

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
			if {[$t cget -dbg-under-srst] ne "working"} {
				reset_config srst_gates_jtag
				echo "'srst_nogate' is not supported by at least one target"
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

		# Examine all targets on enabled taps.
		arp_examine_all
	}

	foreach t $targets {
		$t invoke-event reset-assert-pre
	}

	# Prepare all targets with debug not working under SRST
	# Note: Preparing a target with debug cleared by SRST has no point
	# if SRST enabled
	foreach t $targets {
		set tapenabled [expr {![using_jtag] || [jtag tapisenabled [$t cget -chain-position]]}]
		if {$tapenabled && [$t cget -dbg-under-srst] ne "working"} {
			$t arp_reset assert $arp_reset_halting
		}
	}

	# Assert SRST
	reset_assert_final $MODE

	# Prepare other targets under SRST
	foreach t $targets {
		set tapenabled [expr {![using_jtag] || [jtag tapisenabled [$t cget -chain-position]]}]
		if {$tapenabled && [$t cget -dbg-under-srst] eq "working"} {
			$t arp_reset assert $arp_reset_halting
		}
	}
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
		arp_examine_all
	}
	foreach t $targets {
		if {![using_jtag] || [jtag tapisenabled [$t cget -chain-position]]} {
			$t arp_reset deassert $arp_reset_halting
		}
	}
	foreach t $targets {
		$t invoke-event reset-deassert-post
	}

	# Pass 1 - Now wait for any halt (requested as part of reset
	# assert/deassert) to happen.  Ideally it takes effect without
	# first executing any instructions.
	if { $arp_reset_halting } {
		foreach t $targets {
			if {[using_jtag] && ![jtag tapisenabled [$t cget -chain-position]]} {
				continue
			}

			# don't wait for targets where examination is deferred
			# they can not be halted anyway at this point
			if { ![$t was_examined] && [$t examine_deferred] } {
				continue
			}

			# Wait upto 1 second for target to halt.  Why 1sec? Cause
			# the JTAG tap reset signal might be hooked to a slow
			# resistor/capacitor circuit - and it might take a while
			# to charge

			# Catch, but ignore any errors.
			catch { $t arp_waitstate halted 1000 }

			# Did we succeed?
			set s [$t curstate]

			if { 0 != [string compare $s "halted" ] } {
				return -code error [format "TARGET: %s - Not halted" $t]
			}
		}
	}

	#Pass 2 - if needed "init"
	if { 0 == [string compare init $MODE] } {
		foreach t $targets {
			if {[using_jtag] && ![jtag tapisenabled [$t cget -chain-position]]} {
				continue
			}

			# don't wait for targets where examination is deferred
			# they can not be halted anyway at this point
			if { ![$t was_examined] && [$t examine_deferred] } {
				continue
			}

			set err [catch "$t arp_waitstate halted 5000"]
			# Did it halt?
			if { $err == 0 } {
				$t invoke-event reset-init
			}
		}
	}

	foreach t $targets {
		$t invoke-event reset-end
	}
}

proc using_jtag {} {
	set _TRANSPORT [ transport select ]
	expr { [ string first "jtag" $_TRANSPORT ] != -1 }
}

proc using_swd {} {
	set _TRANSPORT [ transport select ]
	expr { [ string first "swd" $_TRANSPORT ] != -1 }
}

proc using_hla {} {
	set _TRANSPORT [ transport select ]
	expr { [ string first "hla" $_TRANSPORT ] != -1 }
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
