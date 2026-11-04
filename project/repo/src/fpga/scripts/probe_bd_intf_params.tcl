# ============================================================================
# 探针：BD 41-237 / BD 41-3281 的接口参数回读与「是否可修」判定
#
# 背景（2026-09-29 第一次抓取的告警原文）：
#   BD 41-3281 x5: Instance '/zynq_ps' 等「connected on both sides by SmartConnects and
#                cannot automatically configure itself to match the endpoints.
#                Please manually configure its AXI interfaces as required for the design.」
#   BD 41-237 x2 : Bus Interface property AWUSER_WIDTH / ARUSER_WIDTH does not match
#                between /zynq_ps/S_AXI_HP0_FPD(1) and /sctl_mem/M00_AXI(0)
#
# 本脚本回答三件事：
#   1. 两侧的 width / user / id 类参数**实测各是多少**（把「不匹配」变成具体数字）；
#   2. 那些参数在 SmartConnect 侧**是否可写**（set_property 的 rc）；
#   3. 试写之后重新校验，**41-237 是否消失** —— 用「告警消失」而不是「看起来对」当判据。
#
# 只改内存中的 BD，**不保存**（save_bd_design 不调用）：先证明能不能修，再决定要不要落到
# penumbrasight_bd.tcl 里。不跑 OOC 综合（内存不够）。
#
# 全部输出为 ASCII，避免控制台按 GBK 解码时中文变乱码（本项目已踩过一次）。
# ============================================================================

set REPO_ROOT [file normalize [file join [file dirname [info script]] .. .. ..]]
if {[info exists ::env(PS_REPO_ROOT)] && $::env(PS_REPO_ROOT) ne ""} {
    set REPO_ROOT [file normalize $::env(PS_REPO_ROOT)]
}
set BD_PROJ [file normalize [file join $REPO_ROOT build bd_project bd_project.xpr]]

open_project $BD_PROJ
open_bd_design [lindex [get_files -quiet *.bd] 0]
puts "PROBE2_BD_OPENED"

# ---------------------------------------------------------------- 1 参数回读
proc dump_widths {pin} {
    puts "PIN $pin"
    set found 0
    foreach prop [lsort [list_property $pin]] {
        if {![regexp -nocase {width|user|_id|id_|burst} $prop]} { continue }
        set v "?"
        catch { set v [get_property $prop $pin] }
        if {$v eq ""} { continue }
        puts "    $prop = $v"
        incr found
    }
    if {!$found} { puts "    <no width/user/id property exposed>" }
}

puts "=== A) the two endpoints of the BD 41-237 pair ==="
foreach pin {/zynq_ps/S_AXI_HP0_FPD /sctl_mem/M00_AXI} {
    if {[llength [get_bd_intf_pins -quiet $pin]]} { dump_widths [get_bd_intf_pins $pin] }
}

puts "=== B) PS master + the two SmartConnects' PS-facing side ==="
foreach pin {/zynq_ps/M_AXI_HPM0_FPD /sctl_ctrl/S00_AXI /sctl_mem/S00_AXI} {
    if {[llength [get_bd_intf_pins -quiet $pin]]} { dump_widths [get_bd_intf_pins $pin] }
}

puts "=== C) one core: both slave ifaces + its m_axi masters ==="
foreach pin [get_bd_intf_pins -quiet /core_pdhg/*] { dump_widths $pin }

puts "=== D) SmartConnect: all properties (looking for settable width knobs) ==="
foreach pin {/sctl_mem/M00_AXI} {
    foreach prop [lsort [list_property [get_bd_intf_pins $pin]]] {
        set v "?"
        catch { set v [get_property $prop [get_bd_intf_pins $pin]] }
        if {$v eq ""} { continue }
        puts "    $pin : $prop = $v"
    }
}

# ---------------------------------------------------------------- 2 试写：这些钮可写吗
puts "=== E) try to set the mismatching widths on the SmartConnect master ==="
foreach {prop val} {CONFIG.AWUSER_WIDTH 0 CONFIG.ARUSER_WIDTH 0} {
    set rc "-"
    set msg ""
    set rc [catch {set_property $prop $val [get_bd_intf_pins /sctl_mem/M00_AXI]} msg]
    puts "TRY_SET $prop=$val rc=$rc"
    if {$rc} { puts "TRY_SET_MSG $msg" }
}

# ---------------------------------------------------------------- 3 试写后重校验：41-237 还在吗
puts "=== G) path-by-path width pairing: SmartConnect side vs the peer pin ==="
# 目的：把「工具说它没法自动匹配」变成「每一对端口实测各是多少」。
# 41-3281 只说「无法自动配置」，不说哪里不一致；只有逐对回读才能证明**该一致的地方一致**。
puts "--- G1 control: 8 slave ports of sctl_ctrl ---"
foreach mi {M00 M01 M02 M03 M04 M05 M06 M07} {
    set p [get_bd_intf_pins -quiet /sctl_ctrl/${mi}_AXI]
    if {![llength $p]} { continue }
    set d "?"; set a "?"; set i "?"
    catch { set d [get_property CONFIG.DATA_WIDTH $p] }
    catch { set a [get_property CONFIG.ADDR_WIDTH $p] }
    catch { set i [get_property CONFIG.ID_WIDTH   $p] }
    set peers {}
    foreach n [get_bd_intf_nets -quiet -of_objects $p] {
        foreach q [get_bd_intf_pins -quiet -of_objects $n] {
            if {$q eq $p} { continue }
            set pd "?"; set pa "?"
            catch { set pd [get_property CONFIG.DATA_WIDTH $q] }
            catch { set pa [get_property CONFIG.ADDR_WIDTH $q] }
            lappend peers "$q (DATA=$pd ADDR=$pa)"
        }
    }
    puts "P_CTRL $mi DATA=$d ADDR=$a ID=$i -> $peers"
}

puts "--- G2 data: 11 slave ports of sctl_mem ---"
foreach si [get_bd_intf_pins -quiet /sctl_mem/S*_AXI] {
    set d "?"; set a "?"; set i "?"
    catch { set d [get_property CONFIG.DATA_WIDTH $si] }
    catch { set a [get_property CONFIG.ADDR_WIDTH $si] }
    catch { set i [get_property CONFIG.ID_WIDTH   $si] }
    set peers {}
    foreach n [get_bd_intf_nets -quiet -of_objects $si] {
        foreach q [get_bd_intf_pins -quiet -of_objects $n] {
            if {$q eq $si} { continue }
            set pd "?"; set pa "?"; set pb "?"
            catch { set pd [get_property CONFIG.DATA_WIDTH $q] }
            catch { set pa [get_property CONFIG.ADDR_WIDTH $q] }
            catch { set pb [get_property CONFIG.HAS_BURST  $q] }
            lappend peers "$q (DATA=$pd ADDR=$pa HAS_BURST=$pb)"
        }
    }
    puts "P_MEM  $si DATA=$d ADDR=$a ID=$i -> $peers"
}

puts "=== H) re-validate after the trial property writes ==="
set rc [catch {validate_bd_design -force} err]
puts "REVALIDATE rc=$rc"
if {$rc} { puts "REVALIDATE_MSG $err" }

puts "PROBE2_DONE"
close_project
