set_property PACKAGE_PIN J10 [get_ports fan]
set_property IOSTANDARD LVCMOS33 [get_ports fan]
set_property DRIVE 8 [get_ports fan]
# J10=0 keeps the AXU9EGB fan at full speed, independently of PS reset/software.
