

set_property -dict { PACKAGE_PIN A18 IOSTANDARD LVCMOS33 } [get_ports { btn1 }];
set_property -dict { PACKAGE_PIN B18 IOSTANDARD LVCMOS33 } [get_ports { btn2 }];
#set_property -dict { PACKAGE_PIN A17 IOSTANDARD LVCMOS33 } [get_ports { led1 }];
#set_property -dict { PACKAGE_PIN C16 IOSTANDARD LVCMOS33 } [get_ports { led2 }];

set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { clock }];

set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { SDA }];
set_property -dict { PACKAGE_PIN K18 IOSTANDARD LVCMOS33 } [get_ports { SCL }];