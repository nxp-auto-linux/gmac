## Introduction of the GMAC Linux driver ##
The driver is in form of Linux kernel module.

The "GMAC Linux driver" was derived from "Synopsys DWC ETHER QOS Software Driver". 
The original code is an unsupported proprietary work of Synopsys. 
A permission is granted to users to re-use their code under given conditions.

The NXP "GMAC Linux driver" is released under BSD3 license. 

#### HW requirements ####
 * VDK running S32G simulation (developed with model S32G R5.0.0))

#### SW requirements ####
 * BSP 17.0 (linux-s32 kernel 4.1.26-rt29).
 * to probe correctly GMAC should be added to the device tree the following way:
    gmac0: ethernet@4033c000 {
            compatible = "fsl,dwc-eqos";
            reg = <0x0 0x4033c000 0x0 0x2000>;
            interrupt-parent = <&gic>;
            interrupts = <0 57 4>;
            interrupt-names = "macirq";
    };

##### Building the driver #####
 * the driver should be built as a standard kernel module (proper KDIR,ARCH,CROSS_COMPILE and TOOLCHAIN variables should be set)
 * several configuration options are available in the makefile

##### Installing the driver #####
 * the driver can be installed using insmod command
 * after installing the driver new eth interface is available in the system and can be configured with ifconfig command