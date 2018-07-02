 # =========================================================================
 # The Synopsys DWC ETHER QOS Software Driver and documentation (hereinafter
 # "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 # otherwise expressly agreed to in writing between Synopsys and you.
 #
 # The Software IS NOT an item of Licensed Software or Licensed Product under
 # any End User Software License Agreement or Agreement for Licensed Product
 # with Synopsys or any supplement thereto.  Permission is hereby granted,
 # free of charge, to any person obtaining a copy of this software annotated
 # with this license and the Software, to deal in the Software without
 # restriction, including without limitation the rights to use, copy, modify,
 # merge, publish, distribute, sublicense, and/or sell copies of the Software,
 # and to permit persons to whom the Software is furnished to do so, subject
 # to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 # ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 # INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 # SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 # OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 # DAMAGE.
 # =========================================================================
 # =========================================================================
 #  Copyright 2018 NXP
 # 
 # Redistribution and use in source and binary forms, with or without 
 # modification, are permitted provided that the following conditions are met:
 # 
 # 1. Redistributions of source code must retain the above copyright notice,
 #    this list of conditions and the following disclaimer.
 # 
 # 2. Redistributions in binary form must reproduce the above copyright notice,
 #    this list of conditions and the following disclaimer in the documentation 
 #    and/or other materials provided with the distribution.
 # 
 # 3. Neither the name of the copyright holder nor the names of its contributors
 #    may be used to endorse or promote products derived from this software
 #    without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 # THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 # PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 # OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 # OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 # WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 # OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 # ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 # =========================================================================
 
export PGTEST CONFIG_PGTEST_OBJ DWC_ETH_QOS_CONFIG_PGTEST
export DEBUGFS CONFIG_DEBUGFS_OBJ DWC_ETH_QOS_CONFIG_DEBUGFS
export PTPSUPPORT CONFIG_PTPSUPPORT_OBJ DWC_ETH_QOS_CONFIG_PTP

#default values
#DEBUGFS=y	#debugfs is enabled
#PTPSUPPORT=n	#ptp is disabled
#PGTEST=n	#pg(packet generator) is disabled	

ifeq "$(PGTEST)" "y"
CONFIG_PGTEST_OBJ=y
DWC_ETH_QOS_CONFIG_PGTEST=-DPGTEST
EXTRA_CFLAGS+=-DCONFIG_PGTEST_OBJ
else
CONFIG_PGTEST_OBJ=n
endif

ifeq "$(DEBUGFS)" "y"
CONFIG_DEBUGFS_OBJ=y
DWC_ETH_QOS_CONFIG_DEBUGFS=-DDEBUGFS
EXTRA_CFLAGS+=-DCONFIG_DEBUGFS_OBJ
else
CONFIG_DEBUGFS_OBJ=n
endif

ifeq "$(PTPSUPPORT)" "y"
CONFIG_PTPSUPPORT_OBJ=y
DWC_ETH_QOS_CONFIG_PTP=-DPTPSUPPORT
EXTRA_CFLAGS+=-DCONFIG_PTPSUPPORT_OBJ
else
CONFIG_PTPSUPPORT_OBJ=n
endif

ifndef KDIR
	KDIR = /lib/modules/$(shell uname -r)/build
endif

obj-m := dwc-eqos.o

dwc-eqos-y += dwc-eqos-dev.o \
		dwc-eqos-drv.o \
		dwc-eqos-desc.o \
		dwc-eqos-ethtool.o \
		dwc-eqos-platform.o \
		dwc-eqos-mdio.o \
		dwc-eqos-eee.o

dwc-eqos-$(CONFIG_DEBUGFS_OBJ) += dwc-eqos-debug_operation.o
dwc-eqos-$(CONFIG_PGTEST_OBJ) += dwc-eqos-pktgen.o
dwc-eqos-$(CONFIG_PTPSUPPORT_OBJ) += dwc-eqos-ptp.o

all:
	make -C $(KDIR) M=$(PWD) modules
	#gcc -g -DDWC_ETH_QOS_CONFIG_PGTEST -DDWC_ETH_QOS_CONFIG_DEBUGFS -DDWC_ETH_QOS_CONFIG_PTP dwc-eqos-api.c -o dwc-eqos-api && mv dwc-eqos-api $(DWC_ETH_QOS_COMP_DRV_DEBUG_SCRIPTS)

clean:
	make -C $(KDIR) M=$(PWD) clean
	#rm -f $(DWC_ETH_QOS_COMP_DRV_DEBUG_SCRIPTS)/dwc-eqos-api

