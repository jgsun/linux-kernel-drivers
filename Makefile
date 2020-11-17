#
# Kbuild rules (used by the kernel Kbuild infrastructure, both when building
# from within as outside of buildroot)
#
subdir-ccflags-y += -DDEBUG -Wall -Werror -I$(src)/generic -I$(src)/cpld -I$(src)/misc

# Descend in sub-directories
obj-m += generic/ cpld/ reboot_helper/ misc/ dma_alloc/ remoteproc/ net/ eth_insert_extract/

ifeq ($(BUILD_TEST_DRIVERS),y)
obj-m += tests/
endif

#
# Standard make rules (only used when building outside of Buildroot)
#
ifneq (,$(BUILDROOT))
# On some machines (Ubuntu), /.config exists as a directory
-include $(BUILDROOT)/.config
endif

BUILDROOT_OUTPUT=$(BUILDROOT)/output
ARCH := $(subst ",,$(BR2_ARCH))
# Re-correct syntax highlighting: ")
ifneq ($(strip $(BR2_bcm963xx)),y)

# Taken from <buildroot>/Makefile
# KERNEL_ARCH:=$(shell echo "$(ARCH)" | sed -e "s/-.*//" \
#         -e s/i.86/i386/ -e s/sun4u/sparc64/ \
#         -e s/arcle/arc/ \
#         -e s/arcbe/arc/ \
#         -e s/arm.*/arm/ -e s/sa110/arm/ \
#         -e s/aarch64/arm64/ \
#         -e s/bfin/blackfin/ \
#         -e s/parisc64/parisc/ \
#         -e s/powerpc64/powerpc/ \
#         -e s/ppc.*/powerpc/ -e s/mips.*/mips/ \
#         -e s/sh.*/sh/)
#
# TOOLS_PREFIX := $(BR2_TOOLCHAIN_EXTERNAL_PREFIX)-
# KERNEL_TARGET_CROSS:=$(BUILDROOT_OUTPUT)/host/usr/bin/$(TOOLS_PREFIX)
# else
# specific steps for bcm963xx (dual toolchain)
KERNEL_ARCH:=arm64
AARCH64_TOOLCHAIN_PREFIX:=$(subst arm,aarch64,$(BR2_TOOLCHAIN_EXTERNAL_PREFIX))
KERNEL_TARGET_CROSS:=$(BUILDROOT_OUTPUT)/host/opt/ext-toolchain/bin/$(AARCH64_TOOLCHAIN_PREFIX)-
# endif

OPTIONS= -C $(wildcard $(BUILDROOT_OUTPUT)/build/linux*) M=$(PWD) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_TARGET_CROSS)

all: check_vars build install

check_vars:
	@if [ ! -d "$(BUILDROOT)" ]; then (echo "Directory '$(BUILDROOT)' does not exist. Please specify a valid BUILDROOT on the command line"; false); fi

build:
	$(MAKE) $(OPTIONS) modules

clean:
	$(MAKE) $(OPTIONS) clean

install:
	$(MAKE) $(OPTIONS) INSTALL_MOD_PATH=$(BUILDROOT_OUTPUT)/target INSTALL_MOD_STRIP=1 INSTALL_MOD_DIR=isam modules_install

headers_install:
	install -D -m 0644 generic/uio_generic_ioctl.h $(DESTDIR)/usr/include/linux
	install -D -m 0644 misc/gin_vp_ioctl.h $(DESTDIR)/usr/include/linux
