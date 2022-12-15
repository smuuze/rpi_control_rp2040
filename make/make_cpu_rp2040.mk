
include $(FRMWRK_PATH)/make/make_cpu_arm.mk

MCU_NAME 	= rp2040
DEFS 		+= -D__rp2040__
MCU_FLAG 	= -mcpu=cortex-m0plus

MEMORY_LAYOUT_FILE ?= $(FRMWRK_PATH)/src/common/cpu/$(CPU_FAMILY)/$(MCU_NAME)/default_memory_layout.ld

#CFLAGS += --specs=rdimon.specs -ffunction-sections -fdata-sections
CFLAGS += -ffunction-sections -fdata-sections

CSRCS += $(FRMWRK_PATH)/src/common/cpu/$(CPU_FAMILY)/$(MCU_NAME)/boot2.c
CSRCS += $(FRMWRK_PATH)/src/common/cpu/$(CPU_FAMILY)/$(MCU_NAME)/crt0.c
CSRCS += $(FRMWRK_PATH)/src/common/cpu/$(CPU_FAMILY)/$(MCU_NAME)/binary_info.c
CSRCS += $(FRMWRK_PATH)/src/common/cpu/$(CPU_FAMILY)/$(MCU_NAME)/rp2040_reset.c

#ASRCS += $(FRMWRK_PATH)/src/common/cpu/$(CPU_FAMILY)/$(MCU_NAME)/crt0.S
