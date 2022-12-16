#-----------------------------------------------------------------------------

INC_PATH += $(SOC_PATH)/src/cpu/

#-----------------------------------------------------------------------------

include $(FRMWRK_PATH)/make/make_cpu_arm.mk

#-----------------------------------------------------------------------------

MCU_NAME 	= rp2040
DEFS 		+= -D__rp2040__
MCU_FLAG 	= -mcpu=cortex-m0plus

#-----------------------------------------------------------------------------

MEMORY_LAYOUT_FILE ?= $(SOC_PATH)/src/cpu/default_memory_layout.ld

#-----------------------------------------------------------------------------

#CFLAGS += --specs=rdimon.specs -ffunction-sections -fdata-sections
CFLAGS += -ffunction-sections -fdata-sections

#-----------------------------------------------------------------------------

CSRCS += $(SOC_PATH)/src/cpu/boot2.c
CSRCS += $(SOC_PATH)/src/cpu/crt0.c
CSRCS += $(SOC_PATH)/src/cpu/binary_info.c
CSRCS += $(SOC_PATH)/src/cpu/rp2040_reset.c

#-----------------------------------------------------------------------------

#ASRCS += $(SOC_PATH)/src/cpu/crt0.S

#-----------------------------------------------------------------------------
