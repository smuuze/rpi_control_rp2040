#-----------------------------------------------------------------------------

ifneq '' '$(findstring RTC,$(DRIVER_MODULE_CFG))'
	DEFS += -D HAS_DRIVER_RTC0=1
	CSRCS += $(SOC_PATH)/src/driver/rtc_driver_rp2040.c
endif

#-----------------------------------------------------------------------------

ifneq '' '$(findstring CLK,$(DRIVER_MODULE_CFG))'
	CSRCS += $(SOC_PATH)/src/driver/clock_driver_rp2040.c
endif

#-----------------------------------------------------------------------------

ifneq '' '$(findstring IRQ,$(DRIVER_MODULE_CFG))'
	DEFS += -D HAS_DRIVER_IRQ=1
	CSRCS += $(SOC_PATH)/src/driver/irq_driver_rp2040.c
endif

#-----------------------------------------------------------------------------

ifneq '' '$(findstring GPIO,$(DRIVER_MODULE_CFG))'
	DEFS += -D HAS_DRIVER_GPIO=1
	CSRCS += $(SOC_PATH)/src/driver/gpio_driver_rp2040.c
endif

#-----------------------------------------------------------------------------

ifneq '' '$(findstring I2C0,$(DRIVER_MODULE_CFG))'
	#DEFS += -D HAS_DRIVER_I2C0=1
	#CSRCS += $(SOC_PATH)/src/driver/i2c_driver_rp2040.c
endif

#-----------------------------------------------------------------------------

ifneq '' '$(findstring SPI0,$(DRIVER_MODULE_CFG))'
	DEFS += -D HAS_DRIVER_SPI0=1
	CSRCS += $(SOC_PATH)/src/driver/spi_driver_rp2040.c
endif

#-----------------------------------------------------------------------------

ifneq '' '$(findstring USART0,$(DRIVER_MODULE_CFG))'
	DEFS += -D HAS_DRIVER_USART0=1
	RP2040_UART_AVAILABLE = TRUE
endif

ifneq '' '$(findstring USART1,$(DRIVER_MODULE_CFG))'
	DEFS += -D HAS_DRIVER_USART1=1
	RP2040_UART_AVAILABLE = TRUE
endif

ifdef RP2040_UART_AVAILABLE
	CSRCS += $(SOC_PATH)/src/driver/usart_driver_rp2040.c
endif

#-----------------------------------------------------------------------------

