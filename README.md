# Brief

Implementation of several driver interface of the [rpi-control-framework](https://github.com/smuuze/rpi_control_frmwrk).

# Usage

## Repository path

Clone this repository to your local machine and add the following statemnt to your projects makefile

```Makefile
SOC_PATH     = $(BASE_PATH)/rpi_control_rp2040
```
Also be sure to set the correct path to your copy of the rpi-control-framework in your projects makefile.
See the following statement.

## Framework path

```Makefile
BASE_PATH   = ../..
FRMWRK_PATH = $(BASE_PATH)/rpi_control_frmwrk
APP_PATH    = $(FRMWRK_PATH)/src
MAKE_PATH   = $(FRMWRK_PATH)/make

...

include $(MAKE_PATH)/common_make.mk
```

# Changelog

## 1.0.002

Date: not released
Autor: Seabstian Lesse

### New Features:

- Binary information is added automatically to program output, if configured via makefile and config.h

### Bug Fixes:

- Reset of GPIO sub-module (IOBANK0 and PADS0) on POR

### Misc:

- none

### Knwon Bugs:

- none

## 1.0.001

initilai release