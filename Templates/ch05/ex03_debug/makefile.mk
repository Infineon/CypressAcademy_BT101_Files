APP_SRC +=  ex02_debug.c

ifdef DEBUG
C_FLAGS += -DDEBUG
C_FLAGS += -DSMUX_CHIP=$(CHIP)
APP_SRC += ex03_debug_pin_config.c
endif

