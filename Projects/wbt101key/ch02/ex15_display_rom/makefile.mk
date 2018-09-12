APP_SRC = ex15_display_rom.c

C_FLAGS += -DWICED_BT_TRACE_ENABLE

# Include the freetronic SSD1306 128x64 display driver
APP_SRC	+= drivers/u8g_dev_ssd1306_128x64.c

# Include the Freetronix I2C communication driver
APP_SRC += drivers/u8g_com_freetronic_i2c.c

# Include all fonts used by the application
APP_SRC += drivers/fntsrc/u8g_font_unifont.c
APP_SRC += drivers/fntsrc/u8g_font_courr14.c

# Specify the graphic_lib interface to the ROMmed u8g lib
APP_PATCHES_AND_LIBS += graphic_lib.a
