APP_SRC = ex15_display.c

C_FLAGS += -DWICED_BT_TRACE_ENABLE

# Include the u8g library in the application (must be installed in "libraries/u8g_lib").
$(NAME)_COMPONENTS := u8g_lib.a

# Enable the fonts for your application.
# Look for the comment "font definitions" in u8g.h to see a list of fonts (u8g_font_*).
# To use the font call u8g_SetFont( &u8g, u8g_font_unifont );
#
# If you include too many fonts you will run out of memory on the device and your application will not build
C_FLAGS += -DUSE_FONT_u8g_font_unifont
