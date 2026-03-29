# Project Name
TARGET = MySynthSeed

# Sources
CPP_SOURCES = \
	MySynthSeed.cpp \
	lib/Adafruit_NeoPixel/Adafruit_NeoPixel.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

LDFLAGS += -u _printf_float

C_INCLUDES += \
    -I. \
    -Ilib/Adafruit_NeoPixel
