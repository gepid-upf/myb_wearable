#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_ADD_INCLUDEDIRS := . clib
COMPONENT_SRCDIRS := $(COMPONENT_ADD_INCLUDEDIRS)

# make do motter
ULP_APP_NAME ?= ulp_$(COMPONENT_NAME)
ULP_S_SOURCES = $(COMPONENT_PATH)/ulp/ulp.S
ULP_EXP_DEP_OBJECTS := main.o
include $(IDF_PATH)/components/ulp/component_ulp_common.mk

#COMPONENT_SRCDIRS:=csrc
#COMPONENT_ADD_INCLUDEDIRS:=csrc
