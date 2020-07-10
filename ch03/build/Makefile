#################################################################
# PROJECT: DXL Protocol 2.0  Example Makefile
# AUTHOR : ROBOTIS Ltd.
# (https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c/example/protocol2.0/bulk_read_write/linux64/Makefile)
#
# This Project "DXL Protocol 2.0  Example Makefile" was 
# created by ROBOTIS LTD and was modified by RT Corporation in 
# accordance with the terms and conditions set forth in 
# Apache License 2.0. You may only use, reproduce and distribute 
# this Work or the Derivative Work developed by RT Corporation in
# compliance with Apache License 2.0.
#################################################################

#---------------------------------------------------------------------
# Makefile template for projects using DXL SDK
#
# Please make sure to follow these instructions when setting up your
# own copy of this file:
#
#   1- Enter the name of the target (the TARGET variable)
#   2- Add additional source files to the SOURCES variable
#   3- Add additional static library objects to the OBJECTS variable
#      if necessary
#   4- Ensure that compiler flags, INCLUDES, and LIBRARIES are
#      appropriate to your needs
#
#
# This makefile will link against several libraries, not all of which
# are necessarily needed for your project.  Please feel free to
# remove libaries you do not need.
#---------------------------------------------------------------------

# important directories used by assorted rules and other variables
DIR_DXL    = ../../../DynamixelSDK/c
DIR_OBJS   = .objects
DIR_COM    = ../common
DIR_BIN	   = ../bin

# *** ENTER THE TARGET NAME HERE ***
TARGET      = $(DIR_BIN)/crane_x7_test

# compiler options
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) #-Wl,-rpath,$(DIR_THOR)/lib
FORMAT      = -m64

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES   += -I$(DIR_DXL)/include/dynamixel_sdk
INCLUDES   += -I$(DIR_COM)
LIBRARIES  += -ldxl_x64_c
LIBRARIES  += -lrt

#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES  = main.c  \
           $(DIR_COM)/crane_x7_comm.c \
           $(DIR_COM)/arm_parameter.c \
           $(DIR_COM)/matrix.c \
		   myCX7_KDL_library.c \

OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))
#OBJETCS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***

#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/
	mkdir -p $(DIR_BIN)/

$(DIR_OBJS)/%.o: ../%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ../$(DIR_COM)/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ../%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
