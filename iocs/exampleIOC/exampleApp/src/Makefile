TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = example
# example.dbd will be created and installed
DBD += example.dbd

# example.dbd will be made up from these files:
example_DBD += base.dbd

# Include dbd files from all support applications:
example_DBD += asyn.dbd 
example_DBD += drvAsynIPPort.dbd
example_DBD += motorSupport.dbd
example_DBD += ExampleSupport.dbd

# Add all the support libraries needed by this IOC
example_LIBS += Example
example_LIBS += motor
example_LIBS += asyn

# The std module provides the doAfterIocInit iocsh command
ifdef STD
  example_DBD += stdSupport.dbd
  example_LIBS := std $(example_LIBS)
endif

# example_registerRecordDeviceDriver.cpp derives from example.dbd
example_SRCS += example_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
example_SRCS_DEFAULT += exampleMain.cpp
example_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#example_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
example_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

