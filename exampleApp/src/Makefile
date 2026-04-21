TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += Example

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install ExampleSupport.dbd into <top>/dbd
DBD += ExampleSupport.dbd

ExampleSupport_DBD += Example.dbd

# specify all source files to be compiled and added to the library
Example_SRCS += ExampleDriver.cpp

Example_LIBS += motor asyn
Example_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

