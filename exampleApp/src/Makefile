TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += example

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install example.dbd into <top>/dbd
DBD += example.dbd

# specify all source files to be compiled and added to the library
#example_SRCS += xxx

example_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

