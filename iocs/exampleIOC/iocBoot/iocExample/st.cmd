#!../../bin/linux-x86_64/example

< envPaths

cd "${TOP}/iocBoot/${IOC}"

## Register all support components
dbLoadDatabase "$(TOP)/dbd/example.dbd"
example_registerRecordDeviceDriver pdbbase

# Define the IOC prefix
< settings.iocsh

# Allstop, alldone
iocshLoad("$(MOTOR)/iocsh/allstop.iocsh", "P=$(PREFIX)")

## Example Motor Controller
< example.iocsh

iocInit

# Boot complete
