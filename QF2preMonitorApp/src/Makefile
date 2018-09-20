TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += QF2preMonitor

# install QF2preMonitor.dbd into <top>/dbd
DBD += QF2preMonitor.dbd

# specify all source files to be compiled and added to the library
QF2preMonitor_SRCS += QF2preMonitor.c

#ifneq ($(ASYN),)
QF2preMonitor_DBD += asyn.dbd
QF2preMonitor_LIBS += asyn
#endif

QF2preMonitor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

