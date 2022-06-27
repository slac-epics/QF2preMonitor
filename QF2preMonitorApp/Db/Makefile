TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
DB += QF2preMonitor.db

# Using LCLS-II conventions
DB += QF2preMonitorLCLS2.db
DB += perQF2preAlarmLCLS2.template
DB += QF2preUtilityLCLS2.db

# QSFP v3+ support (work in progress)
DB += QF2preUtilityLCLS2_v3.db

# Alarms for LLRF Rack only
DB += QF2preAlarmLCLS2PRC.db
DB += QF2preAlarmLCLS2RFS.db
# Alarms for full LLRF system with external fiber connections
DB += QF2preAlarmLCLS2PRCFull.db
DB += QF2preAlarmLCLS2RFSFull.db
DB += QF2preAlarmLCLS2RESFull.db

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

