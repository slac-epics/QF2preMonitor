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
DB += QF2preAlarmLCLS2PRC.db
DB += QF2preAlarmLCLS2RFS.db
DB += perQF2preAlarmLCLS2.template
DB += QF2preUtilityLCLS2.db

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

