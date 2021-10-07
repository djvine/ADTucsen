TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC += tucsen
LIB_SRCS += tucsen.cpp
ifeq ($(TUCAM_EXTERNAL),YES)
LIB_SYS_LIBS += TUCam
else
LIB_LIBS += TUCam
endif

DBD += tucsenSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

