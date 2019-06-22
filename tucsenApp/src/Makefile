TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# Need to figure this out; library is only shareable, so we say LIB_SYS_LIBS and it works
ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
SHARED_LIBRARIES=NO
endif

LIBRARY_IOC_Linux += tucsen
LIB_SRCS += tucsen.cpp
LIB_SYS_LIBS_Linux += TUCam

DBD += tucsenSupport.dbd

# Add locally compiled object code
PROD_LIBS += tucsen
PROD_SYS_LIBS_Linux += TUCam

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

