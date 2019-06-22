< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/tucsenApp.dbd")
tucsenApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "TUCSEN1:")
# The port name for the detector
epicsEnvSet("PORT",   "TUCSEN")
# In the case of a single camera set to 0 
# In the case of multiple camera set to integer 0 to (num cameras in system) but note that the enumeration will change with system reboot and connect/disconnect
# In the case of multiple camera set this to the camera serial number
epicsEnvSet("CAMERA", "KBS40B1810003")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "21")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "2048")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "2044")
# The maximum number of time seried points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

# tucsenConfig(const char *portName, int cameraId, int traceMask, int maxBuffers,
#              size_t maxMemory, int priority, int stackSize)
tucsenConfig("$(PORT)", $(CAMERA), 0x1, 0)
dbLoadRecords("$(ADTUCSEN)/db/tucsen.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
# Make NELEMENTS in the following be a little bigger than 2048*2048
# Use the following command for 32-bit images.  This is needed for 32-bit detectors or for 16-bit detectors in acccumulate mode if it would overflow 16 bits
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=5600000")
# Use the following command for 16-bit images.  This can be used for 16-bit detector as long as accumulate mode would not result in 16-bit overflow
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=5600000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADTUCSEN)/tucsenApp/Db")

asynSetTraceMask("$(PORT)",-1,9)
asynSetTraceIOMask("$(PORT)",-1,4)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")
