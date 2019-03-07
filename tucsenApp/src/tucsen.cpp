/* This is a driver for the TUcsen Dhyana 900D camera. It should work for all
 * Tucsen USB3 cameras that use the TUCAM api.
 *
 * Author: David Vine
 * Date  : 28 October 2017
 *
 * Based on Mark Rivers PointGrey driver.
 *
 * 19 Jan 2019 - Update to sdk version 2.0. Modified array size to be 2048x2044
 * for Dhyana 400D and 2048x2040 for 400BSI
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>
#include <epicsExport.h>

#include <TUCamApi.h>

#include <ADDriver.h>

#define DRIVER_VERSION      0
#define DRIVER_REVISION     3
#define DRIVER_MODIFICATION 0

#define TucsenBusString           "T_BUS"     
#define TucsenProductIDString     "T_PRODUCT_ID"
#define TucsenDriverVersionString "T_DRIVER_VERSION"
#define TucsenTransferRateString  "T_TRANSER_RATE"
#define TucsenFrameFormatString   "T_FRAME_FORMAT"
#define TucsenBinningModeString   "T_BIN_MODE"
#define TucsenFanGearString       "T_FAN_GEAR"

static const int frameFormats[3] = {
    0x10,
    0x11,
    0x12
};

static const char* driverName = "tucsen";

static int TUCAMInitialized = 0;

/* Main driver class inherited from areaDetector ADDriver class */

class tucsen : public ADDriver
{
    public:
        tucsen( const char* portName, int cameraId, int traceMask, int maxBuffers,
                size_t maxMemory, int priority, int stackSize);

        /* Virtual methods to override from ADDrive */
        virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
        virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);

        /* These should be private but must be called from C */
        void imageGrabTask();
        void shutdown();
        void tempTask();

    protected:
        int TucsenBus;
#define FIRST_TUCSEN_PARAM TucsenBus
        int TucsenProductID;
        int TucsenDriverVersion;
        int TucsenTransferRate;
        int TucsenBinMode;
        int TucsenFanGear;
        int TucsenFrameFormat;
#define LAST_TUCSEN_PARAM TucsenFrameFormat

    private:
        /* Local methods to this class */
        asynStatus grabImage();
        asynStatus startCapture();
        asynStatus stopCapture();

        asynStatus connectCamera();
        asynStatus disconnectCamera();

        /* camera property control functions */
        asynStatus getCamInfo(int nID, char* sBuf, int &val);
        asynStatus setCamInfo(int param, int nID, int dtype);
        asynStatus setSerialNumber();
        asynStatus setProperty(int nID, double value);
        asynStatus setCapability(int property, int value);
        asynStatus getCapability(int property, int& value);

        /* Data */
        int cameraId_;
        int exiting_;
        TUCAM_INIT apiHandle_;
        TUCAM_OPEN camHandle_;
        TUCAM_FRAME frameHandle_;
        TUCAM_TRIGGER_ATTR triggerHandle_;
        epicsEventId startEventId_;
        NDArray *pRaw_;
};

#define NUM_TUCSEN_PARAMS ((int)(&LAST_TUCSEN_PARAM-&FIRST_TUCSEN_PARAM+1))


/* Configuration function to configure one camera
 *
 * This function needs to be called once for each camera used by the IOC. A
 * call to this function instantiates one object of the Tucsen class.
 * \param[in] portName asyn port to assign to the camera
 * \param[in] cameraId The camera index or serial number
 * \param[in] traceMask the initial value of asynTraceMask
 *            if set to 0 or 1 then asynTraceMask will be set to
 *            ASYN_TRACE_ERROR.
 *            if set to 0x21 ( ASYN_TRACE_WARNING | ASYN_TRACE_ERROR) then each
 *            call will be traced during initialization
 * \param[in] maxBuffers Maximum number of NDArray objects (image buffers) this
 *            driver is allowed to allocate.
 *            0 = unlimited
 * \param[in] maxMemory Maximum memort (in bytes) that this driver is allowed
 *            to allocate.
 *            0=unlimited
 * \param[in] priority The epics thread priority for this driver. 0= asyn
 *            default.
 * \param[in] stackSize The size of the stack of the EPICS port thread. 0=use
 *            asyn default.
 */
extern "C" int tucsenConfig(const char *portName, int cameraId, int traceMask,
        int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new tucsen( portName, cameraId, traceMask, maxBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
}

static void c_shutdown(void *arg)
{
    tucsen *t = (tucsen *)arg;
    t->shutdown();
}

static void imageGrabTaskC(void *drvPvt)
{
    tucsen *t = (tucsen *)drvPvt;
    t->imageGrabTask();
}

static void tempReadTaskC(void *drvPvt)
{
    tucsen *t = (tucsen *)drvPvt;
    t->tempTask();
}

/* Constructor for the Tucsen class */

tucsen::tucsen(const char *portName, int cameraId, int traceMask, int maxBuffers,
        size_t maxMemory, int priority, int stackSize)
    : ADDriver( portName, 1, NUM_TUCSEN_PARAMS, maxBuffers, maxMemory, asynEnumMask,
            asynEnumMask, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),
    cameraId_(cameraId), exiting_(0), pRaw_(NULL)
{
    static const char *functionName = "tucsen";

    asynStatus status;

    if(traceMask==0) traceMask = ASYN_TRACE_ERROR;
    pasynTrace->setTraceMask(pasynUserSelf, traceMask);

    createParam(TucsenBusString,           asynParamOctet,   &TucsenBus);
    createParam(TucsenProductIDString,     asynParamFloat64, &TucsenProductID);
    createParam(TucsenDriverVersionString, asynParamOctet,   &TucsenDriverVersion);
    createParam(TucsenTransferRateString,  asynParamFloat64, &TucsenTransferRate);
    createParam(TucsenBinningModeString,   asynParamInt32,   &TucsenBinMode);
    createParam(TucsenFanGearString,       asynParamInt32,   &TucsenFanGear);
    createParam(TucsenFrameFormatString,   asynParamInt32,   &TucsenFrameFormat);

    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: camera connection failed (%d)\n",
                driverName, functionName, status);
        report(stdout, 1);
        return;
    }

    /* Set initial values for some parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    setStringParam(ADManufacturer, "Tucsen");
	
	char cameraModel[256];
	getStringParam(ADModel, sizeof(cameraModel), cameraModel);
	printf("%s\n", cameraModel);
	if (strcmp("Dhyana 400BSI", cameraModel)==0){
		printf("%s\n", cameraModel);
		setIntegerParam(ADMaxSizeX, 2048);
		setIntegerParam(ADMaxSizeY, 2040);
		setIntegerParam(NDArraySizeX, 2048);
		setIntegerParam(NDArraySizeY, 2040);
		setIntegerParam(NDArraySize, 2*2048*2040);
	} else { /* Assume Dhyana 400D */
		setIntegerParam(ADMaxSizeX, 2048);
		setIntegerParam(ADMaxSizeY, 2044);
		setIntegerParam(NDArraySizeX, 2048);
		setIntegerParam(NDArraySizeY, 2048);
		setIntegerParam(NDArraySize, 2*2048*2044);
	}

    startEventId_ = epicsEventCreate(epicsEventEmpty);

    /* Launch image read task */
    epicsThreadCreate("TucsenImageReadTask",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            imageGrabTaskC, this);

    /* Launch temp task */
    epicsThreadCreate("TucsenTempReadTask",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            tempReadTaskC, this);

    /* Launch shutdown task */
    epicsAtExit(c_shutdown, this);

    return;
}

void tucsen::shutdown(void)
{
    exiting_=1;
    if (camHandle_.hIdxTUCam != NULL){
        disconnectCamera();
    }
    TUCAMInitialized--;
    if(TUCAMInitialized==0){
        TUCAM_Api_Uninit();
    }
}

asynStatus tucsen::connectCamera()
{
    static const char* functionName = "connectCamera";
    int tucStatus;
    asynStatus status;

    // Init API
    char szPath[1024] = {0};
    getcwd(szPath, 1024);
    apiHandle_.pstrConfigPath = szPath;
    apiHandle_.uiCamCount = 0;

    tucStatus = TUCAM_Api_Init(&apiHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: TUCAM API init failed (%d)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    if (apiHandle_.uiCamCount<1){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: no camera detected (%d)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    
    TUCAMInitialized++;
    
    // Init camera
    camHandle_.hIdxTUCam = 0;
    camHandle_.uiIdxOpen = 0;

    tucStatus = TUCAM_Dev_Open(&camHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: open camera device failed (%d)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    status = setCamInfo(TucsenBus, TUIDI_BUS, 0);
    status = setCamInfo(TucsenProductID, TUIDI_PRODUCT, 1);
    status = setCamInfo(ADSDKVersion, TUIDI_VERSION_API, 0);
    status = setCamInfo(ADFirmwareVersion, TUIDI_VERSION_FRMW, 0);
    status = setCamInfo(ADModel, TUIDI_CAMERA_MODEL, 0);
    status = setCamInfo(TucsenDriverVersion, TUIDI_VERSION_DRIVER, 0);
    status = setSerialNumber();

    return status;
}

asynStatus tucsen::disconnectCamera(void){
    static const char* functionName = "disconnectCamera";
    int tucStatus;
    int acquiring;
    asynStatus status;

    // check if acquiring
    status = getIntegerParam(ADAcquire, &acquiring);

    // if necessary stop acquiring
    if (status==asynSuccess && acquiring){
        tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to stop acquisition (%d)\n",
                    driverName, functionName, tucStatus);
        }
    }

    // release buffer
    tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to release camera buffer (%d)\n",
                driverName, functionName, tucStatus);
    }

    tucStatus = TUCAM_Dev_Close(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable close camera (%d)\n",
                driverName, functionName, tucStatus);
    }
    return asynSuccess;
}

void tucsen::imageGrabTask(void)
{
    static const char* functionName = "imageGrabTask";
    asynStatus status  = asynSuccess;
    int tucStatus;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;

    lock();

    while(1){
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are nit acquiring wait for a semaphore that is given when
         * acquisition is started */
        if(!acquire){
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for acquisition to start\n",
                    driverName, functionName);
            unlock();
            epicsEventWait(startEventId_);
            // SEQUENCE or STANDARD?
            tucStatus = TUCAM_Cap_Start(camHandle_.hIdxTUCam, TUCCM_SEQUENCE);
            if (tucStatus!=TUCAMRET_SUCCESS){
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: Failed to start image capture (%d)\n",
                        driverName, functionName, tucStatus);
            }
            lock();
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: acquisition started\n",
                    driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
        }

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);

        /* We are now waiting for an image */
        setIntegerParam(ADStatus, ADStatusWaiting);
        callParamCallbacks();

        status = grabImage();
        if (status==asynError){
            // Release the allocated NDArray
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;
            continue;
        }

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        if(arrayCallbacks){
            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
        }

        if (pRaw_) pRaw_->release();
        pRaw_ = NULL;

        if ((imageMode==ADImageSingle) || ((imageMode==ADImageMultiple) && (numImagesCounter>numImages))){
            status = stopCapture();
        }
        callParamCallbacks();
    }
}

asynStatus tucsen::grabImage()
{
    static const char* functionName = "grabImage";
    asynStatus status = asynSuccess;
    int tucStatus;
    int header, offset;
    int nCols, nRows, stride;
    int bitDepth, pixelFormat, channels, pixelBytes;
    int index, dataSize;
    int tDataSize;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    int numColors;
    int pixelSize;
    size_t dims[3];
    int nDims;
    int count;


    unlock();
    tucStatus = TUCAM_Buf_WaitForFrame(camHandle_.hIdxTUCam, &frameHandle_);
    if (tucStatus!= TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Failed to wait for buffer (%d)\n",
                    driverName, functionName, tucStatus);
    }

    lock();
    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();
    header = frameHandle_.usHeader;
    offset = frameHandle_.usOffset;

    // Width & height are array dimensions
    nCols = frameHandle_.usWidth;
    nRows = frameHandle_.usHeight;

    // Not sure what this is "Frame image width step"?
    stride =frameHandle_.uiWidthStep;

    // Pixel bit depth
    bitDepth = frameHandle_.ucDepth;
    // Image format
    pixelFormat = frameHandle_.ucFormat;
    // Number of channels
    channels = frameHandle_.ucChannels;
    // Not sure what this is "Frame image data byte"?
    pixelBytes = frameHandle_.ucElemBytes;
    // Frame image serial number?
    index = frameHandle_.uiIndex;
    // Frame image data size
    tDataSize = frameHandle_.uiImgSize;

    /* There is zero documentation on what the formats mean
     * Most of the below is gleaned through trial and error */
    if (pixelFormat==16){
            // Raw data - no filtering applied
            dataType = NDUInt16;
            colorMode = NDColorModeMono;
            numColors = 1;
            pixelSize = 2;
    } else if (pixelFormat==17){
            if (bitDepth==1){
                dataType=NDUInt8;
                pixelSize = 1;
            } else if (bitDepth==2) {
                dataType=NDUInt16;
                pixelSize = 2;
            }
            
            if (channels==1){
                colorMode = NDColorModeMono;
                numColors = 1;
            } else if (channels==3){
                colorMode = NDColorModeRGB1;
                numColors = 3;
            }
    } else if (pixelFormat==18){
            dataType=NDUInt8;
            pixelSize = 1;
            colorMode = NDColorModeRGB1;
            numColors = 3;
    } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Unsupported pixel format %d\n",
                    driverName, functionName, pixelFormat);
            return asynError;
    }
    if (numColors==1){
        nDims = 2;
        dims[0] = nCols;
        dims[1] = nRows;
    } else {
        nDims = 3;
        dims[0] = 3;
        dims[1] = nCols;
        dims[2] = nRows;
    }

    dataSize = dims[0]*dims[1]*pixelSize;
    if (nDims==3) dataSize *= dims[2];

    if (dataSize != tDataSize){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: data size mismatch: calculated=%d, reported=%d\n",
                driverName, functionName, dataSize, tDataSize);
        return asynError;
    }

    setIntegerParam(NDArraySizeX, nCols);
    setIntegerParam(NDArraySizeY, nRows);
    setIntegerParam(NDArraySize, (int)dataSize);
    setIntegerParam(NDDataType, dataType);
    setIntegerParam(NDColorMode, colorMode);

    pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
    if(!pRaw_){
        // No valid buffer so we need to abort
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s [%s] ERROR: Serious problem: not enough buffers left. Aborting acquisition!\n",
                driverName, functionName, portName);
        setIntegerParam(ADAcquire, 0);
    }
    memcpy(pRaw_->pData, frameHandle_.pBuffer+offset, dataSize);
    getIntegerParam(NDArrayCounter, &count);
    pRaw_->uniqueId = count;
    updateTimeStamp(&pRaw_->epicsTS);
    pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch+pRaw_->epicsTS.nsec/1e9;

    getAttributes(pRaw_->pAttributeList);

    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();
    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

    return status;
}

/* Sets an int32 parameter */
asynStatus tucsen::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
    static const char* functionName = "writeInt32";
    const char* paramName;
    asynStatus status = asynSuccess;
    int tucStatus;
    int function = pasynUser->reason;

    getParamName(function, &paramName);
    status = setIntegerParam(function, value);
    
    if (function==ADAcquire){
        if (value){
            status = startCapture();
        } else {
            status = stopCapture();
        }
    } else if ((function==ADTriggerMode) ||
               (function==ADNumImages)   ||
               (function==ADNumExposures)){
        //status = setTrigger();
    } else if (function==TucsenFrameFormat){
        frameHandle_.ucFormatGet = frameFormats[value];
        frameHandle_.uiRsdSize = 1;
        tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
        frameHandle_.pBuffer = NULL;
        tucStatus = TUCAM_Buf_Alloc(camHandle_.hIdxTUCam, &frameHandle_);
    } else if (function==TucsenBinMode){
        if ((value==0)||(value==1)){
            status = setCapability(TUIDC_RESOLUTION, value);
        }
        tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
        frameHandle_.pBuffer = NULL;
        tucStatus = TUCAM_Buf_Alloc(camHandle_.hIdxTUCam, &frameHandle_);
    } else if (function==TucsenFanGear){
        if ((value>=0)||(value<=6)){
            status = setCapability(TUIDC_FAN_GEAR, value);
        }
    } else {
        if (function < FIRST_TUCSEN_PARAM){
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
            "%s::%s function=%d, value=%d, status=%d\n",
            driverName, functionName, function, value, status);
    callParamCallbacks();
    return status;
}

void tucsen::tempTask(void){
    static const char* functionName = "tempTask";
    TUCAM_VALUE_INFO valInfo;
    int tucStatus;
    double dbVal;

	lock();
    while (!exiting_){
        tucStatus = TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, 
                TUIDP_TEMPERATURE, &dbVal);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: failed to read temperature (%d)\n",
                    driverName, functionName, tucStatus);
        } else {
            setDoubleParam(ADTemperatureActual, dbVal);
        }

        valInfo.nID = TUIDI_TRANSFER_RATE;
        tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: failed to read transfer rate(%d)\n",
                    driverName, functionName, tucStatus);
        } else {
            setDoubleParam(TucsenTransferRate, valInfo.nValue);
        }
        callParamCallbacks();
		unlock();
        epicsThreadSleep(0.5);
		lock();
    }
}

asynStatus tucsen::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    static const char* functionName = "writeFloat64";
    const char* paramName;

    asynStatus status = asynSuccess;
    int function = pasynUser->reason;

    status = setDoubleParam(function, value);

    if (function==ADAcquireTime){
        value*=500.0;
        setProperty(TUIDP_EXPOSURETM, value);
    } else {
        if (function < FIRST_TUCSEN_PARAM){
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }
    callParamCallbacks();
    return (asynStatus)status;
}

asynStatus tucsen::getCamInfo(int nID, char *sBuf, int &val)
{
    static const char* functionName = "getPropertyText";

    // Get camera information
    int tucStatus;
    TUCAM_VALUE_INFO valInfo;
    valInfo.pText = sBuf;
    valInfo.nTextSize = 1024;

    valInfo.nID = nID;
    tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
    if (tucStatus==TUCAMRET_SUCCESS){
        val = valInfo.nValue;
        return asynSuccess;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: could not get %d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
}

asynStatus tucsen::setCamInfo(int param, int nID, int dtype)
{
    static const char* functionName = "setCamInfo";

    int tucStatus;
    TUCAM_VALUE_INFO valInfo;
    int sSize = 1024;
    char sInfo[sSize] = {0};
    valInfo.pText = sInfo;
    valInfo.nTextSize = sSize;
    valInfo.nID = nID;

    tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
    if (tucStatus==TUCAMRET_SUCCESS){
        if (param==TucsenBus){
            if (valInfo.nValue==768){
                setStringParam(TucsenBus, "USB3.0");
            } else{
                setStringParam(TucsenBus, "USB2.0");
            }
        } else if (dtype==0){
            setStringParam(param, valInfo.pText);
        } else if (dtype==1){
            setDoubleParam(param, valInfo.nValue);
        } else if (dtype==2){
            setIntegerParam(param, valInfo.nValue);
        }
        callParamCallbacks();
        return asynSuccess;
    } else {
        return asynError;
    }
}

asynStatus tucsen::setSerialNumber()
{
    static const char* functionName = "setSerialNumber";
    int tucStatus;
    char cSN[TUSN_SIZE] = {0};
    TUCAM_REG_RW regRW;

    regRW.nRegType = TUREG_SN;
    regRW.pBuf = &cSN[0];
    regRW.nBufSize = TUSN_SIZE;

    tucStatus = TUCAM_Reg_Read(camHandle_.hIdxTUCam, regRW);
    if (tucStatus==TUCAMRET_SUCCESS){
        setStringParam(ADSerialNumber, cSN);
        return asynSuccess;
    } else {
        return asynError;
    }
}

asynStatus tucsen::setProperty(int property, double value){

    static const char* functionName = "setProperty";
    TUCAM_PROP_ATTR attrProp;
    int tucStatus;

    attrProp.nIdxChn = 0;
    attrProp.idProp = property;
    tucStatus = TUCAM_Prop_GetAttr(camHandle_.hIdxTUCam, &attrProp);
    if (tucStatus==TUCAMRET_SUCCESS)
    {
        if(value<attrProp.dbValMin){
            value = attrProp.dbValMin;
        } else if (value>attrProp.dbValMax){
            value = attrProp.dbValMax;
        }
    }
    tucStatus = TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, property, value);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to set property %d\n",
                driverName, functionName, property);
        return asynError;
    } else {
        return asynSuccess;
    }
}

asynStatus tucsen::setCapability(int property, int val)
{
    static const char* functionName = "setCapability";
    int tucStatus;

    tucStatus = TUCAM_Capa_SetValue(camHandle_.hIdxTUCam, property, val);
    if (tucStatus!=TUCAMRET_SUCCESS)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to set capability %d=%d\n",
                driverName, functionName, property, val);
        return asynError;
    }
    return asynSuccess;
}

asynStatus tucsen::getCapability(int property, int& val)
{
    static const char* functionName = "getCapability";
    int tucStatus;

    tucStatus = TUCAM_Capa_GetValue(camHandle_.hIdxTUCam, property, &val);
    if (tucStatus!=TUCAMRET_SUCCESS)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get capability %d=%d\n",
                driverName, functionName, property, val);
        return asynError;
    }
    return asynSuccess;
}


asynStatus tucsen::startCapture()
{
    static const char* functionName = "startCapture";

    setIntegerParam(ADNumImagesCounter, 0);
    setShutter(1);
    epicsEventSignal(startEventId_);
    return asynSuccess;
}

asynStatus tucsen::stopCapture()
{
    static const char* functionName = "stopCapture";
    int tucStatus;

    setShutter(0);
    tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, ADStatusIdle);
    callParamCallbacks();
    return asynSuccess;
}

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"CameraId", iocshArgInt};
static const iocshArg configArg2 = {"traceMask", iocshArgInt};
static const iocshArg configArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg configArg4 = {"maxMemory", iocshArgInt};
static const iocshArg configArg5 = {"priority", iocshArgInt};
static const iocshArg configArg6 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs [] = {&configArg0,
                                               &configArg1,
                                               &configArg2,
                                               &configArg3,
                                               &configArg4,
                                               &configArg5,
                                               &configArg6};
static const iocshFuncDef configtucsen = {"tucsenConfig", 7, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    tucsenConfig(args[0].sval, args[1].ival, args[2].ival,
                 args[3].ival, args[4].ival, args[5].ival,
                 args[6].ival);
}

static void tucsenRegister(void)
{
    iocshRegister(&configtucsen, configCallFunc);
}

extern "C" {
    epicsExportRegistrar(tucsenRegister);
}

