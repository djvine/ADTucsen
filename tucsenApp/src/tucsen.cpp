/* This is a driver for the TUcsen Dhyana 900D camera. It should work for all
 * Tucsen USB3 cameras that use the TUCAM api.
 *
 * Author: David Vine
 * Date  : 28 October 2017
 *
 * Based on Mark Rivers PointGrey driver.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <direct.h>
#define getcwd _getcwd
#else
#include <unistd.h>
#endif

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <TUCamApi.h>

#include <ADDriver.h>

#include <epicsExport.h>

#define DRIVER_VERSION      0
#define DRIVER_REVISION     2
#define DRIVER_MODIFICATION 0

#define TucsenBusString           "T_BUS"
#define TucsenProductIDString     "T_PRODUCT_ID"
#define TucsenTransferRateString  "T_TRANSFER_RATE"
#define TucsenFrameSpeedString    "T_FRAME_SPEED"
#define TucsenBitDepthString      "T_BIT_DEPTH"
#define TucsenFrameFormatString   "T_FRAME_FORMAT"
#define TucsenBinningModeString   "T_BIN_MODE"
#define TucsenImageModeString     "T_IMG_MODE"
#define TucsenAutoExposureString  "T_AUTO_EXPOSURE"
#define TucsenFanGearString       "T_FAN_GEAR"
#define TucsenAutoLevelsString    "T_AUTO_LEVELS"
#define TucsenHistogramString     "T_HISTOGRAM"
#define TucsenEnhanceString       "T_ENHANCE"
#define TucsenDefectCorrString    "T_DEFECT_CORR"
#define TucsenDenoiseString       "T_ENABLE_DENOISE"
#define TucsenFlatCorrString      "T_FLAT_CORR"
#define TucsenDynRgeCorrString    "T_DYN_RGE_CORR"
#define TucsenBrightnessString    "T_BRIGHTNESS"
#define TucsenBlackLevelString    "T_BLACK_LEVEL"
#define TucsenSharpnessString     "T_SHARPNESS"
#define TucsenNoiseLevelString    "T_NOISE_LEVEL"
#define TucsenHDRKString          "T_HDRK"
#define TucsenGammaString         "T_GAMMA"
#define TucsenContrastString      "T_CONTRAST"
#define TucsenLeftLevelString     "T_LEFT_LEVELS"
#define TucsenRightLevelString    "T_RIGHT_LEVELS"
#define TucsenTriggerEdgeString   "T_TRIG_EDGE"
#define TucsenTriggerExposureString "T_TRIG_EXP"
#define TucsenTriggerDelayString  "T_TRIG_DLY"
#define TucsenTriggerSoftwareString "T_TRIG_SOFT"
#define TucsenTriggerOut1ModeString   "T_TRGOUT1_MODE"
#define TucsenTriggerOut1EdgeString   "T_TRGOUT1_EDGE"
#define TucsenTriggerOut1DelayString  "T_TRGOUT1_DLY"
#define TucsenTriggerOut1WidthString  "T_TRGOUT1_WIDTH"
#define TucsenTriggerOut2ModeString   "T_TRGOUT2_MODE"
#define TucsenTriggerOut2EdgeString   "T_TRGOUT2_EDGE"
#define TucsenTriggerOut2DelayString  "T_TRGOUT2_DLY"
#define TucsenTriggerOut2WidthString  "T_TRGOUT2_WIDTH"
#define TucsenTriggerOut3ModeString   "T_TRGOUT3_MODE"
#define TucsenTriggerOut3EdgeString   "T_TRGOUT3_EDGE"
#define TucsenTriggerOut3DelayString  "T_TRGOUT3_DLY"
#define TucsenTriggerOut3WidthString  "T_TRGOUT3_WIDTH"

static const int frameFormats[3] = {
    TUFRM_FMT_RAW,
    TUFRM_FMT_USUAl,
    TUFRM_FMT_RGB888
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
        virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn);
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
        int TucsenTransferRate;
        int TucsenFrameSpeed;
        int TucsenBitDepth;
        int TucsenBinMode;
        int TucsenFanGear;
        int TucsenImageMode;
        int TucsenAutoExposure;
        int TucsenFrameFormat;
        int TucsenAutoLevels;
        int TucsenHistogram;
        int TucsenEnhance;
        int TucsenDefectCorr;
        int TucsenDenoise;
        int TucsenFlatCorr;
        int TucsenDynRgeCorr;
        int TucsenBrightness;
        int TucsenBlackLevel;
        int TucsenSharpness;
        int TucsenNoiseLevel;
        int TucsenHDRK;
        int TucsenGamma;
        int TucsenContrast;
        int TucsenLeftLevel;
        int TucsenRightLevel;
        int TucsenTriggerEdge;
        int TucsenTriggerExposure;
        int TucsenTriggerDelay;
        int TucsenTriggerSoftware;
        int TucsenTriggerOut1Mode;
        int TucsenTriggerOut1Edge;
        int TucsenTriggerOut1Delay;
        int TucsenTriggerOut1Width;
        int TucsenTriggerOut2Mode;
        int TucsenTriggerOut2Edge;
        int TucsenTriggerOut2Delay;
        int TucsenTriggerOut2Width;
        int TucsenTriggerOut3Mode;
        int TucsenTriggerOut3Edge;
        int TucsenTriggerOut3Delay;
        int TucsenTriggerOut3Width;
#define LAST_TUCSEN_PARAM TucsenTriggerOut3Width

    private:
        /* Local methods to this class */
        asynStatus grabImage();
        asynStatus startCapture();
        asynStatus stopCapture();

        asynStatus connectCamera();
        asynStatus disconnectCamera();

        asynStatus getTrigger();
        asynStatus setTrigger();
        asynStatus getTriggerOut(int port);
        asynStatus setTriggerOut(int port);
        asynStatus getROI();
        asynStatus setROI();

        /* camera property control functions */
        asynStatus getCamInfo(int nID, char* sBuf, int &val);
        asynStatus setCamInfo(int param, int nID, int dtype);
        asynStatus setSerialNumber();
        asynStatus getProperty(int nID, double& value);
        asynStatus setProperty(int nID, double value);
        asynStatus setCapability(int property, int value);
        asynStatus getCapability(int property, int& value);
        asynStatus getCapabilityText(int property, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn);

        /* Data */
        int cameraId_;
        int exiting_;
        TUCAM_INIT apiHandle_;
        TUCAM_OPEN camHandle_;
        TUCAM_FRAME frameHandle_;
        TUCAM_TRIGGER_ATTR triggerHandle_;
        TUCAM_TRGOUT_ATTR triggerOutHandle_[3];
        epicsEventId startEventId_;
        NDArray *pRaw_;
        int triggerOutSupport_;
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
    cameraId_(cameraId), exiting_(0), pRaw_(NULL), triggerOutSupport_(0)
{
    static const char *functionName = "tucsen";

    char versionString[20];
    asynStatus status;

    if(traceMask==0) traceMask = ASYN_TRACE_ERROR;
    pasynTrace->setTraceMask(pasynUserSelf, traceMask);

    createParam(TucsenBusString,           asynParamOctet,   &TucsenBus);
    createParam(TucsenProductIDString,     asynParamFloat64, &TucsenProductID);
    createParam(TucsenTransferRateString,  asynParamFloat64, &TucsenTransferRate);
    createParam(TucsenFrameSpeedString,    asynParamInt32,   &TucsenFrameSpeed);
    createParam(TucsenBitDepthString,      asynParamInt32,   &TucsenBitDepth);
    createParam(TucsenBinningModeString,   asynParamInt32,   &TucsenBinMode);
    createParam(TucsenFanGearString,       asynParamInt32,   &TucsenFanGear);
    createParam(TucsenImageModeString,     asynParamInt32,   &TucsenImageMode);
    createParam(TucsenAutoExposureString,  asynParamInt32,   &TucsenAutoExposure);
    createParam(TucsenAutoLevelsString,    asynParamInt32,   &TucsenAutoLevels);
    createParam(TucsenHistogramString,     asynParamInt32,   &TucsenHistogram);
    createParam(TucsenEnhanceString,       asynParamInt32,   &TucsenEnhance);
    createParam(TucsenDefectCorrString,    asynParamInt32,   &TucsenDefectCorr);
    createParam(TucsenDenoiseString,       asynParamInt32,   &TucsenDenoise);
    createParam(TucsenFlatCorrString,      asynParamInt32,   &TucsenFlatCorr);
    createParam(TucsenDynRgeCorrString,    asynParamInt32,   &TucsenDynRgeCorr);
    createParam(TucsenFrameFormatString,   asynParamInt32,   &TucsenFrameFormat);
    createParam(TucsenBrightnessString,    asynParamFloat64, &TucsenBrightness);
    createParam(TucsenBlackLevelString,    asynParamFloat64, &TucsenBlackLevel);
    createParam(TucsenSharpnessString,     asynParamFloat64, &TucsenSharpness);
    createParam(TucsenNoiseLevelString,    asynParamFloat64, &TucsenNoiseLevel);
    createParam(TucsenHDRKString,          asynParamFloat64, &TucsenHDRK);
    createParam(TucsenGammaString,         asynParamFloat64, &TucsenGamma);
    createParam(TucsenContrastString,      asynParamFloat64, &TucsenContrast);
    createParam(TucsenLeftLevelString,     asynParamFloat64, &TucsenLeftLevel);
    createParam(TucsenRightLevelString,    asynParamFloat64, &TucsenRightLevel);
    createParam(TucsenTriggerEdgeString,   asynParamInt32,   &TucsenTriggerEdge);
    createParam(TucsenTriggerExposureString, asynParamInt32, &TucsenTriggerExposure);
    createParam(TucsenTriggerDelayString,   asynParamFloat64, &TucsenTriggerDelay);
    createParam(TucsenTriggerSoftwareString, asynParamInt32, &TucsenTriggerSoftware);
    createParam(TucsenTriggerOut1ModeString,  asynParamInt32,   &TucsenTriggerOut1Mode);
    createParam(TucsenTriggerOut1EdgeString,  asynParamInt32,   &TucsenTriggerOut1Edge);
    createParam(TucsenTriggerOut1DelayString, asynParamFloat64, &TucsenTriggerOut1Delay);
    createParam(TucsenTriggerOut1WidthString, asynParamFloat64, &TucsenTriggerOut1Width);
    createParam(TucsenTriggerOut2ModeString,  asynParamInt32,   &TucsenTriggerOut2Mode);
    createParam(TucsenTriggerOut2EdgeString,  asynParamInt32,   &TucsenTriggerOut2Edge);
    createParam(TucsenTriggerOut2DelayString, asynParamFloat64, &TucsenTriggerOut2Delay);
    createParam(TucsenTriggerOut2WidthString, asynParamFloat64, &TucsenTriggerOut2Width);
    createParam(TucsenTriggerOut3ModeString,  asynParamInt32,   &TucsenTriggerOut3Mode);
    createParam(TucsenTriggerOut3EdgeString,  asynParamInt32,   &TucsenTriggerOut3Edge);
    createParam(TucsenTriggerOut3DelayString, asynParamFloat64, &TucsenTriggerOut3Delay);
    createParam(TucsenTriggerOut3WidthString, asynParamFloat64, &TucsenTriggerOut3Width);

    /* Set initial values for some parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    setStringParam(ADManufacturer, "Tucsen");
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
            DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);

    status = connectCamera();
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: camera connection failed\n",
                driverName, functionName);
        setIntegerParam(ADStatus, ADStatusDisconnected);
        setStringParam(ADStatusMessage, "camera connection failed");
        report(stdout, 1);
        return;
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
    int status = asynSuccess;

    // Init API
    char szPath[1024] = {0};
    getcwd(szPath, 1024);
    apiHandle_.pstrConfigPath = szPath;
    apiHandle_.uiCamCount = 0;

    tucStatus = TUCAM_Api_Init(&apiHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: TUCAM API init failed (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    if (apiHandle_.uiCamCount<1){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: no camera detected (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    TUCAMInitialized++;

    // Init camera
    camHandle_.hIdxTUCam = NULL;
    camHandle_.uiIdxOpen = cameraId_;

    tucStatus = TUCAM_Dev_Open(&camHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: open camera device failed (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    status |= setCamInfo(TucsenBus, TUIDI_BUS, 0);
    status |= setCamInfo(TucsenProductID, TUIDI_PRODUCT, 1);
    status |= setCamInfo(ADSDKVersion, TUIDI_VERSION_API, 0);
    status |= setCamInfo(ADFirmwareVersion, TUIDI_VERSION_FRMW, 0);
    status |= setCamInfo(ADModel, TUIDI_CAMERA_MODEL, 0);
    status |= setSerialNumber();
    status |= setCamInfo(ADMaxSizeX, TUIDI_CURRENT_WIDTH, 2);
    status |= setCamInfo(ADMaxSizeY, TUIDI_CURRENT_HEIGHT, 2);
    status |= getROI();
    status |= getTrigger();

    triggerOutSupport_ = 1;
    for(int port = 0; port < 3; port++) {
        if (getTriggerOut(port) != asynSuccess) {
            triggerOutSupport_ = 0;
            break;
        }
    }

    return (asynStatus)status;
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
        status = stopCapture();
    }

    tucStatus = TUCAM_Dev_Close(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        status = asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable close camera (0x%x)\n",
                driverName, functionName, tucStatus);
    }
    return status;
}

void tucsen::imageGrabTask(void)
{
    static const char* functionName = "imageGrabTask";
    int status  = asynSuccess;
    int tucStatus;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int triggerMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;

    lock();

    while(1){
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring wait for a semaphore that is given when
         * acquisition is started */
        if(!acquire){
            if (!status) {
                setIntegerParam(ADStatus, ADStatusIdle);
                setStringParam(ADStatusMessage, "Waiting for the acquire command");
                callParamCallbacks();
            }
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for acquisition to start\n",
                    driverName, functionName);
            unlock();
            tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
            epicsEventWait(startEventId_);
            lock();
            getIntegerParam(ADTriggerMode, &triggerMode);
            tucStatus = TUCAM_Buf_Alloc(camHandle_.hIdxTUCam, &frameHandle_);
            tucStatus = TUCAM_Cap_Start(camHandle_.hIdxTUCam, triggerMode);
            if (tucStatus!=TUCAMRET_SUCCESS){
                status = asynError;
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: failed to start image capture (0x%x)\n",
                        driverName, functionName, tucStatus);
                setIntegerParam(ADAcquire, 0);
                setIntegerParam(ADStatus, ADStatusError);
                setStringParam(ADStatusMessage, "Failed to start image capture");
                callParamCallbacks();
                continue;
            }

            setIntegerParam(ADStatus, ADStatusAcquire);
            setStringParam(ADStatusMessage, "Acquiring...");
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: acquisition started\n",
                    driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
            callParamCallbacks();
        }

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);

        status = grabImage();
        if (status==asynError){
            // Release the allocated NDArray
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;

            getIntegerParam(ADAcquire, &acquire);
            if (acquire == 0) {
                if (imageMode != ADImageContinuous) {
                    setIntegerParam(ADStatus, ADStatusAborted);
                    setStringParam(ADStatusMessage, "Aborted by user");
                } else {
                    status = asynSuccess;
                }
            } else {
                stopCapture();
                setIntegerParam(ADAcquire, 0);
                setIntegerParam(ADStatus, ADStatusError);
                setStringParam(ADStatusMessage, "Failed to get image");
            }
            callParamCallbacks();
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

        if ((imageMode==ADImageSingle) || ((imageMode==ADImageMultiple) && (numImagesCounter>=numImages))){
            status = stopCapture();
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusIdle);
        }
        callParamCallbacks();
    }
}

asynStatus tucsen::grabImage()
{
    static const char* functionName = "grabImage";
    asynStatus status = asynSuccess;
    int tucStatus;
    int nCols, nRows;
    int pixelFormat, channels, pixelBytes;
    size_t dataSize, tDataSize;
    NDDataType_t dataType = NDUInt16;
    NDColorMode_t colorMode = NDColorModeMono;
    int numColors = 1;
    int pixelSize = 2;
    size_t dims[3] = {0};
    int nDims;
    int count;


    unlock();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: wait for buffer\n",
            driverName, functionName);
    tucStatus = TUCAM_Buf_WaitForFrame(camHandle_.hIdxTUCam, &frameHandle_);
    lock();
    if (tucStatus!= TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failed to wait for buffer (0x%x)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    // Width & height are array dimensions
    nCols = frameHandle_.usWidth;
    nRows = frameHandle_.usHeight;

    // Image format
    pixelFormat = frameHandle_.ucFormat;
    // Number of channels
    channels = frameHandle_.ucChannels;
    // Bytes per pixel
    pixelBytes = frameHandle_.ucElemBytes;
    // Frame image data size
    tDataSize = frameHandle_.uiImgSize;

    /* There is zero documentation on what the formats mean
     * Most of the below is gleaned through trial and error */
    if (pixelFormat==TUFRM_FMT_RAW){
        // Raw data - no filtering applied
        if (pixelBytes == 1){
            dataType = NDUInt8;
            pixelSize = 1;
        } else if (pixelBytes==2) {
            dataType = NDUInt16;
            pixelSize = 2;
        }
        colorMode = NDColorModeMono;
        numColors = 1;
    } else if (pixelFormat==TUFRM_FMT_USUAl){
        if (pixelBytes == 1){
            dataType = NDUInt8;
            pixelSize = 1;
        } else if (pixelBytes == 2) {
            dataType = NDUInt16;
            pixelSize = 2;
        }

        if (channels==1){
            colorMode = NDColorModeMono;
            numColors = 1;
        } else if (channels==3){
            colorMode = NDColorModeRGB1;
            numColors = 3;
        }
    } else if (pixelFormat==TUFRM_FMT_RGB888){
        dataType = NDUInt8;
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
                "%s:%s: data size mismatch: calculated=%zu, reported=%zu\n",
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s [%s] ERROR: Serious problem: not enough buffers left. Aborting acquisition!\n",
                driverName, functionName, portName);
        return asynError;
    }
    memcpy(pRaw_->pData, frameHandle_.pBuffer+frameHandle_.usOffset, dataSize);
    getIntegerParam(NDArrayCounter, &count);
    pRaw_->uniqueId = count;
    updateTimeStamp(&pRaw_->epicsTS);
    pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch+pRaw_->epicsTS.nsec/1e9;

    getAttributes(pRaw_->pAttributeList);

    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

    return status;
}

/* Read enum menu */
asynStatus tucsen::readEnum(asynUser *pasynUser,
        char *strings[], int values[], int severities[], size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    const char *functionName = "readEnum";
    asynStatus status = asynSuccess;

    if (function == TucsenBinMode) {
        status = getCapabilityText(TUIDC_RESOLUTION, strings, values, severities, nElements, nIn);
    } else if (function == TucsenBitDepth) {
        TUCAM_CAPA_ATTR attrCapa;
        attrCapa.idCapa = TUIDC_BITOFDEPTH;
        int tucStatus = TUCAM_Capa_GetAttr(camHandle_.hIdxTUCam, &attrCapa);
        if (tucStatus != TUCAMRET_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get bit of depth capability (0x%x)\n",
                driverName, functionName, tucStatus);
            *nIn = 0;
            return asynError;
        }
        /* The min/max value cannot be interpreted as a range, instead just two choices.
         * e.g. a Dhyana 95 sCMOS returns min=12 max=16 */
        char szRes[64] = {0};
        sprintf(szRes, "%d", attrCapa.nValMin);
        if (strings[0]) free(strings[0]);
        strings[0] = epicsStrDup(szRes);
        values[0] = attrCapa.nValMin;
        severities[0] = 0;
        *nIn = 1;
        if (attrCapa.nValMax > attrCapa.nValMin) {
            sprintf(szRes, "%d", attrCapa.nValMax);
            if (strings[1]) free(strings[1]);
            strings[1] = epicsStrDup(szRes);
            values[1] = attrCapa.nValMax;
            severities[1] = 0;
            *nIn = 2;
        }
    } else if (function == TucsenFanGear) {
        status = getCapabilityText(TUIDC_FAN_GEAR, strings, values, severities, nElements, nIn);
    } else if (function == TucsenImageMode) {
        status = getCapabilityText(TUIDC_IMGMODESELECT, strings, values, severities, nElements, nIn);
    } else if (function == TucsenFrameSpeed) {
        status = getCapabilityText(TUIDC_PIXELCLOCK, strings, values, severities, nElements, nIn);
    } else {
        *nIn = 0;
        status = asynError;
    }
    return status;
}

/* Sets an int32 parameter */
asynStatus tucsen::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
    static const char* functionName = "writeInt32";
    const char* paramName;
    int status = asynSuccess;
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
    } else if (function==ADMinX ||
               function==ADMinY ||
               function==ADSizeX ||
               function==ADSizeY){
        status = setROI();
    } else if (function==ADReverseX){
        status |= setCapability(TUIDC_HORIZONTAL, value);
        status |= getCapability(TUIDC_HORIZONTAL, value);
        if (status) {
            value = 0;
        }
        status |= setIntegerParam(ADReverseX, value);
    } else if (function==ADReverseY){
        status |= setCapability(TUIDC_VERTICAL, value);
        status |= getCapability(TUIDC_VERTICAL, value);
        if (status) {
            value = 0;
        }
        status |= setIntegerParam(ADReverseY, value);
    } else if ((function==ADTriggerMode) ||
               (function==TucsenTriggerExposure)){
        status |= setTrigger();
        status |= getTrigger();
    } else if ((function==TucsenTriggerOut1Mode) ||
               (function==TucsenTriggerOut1Edge)){
        if (triggerOutSupport_) {
            status |= setTriggerOut(0);
            status |= getTriggerOut(0);
        } else {
            setIntegerParam(function, 0);
        }
    } else if ((function==TucsenTriggerOut2Mode) ||
               (function==TucsenTriggerOut2Edge)){
        if (triggerOutSupport_) {
            status |= setTriggerOut(1);
            status |= getTriggerOut(1);
        } else {
            setIntegerParam(function, 0);
        }
    } else if ((function==TucsenTriggerOut3Mode) ||
               (function==TucsenTriggerOut3Edge)){
        if (triggerOutSupport_) {
            status |= setTriggerOut(2);
            status |= getTriggerOut(2);
        } else {
            setIntegerParam(function, 0);
        }
    } else if (function==TucsenFrameFormat){
        frameHandle_.ucFormatGet = frameFormats[value];
    } else if (function==TucsenBinMode){
        status |= setCapability(TUIDC_RESOLUTION, value);
        status |= setROI();
    } else if (function==TucsenBitDepth){
        status |= setCapability(TUIDC_BITOFDEPTH, value);
        status |= getCapability(TUIDC_BITOFDEPTH, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenFanGear){
        status |= setCapability(TUIDC_FAN_GEAR, value);
        status |= getCapability(TUIDC_FAN_GEAR, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenImageMode){
        status |= setCapability(TUIDC_IMGMODESELECT, value);
        status |= getCapability(TUIDC_IMGMODESELECT, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenAutoExposure){
        status |= setCapability(TUIDC_ATEXPOSURE, value);
        status |= getCapability(TUIDC_ATEXPOSURE, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenAutoLevels){
        status |= setCapability(TUIDC_ATLEVELS, value);
        status |= getCapability(TUIDC_ATLEVELS, value);
        status |= setIntegerParam(function, value);
        int hist = (value!=0);
        status |= setCapability(TUIDC_HISTC, hist);
        status |= getCapability(TUIDC_HISTC, hist);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenHistogram){
        status |= setCapability(TUIDC_HISTC, value);
        status |= getCapability(TUIDC_HISTC, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenEnhance){
        status |= setCapability(TUIDC_ENHANCE, value);
        status |= getCapability(TUIDC_ENHANCE, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenDefectCorr){
        status |= setCapability(TUIDC_DFTCORRECTION, value);
        status |= getCapability(TUIDC_DFTCORRECTION, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenDenoise){
        status |= setCapability(TUIDC_ENABLEDENOISE, value);
        status |= getCapability(TUIDC_ENABLEDENOISE, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenFlatCorr){
        status |= setCapability(TUIDC_FLTCORRECTION, value);
        status |= getCapability(TUIDC_FLTCORRECTION, value);
        status |= setIntegerParam(function, value);
    } else if (function==TucsenTriggerSoftware){
        int acquire, triggerMode;
        getIntegerParam(ADAcquire, &acquire);
        getIntegerParam(ADTriggerMode, &triggerMode);
        if (acquire && triggerMode == TUCCM_TRIGGER_SOFTWARE) {
            tucStatus = TUCAM_Cap_DoSoftwareTrigger(camHandle_.hIdxTUCam);
            if (tucStatus != TUCAMRET_SUCCESS)
                status = asynError;
        }
    } else {
        if (function < FIRST_TUCSEN_PARAM){
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s: error, status=%d function=%d, value=%d\n",
                driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%d\n",
                driverName, functionName, function, value);

    return (asynStatus)status;
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
                   "%s:%s: failed to read temperature (0x%x)\n",
                    driverName, functionName, tucStatus);
        } else {
            setDoubleParam(ADTemperatureActual, dbVal);
        }

        valInfo.nID = TUIDI_TRANSFER_RATE;
        tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: failed to read transfer rate(0x%x)\n",
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
    int status = asynSuccess;
    int function = pasynUser->reason;

    getParamName(function, &paramName);
    status = setDoubleParam(function, value);

    if (function==ADAcquireTime){
        double valMilliSec = value*1000.0;
        status |= setProperty(TUIDP_EXPOSURETM, valMilliSec);
        status |= getProperty(TUIDP_EXPOSURETM, valMilliSec);
        status |= setDoubleParam(function, valMilliSec / 1000.0);
    } else if (function==ADTemperature){
        /* Temperature is scaled -50C to 50C->0 to 100 */
        value = value+50.0;
        status |= setProperty(TUIDP_TEMPERATURE, value);
    } else if (function==ADGain){
        status |= setProperty(TUIDP_GLOBALGAIN, value);
        status |= getProperty(TUIDP_GLOBALGAIN, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenTriggerDelay){
        status |= setTrigger();
        status |= getTrigger();
    } else if ((function==TucsenTriggerOut1Delay) ||
               (function==TucsenTriggerOut1Width)){
        if (triggerOutSupport_) {
            status |= setTriggerOut(0);
            status |= getTriggerOut(0);
        } else {
            setDoubleParam(function, 0);
        }
    } else if ((function==TucsenTriggerOut2Delay) ||
               (function==TucsenTriggerOut2Width)){
        if (triggerOutSupport_) {
            status |= setTriggerOut(1);
            status |= getTriggerOut(1);
        } else {
            setDoubleParam(function, 0);
        }
    } else if ((function==TucsenTriggerOut3Delay) ||
               (function==TucsenTriggerOut3Width)){
        if (triggerOutSupport_) {
            status |= setTriggerOut(2);
            status |= getTriggerOut(2);
        } else {
            setDoubleParam(function, 0);
        }
    } else if (function==TucsenBrightness){
        status |= setProperty(TUIDP_BRIGHTNESS, value);
        status |= getProperty(TUIDP_BRIGHTNESS, value);
        status |= setDoubleParam(function, value);
    } else if (function==TucsenBlackLevel){
        status |= setProperty(TUIDP_BLACKLEVEL, value);
        status |= getProperty(TUIDP_BLACKLEVEL, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenSharpness){
        status |= setProperty(TUIDP_SHARPNESS, value);
        status |= getProperty(TUIDP_SHARPNESS, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenNoiseLevel){
        status |= setProperty(TUIDP_NOISELEVEL, value);
        status |= getProperty(TUIDP_NOISELEVEL, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenHDRK){
        status |= setProperty(TUIDP_HDR_KVALUE, value);
        status |= getProperty(TUIDP_HDR_KVALUE, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenGamma){
        status |= setProperty(TUIDP_GAMMA, value);
        status |= getProperty(TUIDP_GAMMA, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenContrast){
        status |= setProperty(TUIDP_CONTRAST, value);
        status |= getProperty(TUIDP_CONTRAST, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenLeftLevel){
        status |= setProperty(TUIDP_LFTLEVELS, value);
        status |= getProperty(TUIDP_LFTLEVELS, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else if (function==TucsenRightLevel){
        status |= setProperty(TUIDP_RGTLEVELS, value);
        status |= getProperty(TUIDP_RGTLEVELS, value);
        if (status) value = 0;
        status |= setDoubleParam(function, value);
    } else {
        if (function < FIRST_TUCSEN_PARAM){
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d function=%d, value=%f\n",
                driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%f\n",
                driverName, functionName, function, value);

    return (asynStatus)status;
}

asynStatus tucsen::getCamInfo(int nID, char *sBuf, int &val)
{
    static const char* functionName = "getCamInfo";

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
                "%s:%s: could not get %d (0x%x)\n",
                driverName, functionName, nID, tucStatus);
        return asynError;
    }
}

asynStatus tucsen::setCamInfo(int param, int nID, int dtype)
{
    static const char* functionName = "setCamInfo";

    int tucStatus;
    TUCAM_VALUE_INFO valInfo;
    const int sSize = 1024;
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
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s param %d id %d (error=0x%x)\n",
                driverName, functionName, param, nID, tucStatus);
    }

    return asynSuccess;
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get serial number (error=0x%x)\n",
                driverName, functionName, tucStatus);
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
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: property value range [%f %f]\n",
                driverName, functionName,
                attrProp.dbValMin, attrProp.dbValMax);
        if(value<attrProp.dbValMin){
            value = attrProp.dbValMin;
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s:%s: Clipping set min value: %d, %f\n",
                    driverName, functionName, property, value);
        } else if (value>attrProp.dbValMax){
            value = attrProp.dbValMax;
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s:%s: Clipping set max value: %d, %f\n",
                    driverName, functionName, property, value);
        }
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: value %f\n",
            driverName, functionName, value);

    tucStatus = TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, property, value);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to set property %d to %f (0x%x)\n",
                driverName, functionName, property, value, tucStatus);
        return asynError;
    }
    return asynSuccess;
}

asynStatus tucsen::getProperty(int property, double& value)
{
    static const char* functionName = "getProperty";
    int tucStatus;

    tucStatus = TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, property, &value);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get property %d (0x%x)\n",
                driverName, functionName, property, tucStatus);
        return asynError;
    }
    return asynSuccess;
}

asynStatus tucsen::setCapability(int property, int val)
{
    static const char* functionName = "setCapability";
    int tucStatus;

    tucStatus = TUCAM_Capa_SetValue(camHandle_.hIdxTUCam, property, val);
    if (tucStatus!=TUCAMRET_SUCCESS)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to set capability %d=%d (0x%x)\n",
                driverName, functionName, property, val, tucStatus);
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


asynStatus tucsen::getCapabilityText(int property, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn)
{

    static const char* functionName = "getCapabilityText";
    int tucStatus;
    int i=0;

    TUCAM_CAPA_ATTR attrCapa;
    attrCapa.idCapa = property;

    tucStatus = TUCAM_Capa_GetAttr(camHandle_.hIdxTUCam, &attrCapa);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get capability %d (0x%x)\n",
                driverName, functionName, property, tucStatus);
        *nIn = 0;
        return asynError;
    }

    for (i=0; i<attrCapa.nValMax-attrCapa.nValMin+1; i++) {
        char szRes[64] = {0};
        TUCAM_VALUE_TEXT valText;
        valText.dbValue = i;
        valText.nID = property;
        valText.nTextSize = 64;
        valText.pText = &szRes[0];

        tucStatus = TUCAM_Capa_GetValueText(camHandle_.hIdxTUCam, &valText);

        if (tucStatus != TUCAMRET_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s unable to get capability text %d:%d (0x%x)\n",
                    driverName, functionName, property, i, tucStatus);
            sprintf(valText.pText, "%d", i + attrCapa.nValMin);
        }

        if (strings[i])
        {
            free(strings[i]);
        }

        strings[i] = epicsStrDup(valText.pText);
        values[i]  = i + attrCapa.nValMin;
        severities[i] = 0;
    }

    *nIn = i;
    return asynSuccess;
}

asynStatus tucsen::startCapture()
{
    //static const char* functionName = "startCapture";

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

    tucStatus = TUCAM_Buf_AbortWait(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to abort wait (%d)\n",
                driverName, functionName, tucStatus);
    }

    tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to stop acquisition (%d)\n",
                driverName, functionName, tucStatus);
    }

    tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to release camera buffer (%d)\n",
                driverName, functionName, tucStatus);
    }

    return asynSuccess;
}

asynStatus tucsen::getTrigger()
{
    static const char* functionName = "getTrigger";
    int tucStatus;

    tucStatus = TUCAM_Cap_GetTrigger(camHandle_.hIdxTUCam, &triggerHandle_);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    setIntegerParam(ADTriggerMode, triggerHandle_.nTgrMode);
    setIntegerParam(TucsenTriggerEdge, triggerHandle_.nEdgeMode);
    setIntegerParam(TucsenTriggerExposure, triggerHandle_.nExpMode);
    setDoubleParam(TucsenTriggerDelay, triggerHandle_.nDelayTm/1.0e6);

    return asynSuccess;
}

asynStatus tucsen::setTrigger()
{
    static const char* functionName = "setTrigger";
    int triggerMode, triggerEdge, triggerExposure;
    double triggerDelay;
    int tucStatus;

    getIntegerParam(ADTriggerMode, &triggerMode);
    getIntegerParam(TucsenTriggerEdge, &triggerEdge);
    getIntegerParam(TucsenTriggerExposure, &triggerExposure);
    getDoubleParam(TucsenTriggerDelay, &triggerDelay);

    frameHandle_.uiRsdSize = 1;

    triggerHandle_.nTgrMode = triggerMode;
    triggerHandle_.nEdgeMode = triggerEdge;
    triggerHandle_.nExpMode = triggerExposure;
    triggerHandle_.nFrames = 1;
    triggerHandle_.nDelayTm = int(triggerDelay*1e6);

    tucStatus = TUCAM_Cap_SetTrigger(camHandle_.hIdxTUCam, triggerHandle_);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    return asynSuccess;
}

asynStatus tucsen::getTriggerOut(int port)
{
    static const char* functionName = "getTriggerOut";
    int tucStatus;

    tucStatus = TUCAM_Cap_GetTriggerOut(camHandle_.hIdxTUCam, &triggerOutHandle_[port]);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    setIntegerParam(TucsenTriggerOut1Mode + port * 4, triggerOutHandle_[port].nTgrOutMode);
    setIntegerParam(TucsenTriggerOut1Edge + port * 4, triggerOutHandle_[port].nEdgeMode);
    setDoubleParam(TucsenTriggerOut1Delay + port * 4, triggerOutHandle_[port].nDelayTm/1.0e6);
    setDoubleParam(TucsenTriggerOut1Width + port * 4, triggerOutHandle_[port].nWidth/1.0e6);

    return asynSuccess;
}

asynStatus tucsen::setTriggerOut(int port)
{
    static const char* functionName = "setTriggerOut";
    int triggerMode, triggerEdge;
    double triggerDelay, triggerWidth;
    int tucStatus;

    getIntegerParam(TucsenTriggerOut1Mode + port * 4, &triggerMode);
    getIntegerParam(TucsenTriggerOut1Edge + port * 4, &triggerEdge);
    getDoubleParam(TucsenTriggerOut1Delay + port * 4, &triggerDelay);
    getDoubleParam(TucsenTriggerOut1Width + port * 4, &triggerWidth);

    triggerOutHandle_[port].nTgrOutPort = port;
    triggerOutHandle_[port].nTgrOutMode = triggerMode;
    triggerOutHandle_[port].nEdgeMode   = triggerEdge;
    triggerOutHandle_[port].nDelayTm    = int(triggerDelay*1e6);
    triggerOutHandle_[port].nWidth      = int(triggerWidth*1e6);

    tucStatus = TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, triggerOutHandle_[port]);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    return asynSuccess;
}

asynStatus tucsen::setROI()
{
    static const char* functionName = "setROI";
    int minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
    int tucStatus;
    int status = asynSuccess;

    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    getIntegerParam(ADMaxSizeX, &maxSizeX);
    getIntegerParam(ADMaxSizeY, &maxSizeY);

    if (minX + sizeX > maxSizeX) {
        sizeX = maxSizeX - minX;
        setIntegerParam(ADSizeX, sizeX);
    }
    if (minY + sizeY > maxSizeY) {
        sizeY = maxSizeY - minY;
        setIntegerParam(ADSizeX, sizeY);
    }

    TUCAM_ROI_ATTR roiAttr;
    roiAttr.bEnable = true;
    roiAttr.nHOffset = minX;
    roiAttr.nVOffset = minY;
    roiAttr.nWidth = sizeX;
    roiAttr.nHeight = sizeY;

    tucStatus = TUCAM_Cap_SetROI(camHandle_.hIdxTUCam, roiAttr);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s SetROI error, status=0x%x\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    tucStatus = TUCAM_Cap_GetROI(camHandle_.hIdxTUCam, &roiAttr);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s GetROI error, status=%d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    status |= setIntegerParam(ADMinX, roiAttr.nHOffset);
    status |= setIntegerParam(ADMinY, roiAttr.nVOffset);
    status |= setIntegerParam(ADSizeX, roiAttr.nWidth);
    status |= setIntegerParam(ADSizeY, roiAttr.nHeight);

    return (asynStatus)status;
}

asynStatus tucsen::getROI()
{
    static const char* functionName = "getROI";
    int minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
    int tucStatus;
    int status = asynSuccess;

    getIntegerParam(ADMaxSizeX, &maxSizeX);
    getIntegerParam(ADMaxSizeY, &maxSizeY);

    TUCAM_ROI_ATTR roiAttr;
    tucStatus = TUCAM_Cap_GetROI(camHandle_.hIdxTUCam, &roiAttr);
    if (tucStatus != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s GetROI error, status=%d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    if (roiAttr.bEnable) {
        minX = roiAttr.nHOffset;
        minY = roiAttr.nVOffset;
        sizeX = roiAttr.nWidth;
        sizeY = roiAttr.nHeight;
    } else {
        minX = 0;
        minY = 0;
        sizeX = maxSizeX;
        sizeY = maxSizeY;
    }

    status |= setIntegerParam(ADMinX, minX);
    status |= setIntegerParam(ADMinY, minY);
    status |= setIntegerParam(ADSizeX, sizeX);
    status |= setIntegerParam(ADSizeY, sizeY);

    return (asynStatus)status;
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

