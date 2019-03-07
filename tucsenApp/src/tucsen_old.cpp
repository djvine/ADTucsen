/*
 * tucsen.cpp
 *
 * This is a driver for Tucsen Dhyan 95 and Dhyana 400D camera
 *
 * Author: David Vine
 *         Sigray, Inc
 *
 * Co-author: Mark Rivers
 *            Univerity of Chicago
 *
 * Created: July 14, 2017
 *
 */

#include <stdio.h>
#include <string.h>
#include <wchar.h>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <TUCamApi.h>

#include "ADDriver.h"

#include <epicsExport.h>

static const char *driverName = "tucsen";

/**
 * Driver for Tucsen sCMOS cameras using beta version of their SDK; inherits from ADDriver class in ADCore.
 *
 */
class tucsen : public ADDriver {
public:
    tucsen(const char *portName, int cameraId, int maxBuffers, 
           size_t maxMemory, int priority, int stackSize, int maxFrames);

    /* override ADDriver methods */ 
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                                size_t nElements, size_t *nIn);
    virtual void report(FILE *fp, int details);

    /* "private methods" that need to be called from C */
    void shutdown();
    void imageTask();
    void tempTask();

protected:
    int TucsenFrameRate;
    #define FIRST_TUCSEN_PARAM TucsenFrameRate
    int TucsenPixelEncoding;
    int TucsenBinning;
    int TucsenShutterMode;
    int TucsenSoftwareTrigger;
    int TucsenTempControl;
    int TucsenTempStatus;
    int TucsenSerialNumber;
    int TucsenFirmwareVersion;
    int TucsenSoftwareVersion;
    int TucsenControllerID;
    int TucsenReadoutRate;
    int TucsenReadoutTime;
    int TucsenTransferRate;
    #define LAST_TUCSEN_PARAM TucsenTransferRate
private:
    int reportFeature(int paramIndex, FILE *fp, int details);
    int allocateBuffers();
    int freeBuffers();
    int connectCamera();
    int disconnectCamera();
    
    TUCAM_INIT 		apiHandle_;
	TUCAM_OPEN 		camHandle_;
	TUCAM_FRAME 	frameHandle_;
	int 			id_;
    int 			maxFrames_;
    unsigned char **drvBuffers_;    
    long long 		imageSize_;
	int 			exiting_;
    epicsEventId 	startEvent_;
	
};
#define NUM_TUCSEN_PARAMS ((int)(&LAST_TUCSEN_PARAM - &FIRST_TUCSEN_PARAM + 1))

/* Andor3 driver specific parameters */
#define TucsenFrameRateString        "T_FRAME_RATE"        /* asynFloat64  rw */
#define TucsenPixelEncodingString    "T_PIXEL_ENCODING"    /* asynInt32    rw */
#define TucsenBinningString          "T_BINNING"           /* asynInt32    rw */
#define TucsenShutterModeString      "T_SHUTTER_MODE"      /* asynInt32    rw */
#define TucsenSoftwareTriggerString  "T_SOFTWARE_TRIGGER"  /* asynInt32    wo */
#define TucsenTempControlString      "T_TEMP_CONTROL"      /* asynInt32    rw */
#define TucsenTempStatusString       "T_TEMP_STATUS"       /* asynInt32    ro */
#define TucsenSerialNumberString     "T_SERIAL_NUMBER"     /* asynOctet    ro */
#define TucsenFirmwareVersionString  "T_FIRMWARE_VERSION"  /* asynOctet    ro */
#define TucsenSoftwareVersionString  "T_SOFTWARE_VERSION"  /* asynOctet    ro */
#define Andor3ControllerIDString     "T_CONTROLLER_ID"     /* asynOctet    ro */
#define Andor3ReadoutRateString      "T_READOUT_RATE"      /* asynInt32    rw */
#define Andor3ReadoutTimeString      "T_READOUT_TIME"      /* asynFloat64  rw */
#define Andor3TransferRateString     "T_TRANSFER_RATE"     /* asynFloat64  rw */

static void c_shutdown(void *arg)
{
  tucsen *p = (tucsen *)arg;
  p->shutdown();
}

static void c_imagetask(void *arg)
{
  tucsen *p = (tucsen *)arg;
  p->imageTask();
}

static void c_temptask(void *arg)
{
  tucsen *p = (tucsen *)arg;
  p->tempTask();
}

void tucsen::shutdown(void)
{
    exiting_ = 1;
    if(handle_) {
        disconnectCamera();
    }
    AtInitialized--;
    if(AtInitialized == 0) {
        AT_FinaliseLibrary();
    }
}

void andor3::imageTask()
{
    epicsTimeStamp imageStamp;
    int status;
    AT_U8  *image;
    int size;
    char modeString[MAX_FEATURE_NAME_LEN];
    int acquire;
    int total;
    int number;
    int count;
    int callback;

    static const char *functionName = "imageTask";

    lock();

    while(!exiting_) {

        getIntegerParam(ADAcquire, &acquire);
        if(!acquire) {
            AT_Flush(handle_);

            unlock();
            epicsEventWait(startEvent_);
            lock();
            
            AT_Flush(handle_);
            for(int x=0; x<maxFrames_; x++) {
                if(drvBuffers_[x]) {
                    status = AT_QueueBuffer(handle_, drvBuffers_[x],
                                            (int)imageSize_);
                    if(status) {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s:%s: AT_QueueBuffer error: %d\n", 
                            driverName, functionName, status);
                    }
                }
            }
            AT_Command(handle_, L"AcquisitionStart");
        }
        
        unlock();
        status = AT_WaitBuffer(handle_, &image, &size, AT_INFINITE);
        lock();
        if(status != AT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: AT_WaitBuffer, error=%d\n", 
                driverName, functionName, status);
            continue;
        }
        epicsTimeGetCurrent(&imageStamp);

        getIntegerParam(ADNumImagesCounter, &number);
        number++;
        setIntegerParam(ADNumImagesCounter, number);
        getIntegerParam(NDArrayCounter, &count);
        count++;
        setIntegerParam(NDArrayCounter, count);
        callParamCallbacks();

        getIntegerParam(NDArrayCallbacks, &callback);
        if(callback) {
            NDArray *pImage;
            size_t dims[2];
            int itemp;
            char encodingString[MAX_FEATURE_NAME_LEN];
            AT_64 stride;
            int pixelSize;

            getIntegerParam(NDArraySizeX, &itemp); dims[0] = itemp;
            getIntegerParam(NDArraySizeY, &itemp); dims[1] = itemp;

            getEnumString(Andor3PixelEncoding, encodingString, sizeof(encodingString));
            if (strcmp(encodingString, "Mono32")==0) {
                pImage = pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);
                pixelSize = 4;
                setIntegerParam(NDDataType, NDUInt32);
            } else {
                pImage = pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);
                pixelSize = 2;
                setIntegerParam(NDDataType, NDUInt16);
            }
            if(pImage) {
                pImage->uniqueId = count;
                pImage->timeStamp = 631152000 + imageStamp.secPastEpoch +
                    (imageStamp.nsec / 1.0e9);
                updateTimeStamp(&pImage->epicsTS);

                AT_GetInt(handle_, L"AOIStride", &stride);
                if ((strcmp(encodingString, "Mono12")==0) || 
                    (strcmp(encodingString, "Mono16")==0) ||
                    (strcmp(encodingString, "Mono32")==0)) {
                    AT_U8 *p;

                    p = (AT_U8 *)pImage->pData;
                    for(int x = 0; x < size; x += (int)stride) {
                        memcpy(p, image+x, dims[0]*pixelSize);
                        p += dims[0]*pixelSize;
                    }
                } else if (strcmp(encodingString, "Mono12Packed")==0) {
                    AT_U8 *enc = image;
                    unsigned short *dec = (unsigned short*)pImage->pData;

                    for(int x = 0; x < size; x += (int)stride) {
                        enc = image + x;
                        for (size_t j = 0; j < dims[0]/pixelSize; j++) {
                            *dec     = (*enc << 4) + (*(enc+1) & 0xf);
                            *(dec+1) = (*(enc+2)<<4) + ((*(enc+1) >> 4) & 0xf);
                            enc += 3;
                            dec += pixelSize;
                        }
                    }
                }

                getAttributes(pImage->pAttributeList);

                unlock();
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                lock();

                pImage->release();
            }
        }
        getEnumString(ADImageMode, modeString, sizeof(modeString));
        getIntegerParam(ADNumImages, &total);
        if((strcmp(modeString, "Fixed")==0) && number == total) {
            setShutter(0);
            AT_Command(handle_, L"AcquisitionStop");
            setIntegerParam(ADAcquire, 0);
        }
        callParamCallbacks();

        status = AT_QueueBuffer(handle_, image, size);
        if(status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: AT_QueueBuffer 2: error=%d\n", 
                driverName, functionName, status);
        }
    }
}

void tucsen::tempTask(void)
{
    int status;
    static const char *functionName = "tempTask";

    while(!exiting_) {
        status  = getFeature(ADTemperatureActual);
        status |= getFeature(Andor3SensorCooling);
        status |= getFeature(Andor3TempStatus);

        if(status && (status != AT_ERR_NOTIMPLEMENTED)) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: temperature read error = %d\n", 
                driverName, functionName, status);
        }
        epicsThreadSleep(1.0);
    }
}

void tucsen::report(FILE *fp, int details)
{
    char str[MAX_FEATURE_NAME_LEN];
    
    getStringParam(Andor3SoftwareVersion, sizeof(str), str);
    fprintf(fp, "Andor driver, SDK version=%s\n", str);
    if (details < 1) return;

    reportFeature(ADModel, fp, details);
    reportFeature(Andor3SerialNumber, fp, details);
    reportFeature(Andor3FirmwareVersion, fp, details);
    reportFeature(Andor3ControllerID, fp, details);
    reportFeature(ADMaxSizeX, fp, details);
    reportFeature(ADMaxSizeY, fp, details);
    reportFeature(Andor3FullAOIControl, fp, details);
    reportFeature(ADSizeX, fp, details);
    reportFeature(ADSizeY, fp, details);
    reportFeature(ADMinX, fp, details);
    reportFeature(ADMinY, fp, details);
    reportFeature(Andor3Binning, fp, details);
    reportFeature(Andor3ShutterMode, fp, details);
    reportFeature(Andor3PixelEncoding, fp, details);
    reportFeature(Andor3ReadoutRate, fp, details);
    reportFeature(Andor3FrameRate, fp, details);
    reportFeature(Andor3Overlap, fp, details);
    reportFeature(Andor3PreAmpGain, fp, details);
    reportFeature(Andor3ReadoutTime, fp, details);
    reportFeature(Andor3TransferRate, fp, details);
    reportFeature(Andor3NoiseFilter, fp, details);
    reportFeature(ADImageMode, fp, details);
    reportFeature(ADAcquireTime, fp, details);
    reportFeature(ADNumExposures, fp, details);
    reportFeature(ADNumImages, fp, details);
    reportFeature(ADStatus, fp, details);
    reportFeature(ADTriggerMode, fp, details);
    reportFeature(Andor3SoftwareTrigger, fp, details);
    reportFeature(Andor3SensorCooling, fp, details);
    reportFeature(ADTemperature, fp, details);
    reportFeature(ADTemperatureActual, fp, details);
    reportFeature(Andor3TempControl, fp, details);
    reportFeature(Andor3TempStatus, fp, details);
    reportFeature(Andor3FanSpeed, fp, details);
    
    ADDriver::report(fp, details);
}    

int tucsen::allocateBuffers(void)
{
	int status;
	long long size;

	
	freeBuffers();

	size = frameHandle_.uiImgSize;
	imgSize_ = size;

    drvBuffers_ = (unsigned char **)calloc(maxFrames_, sizeof(unsigned char *));
    if(drvBuffers_) {
        for(int x = 0; x < maxFrames_; x++) {
			/* allocate 8 byte aligned buffer */
			if(!posix_memalign((void **)&drvBuffers_[x], 8, size)) {
				//status |= AT_QueueBuffer(handle_,
				//                         drvBuffers[x], (int)size);
			} else {   
				drvBuffers_[x] = NULL;
			}
        }
    }

    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:allocateBuffers: Failed to allocate and queue buffers\n",
            driverName);
    } else {
      setIntegerParam(NDArraySize, size);
	} 

    return status;
}

int andor3::freeBuffers()
{
    int status;

    status = AT_Flush(handle_);
    if(drvBuffers_) {
        for(int x = 0; x < maxFrames_; x++) {
            if(drvBuffers_[x]) {
                #ifdef _WIN32
                    _aligned_free(drvBuffers_[x]);
                #else
                    free(drvBuffers_[x]);
                #endif
                drvBuffers_[x] = NULL;
            }
        }
        free(drvBuffers_);
        drvBuffers_ = NULL;
    }
    return status;
}

    
int tucsen::connectCamera(void)
{
    static const char *functionName = "connectCamera";

    int status = TUCAMRET_SUCCESS;

    /* disconnect any connected camera first */
    disconnectCamera();

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: connecting camera %d\n",
        driverName, functionName, id_);

	/* Initialize API */
	status  = TUCAM_Api_Init(&apiHandle_)
    /* open handle to camera */
    status |= TUCAM_Dev_Open(&camhandle_);
    if(status != TUCAMRET_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to open camera %d\n",
            driverName, functionName, id_);
        return status;
    }

    allocateBuffers();
    return status;
}

int tucsen::disconnectCamera(void) 
{
    static const char *functionName = "disconnectCamera";

    int status;
    int acquiring;

    if(!camhandle_) {
        return TUCAMRET_SUCCESS;
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: disconnecting camera %d\n",
        driverName, functionName, id_);

    status = GetIntegerParam("CameraAcquiring", &acquiring);
    if(status == TUCAMRET_SUCCESS && acquiring) {
		// Command camera to stop acquiring
        status |= AT_Command(handle_, L"Acquisition Stop");
    }

    status |= freeBuffers();
	status |= TUCAM_Dev_Close(camHandle_.hldxTUCam); // Close camera
	status |= TUCAM_Api_Uninit();

    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error closing camera %d\n",
            driverName, functionName, id_);
    }

    apiHandle_ = 0;
    camHandle_ = 0;
    return status;
}

asynStatus tucsen::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    static const char *functionName = "writeInt32";
    const char *paramName;

    int index = pasynUser->reason;
    int status;

    getParamName(index, &paramName);

    /* set "locked" parameters */
    if(index == NDDataType) {
        value = NDUInt16;
    } else if(index == NDColorMode) {
        value = NDColorModeMono;
    }

    /* set parameter from value */
    status = setIntegerParam(index, value);
    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: failed to write parameter %s = %d\n",
            driverName, functionName, paramName, value);
        return (asynStatus)status;
    }

    if(index == ADAcquire) {
        if(value) {
            setIntegerParam(ADNumImagesCounter, 0);
            setShutter(1);
            epicsEventSignal(startEvent_);
        } else {
            setShutter(0);
            status = AT_Command(handle_, L"AcquisitionStop");
        }
    }
    else if(index == ADImageMode) {
        status = setFeature(ADImageMode);        
    }
    else if(index == ADTriggerMode) {
        status = setFeature(ADTriggerMode);
    }
    else if(index == ADNumExposures) {
        // The SDK requires FrameCount to be desired number of frames times AccumulateCount
        int numImages;
        status  = getIntegerParam(ADNumImages, &numImages);
        status |= setIntegerParam(ADNumImages, value * numImages);
        status |= setFeature(ADNumImages);
        status |= setIntegerParam(ADNumImages, numImages);
        status |= setFeature(ADNumExposures);
    }
    else if(index == ADNumImages) {
        // The SDK requires FrameCount to be desired number of frames times AccumulateCount
        int numExposures;
        status = getIntegerParam(ADNumExposures, &numExposures);
        status |= setIntegerParam(ADNumImages, value * numExposures);
        status |= setFeature(ADNumImages);
        status |= setIntegerParam(ADNumImages, value);
    }
    else if 
       ((index == Andor3Binning) ||
        (index == ADMinX)  ||
        (index == ADSizeX) ||
        (index == ADMinY)  ||
        (index == ADSizeY)) {
        status = setAOI();
    }
    else if(index == ADReadStatus) {
        status  = getFeature(ADStatus);
        status |= getAOI();
    }
    else if(index == Andor3PixelEncoding) {
        status = setFeature(Andor3PixelEncoding);
        if(status == AT_SUCCESS) {
            status = allocateBuffers();
        }
    }
    else if(index == Andor3SensorCooling) {
        status = setFeature(Andor3SensorCooling);
    }
    else if(index == Andor3ShutterMode) {
        status = setFeature(Andor3ShutterMode);
    }
    else if(index == Andor3SoftwareTrigger) {
        if(value) {
            status = AT_Command(handle_, L"SoftwareTrigger");
            setIntegerParam(Andor3SoftwareTrigger, 0);
        }
    }
    else if(index == Andor3TempControl) {
        status = setFeature(Andor3TempControl);
    }
    else if(index == Andor3Overlap) {
        status = setFeature(Andor3Overlap);
    }
    else if(index == Andor3FanSpeed) {
        status = setFeature(Andor3FanSpeed);
    }
    else if(index == Andor3ReadoutRate) {
        status = setFeature(Andor3ReadoutRate);
    }
    else if(index == Andor3PreAmpGain) {
        status = setFeature(Andor3PreAmpGain);
    }
    else if(index == Andor3NoiseFilter) {
        status = setFeature(Andor3NoiseFilter);
    }
    else {
        if(index < FIRST_ANDOR3_PARAM) {
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%d\n",
            driverName, functionName, status, paramName, index, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%d\n",
            driverName, functionName, paramName, index, value);
    }
    return (asynStatus)status;
}

asynStatus tucsen::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    static const char *functionName = "writeFloat64";
    const char *paramName;

    int index = pasynUser->reason;
    int status = asynSuccess;

    getParamName(index, &paramName);

    /* set parameter from value */
    status = setDoubleParam(index, value);
    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: failed to write parameter %s = %f\n",
            driverName, functionName, paramName, value);
        return (asynStatus)status;
    }

    if(index == ADTemperature) {
        status = setFeature(ADTemperature);
    }
    else if(index == ADAcquireTime) {
        status = setFeature(ADAcquireTime);
    }
    else if(index == ADAcquirePeriod) {
        status = setDoubleParam(Andor3FrameRate, value ? 1.0/value : 0);
        if(!status) {
            status = setFeature(Andor3FrameRate);
        }
    }
    else if(index == Andor3FrameRate) {
        status = setFeature(Andor3FrameRate);
    }
    else {    
        if(index < FIRST_ANDOR3_PARAM) {
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%f\n",
            driverName, functionName, status, paramName, index, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%f\n",
            driverName, functionName, paramName, index, value);
    }
    return (asynStatus)status;
}



extern "C" int tucsenConfig(const char *portName, int cameraId, int maxBuffers,
                            size_t maxMemory, int priority, int stackSize,
                            int maxFrames)
{
    new tucsen(portName, cameraId, maxBuffers, maxMemory, priority, stackSize,
               maxFrames);
    return(asynSuccess);
}

/** Constructor for Tucsen driver; most parameters are simply passed to
  * ADDriver::ADDriver.
  *
  * After calling the base class constructor this method creates a thread to
  * collect the images from the detector and sets reasonable default values for
  * the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  *
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] cameraId The id number of the Andor camera (see listdevices
  *            example for number).
  * \param[in] maxBuffers The maximum number of NDArray buffers that the
  *            NDArrayPool for this driver is allowed to allocate. Set this to
  *            -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for
  *            this driver is allowed to allocate. Set this to -1 to allow an
  *            unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread
  *            if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if
  *            ASYN_CANBLOCK is set in asynFlags.
  * \param[in] maxFrames The number of frame buffers to use in driver.
  */
tucsen::tucsen(const char *portName, int cameraId, int maxBuffers,
               size_t maxMemory, int priority, int stackSize, int maxFrames)
    : ADDriver(portName, 1, NUM_TUCSEN_PARAMS, maxBuffers, maxMemory,
               asynEnumMask, asynEnumMask, 
               ASYN_CANBLOCK,  /* ASYN_CANBLOCK=1 ASYN_MULTIDEVICE=0 */
               1,              /* autoConnect=1 */
               priority, stackSize),
      handle_(0), id_(cameraId), drvBuffers_(NULL), exiting_(0)
{
    static const char *functionName = "andor3";
    int status;

    /* set max frames */
    if(maxFrames == 0) {
        maxFrames_ = 10;
    } else {
        maxFrames_ = maxFrames;
    }
    
    /* create andor specific parameters */
    createParam(Andor3FrameRateString,        asynParamFloat64, &Andor3FrameRate);
    createParam(Andor3PixelEncodingString,    asynParamInt32,   &Andor3PixelEncoding);
    createParam(Andor3FullAOIControlString,   asynParamInt32,   &Andor3FullAOIControl);
    createParam(Andor3BinningString,          asynParamInt32,   &Andor3Binning);
    createParam(Andor3ShutterModeString,      asynParamInt32,   &Andor3ShutterMode);
    createParam(Andor3SoftwareTriggerString,  asynParamInt32,   &Andor3SoftwareTrigger);
    createParam(Andor3SensorCoolingString,    asynParamInt32,   &Andor3SensorCooling);
    createParam(Andor3TempControlString,      asynParamInt32,   &Andor3TempControl);
    createParam(Andor3TempStatusString,       asynParamInt32,   &Andor3TempStatus);
    createParam(Andor3SerialNumberString,     asynParamOctet,   &Andor3SerialNumber);
    createParam(Andor3FirmwareVersionString,  asynParamOctet,   &Andor3FirmwareVersion);
    createParam(Andor3SoftwareVersionString,  asynParamOctet,   &Andor3SoftwareVersion);
    createParam(Andor3ControllerIDString,     asynParamOctet,   &Andor3ControllerID);
    createParam(Andor3OverlapString,          asynParamInt32,   &Andor3Overlap);
    createParam(Andor3ReadoutRateString,      asynParamInt32,   &Andor3ReadoutRate);
    createParam(Andor3ReadoutTimeString,      asynParamFloat64, &Andor3ReadoutTime);
    createParam(Andor3TransferRateString,     asynParamFloat64, &Andor3TransferRate);
    createParam(Andor3PreAmpGainString,       asynParamInt32,   &Andor3PreAmpGain);
    createParam(Andor3NoiseFilterString,      asynParamInt32,   &Andor3NoiseFilter);
    createParam(Andor3FanSpeedString,         asynParamInt32,   &Andor3FanSpeed);

    featureInfo_ = (featureInfo *)calloc(LAST_ANDOR3_PARAM+1, sizeof(featureInfo));
        
    /* set read-only parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");

    /* open camera (also allocates frames) */
    status = AT_InitialiseLibrary();
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: Andor Library initialization failed (%d)\n",
            driverName, functionName, status);
        return;
    }
    AtInitialized++;

    status = connectCamera();
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        return;
    }

    status  = setStringParam(ADManufacturer, "Andor");

    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: failed to register all features for camera %d\n",
            driverName, functionName, id_);
    }

    startEvent_ = epicsEventCreate(epicsEventEmpty);

    /* launch image read task */
	/*
    epicsThreadCreate("TucsenImageTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_imagetask, this);
	*/

    /* launch temp read task */
    epicsThreadCreate("TucsenTempTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_temptask, this);

    /* shutdown on exit */
    epicsAtExit(c_shutdown, this);

}



/* Code for iocsh registration */
static const iocshArg tucsenConfigArg0 = {"Port name", iocshArgString};
static const iocshArg tucsenConfigArg1 = {"CameraId", iocshArgInt};
static const iocshArg tucsenConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg tucsenConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg tucsenConfigArg4 = {"priority", iocshArgInt};
static const iocshArg tucsenConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg tucsenConfigArg6 = {"maxFrames", iocshArgInt};
static const iocshArg * const tucsenConfigArgs[] =  {&tucsenConfigArg0,
                                                     &tucsenConfigArg1,
                                                     &tucsenConfigArg2,
                                                     &tucsenConfigArg3,
                                                     &tucsenConfigArg4,
                                                     &tucsenConfigArg5,
                                                     &tucsenConfigArg6};
static const iocshFuncDef configTucsen = {"tucsenConfig", 7, tucsenConfigArgs};
static void configTucsenCallFunc(const iocshArgBuf *args)
{
    tucsenConfig(args[0].sval, args[1].ival, args[2].ival,  args[3].ival, 
                 args[4].ival, args[5].ival, args[6].ival);
}

static void tucsenRegister(void)
{

    iocshRegister(&configTucsen, configTucsenCallFunc);
}

extern "C" {
epicsExportRegistrar(tucsenRegister);
}
