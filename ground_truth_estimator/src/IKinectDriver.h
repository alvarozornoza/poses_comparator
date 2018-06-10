//
//  IKinect.h
//  Skeleton Tracker
//
//  Created by Abdallah Dib on 4/25/13.
//  Distributed under GPL license
//

#ifndef denseRgbd_IKinect_h
#define denseRgbd_IKinect_h


//uncoment to decouple fake kinect from vvsision basic types ( this makes a standalone file independent from vvision utils)
//#define LINK_TO_VVISION

#ifdef LINK_TO_VVISION
    #include "basic_types.h"
#else
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>


using namespace std;


/* 8 bit unsigned variable.*/
typedef unsigned char		uchar8;

/* 8 bit signed variable.*/
typedef signed char		schar8;

/** 8 bit character variable. */
typedef char			char8;

/** 16 bit unsigned variable. */
typedef unsigned short		ushort16;

/** 16 bit signed variable.*/
typedef signed short		sshort16;

/** 32 bit unsigned variable.*/
typedef unsigned int		uint32;

/** 32 bit signed variable.*/
typedef signed int		sint32;

/** 32 bit signed variable.*/
typedef  int		int32;

/** 32 bit floating point variable.*/
typedef float				float32;

/** 64 bit floating point variable.*/
typedef double				float64;

#define SAFE_DELETE(ptr) \
if(ptr != NULL) \
{delete ptr; ptr = NULL;}

//maximum finite value
#define VV_MAXIMUM_FLOAT        std::numeric_limits<float32>::max( )
#define VV_MAXIMUM_DOUBLE       std::numeric_limits<float64>::max( )
#define VV_MAXIMUM_INTEGER      std::numeric_limits<int32>::max( )
#define VV_MAXIMUM_CHAR        std::numeric_limits<char8>::max( )
#define VV_MAXIMUM_SHOR_INT    std::numeric_limits<sshort16>::max( )

#endif

/** kinect led color*/
enum KINECT_LED_COLOR
{
    kKINECT_LED_COLOR_OFF                 = 0,
    kKINECT_LED_COLOR_GREEN               = 1 ,
    kKINECT_LED_COLOR_RED                 = 2,
    kKINECT_LED_COLOR_YELLOW              = 3 ,
    kKINECT_LED_COLOR_BLINK_YELLOW        = 4,
    kKINECT_LED_COLOR_BLINK_GREEN         = 5,
    kKINECT_LED_COLOR_BLINK_RED_YELLOW    = 6
};

/** motor status*/
enum KINECT_TILT_MOTOR_STATUS
{
    kKINECT_TILT_MOTOR_STATUS_STOPPED   = 0x00,
    kKINECT_TILT_MOTOR_STATUS_LIMIT    = 0x01,
    kKINECT_TILT_MOTOR_STATUS_MOVING   = 0x04,
    kKINECT_TILT_MOTOR_STATUS_UNKNOWN   = 0x08
};

/** kinect status*/
enum KINECT_STATUS
{
    kKINECT_STATUS_UNKNOWN = -1,
    kKINECT_STATUS_STOPPED = 0,
    kKINECT_STATUS_RUNNING,
    kKINECT_STATUS_SUSPENDED
};

#define KINECT_IMAGE_WIDTH		640
#define KINECT_IMAGE_HEIGHT		480
#define MAX_TILT_ANGLE          31.f           // The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
#define MIN_TILT_ANGLE          -31.f        // The control input looks like 2*desired_degrees

/** @brief abstract base kinect class*/
class IKinectDriver
{
    
public:
    
    /** base constructor*/
    IKinectDriver() {}
    
    /** destructor*/
    virtual ~IKinectDriver() {}
    
    
#ifdef MULTI_THREADED
    /** @inherited from Thread*/
    virtual void run() = 0;
#endif
    
    /*!
     \brief Open and connect to the Kinect device
     \return 0 success
     \return -1 if failed ( check logger for error)
     */
    virtual int32 OpenDevice() = 0;
    
	/**
	 Close the driver
	 this function will sync the main thread with the kinect driver.
	 after this call the KinectDriver object can be deleted safely...
	 */
    virtual bool Close() = 0;
    
#ifdef MULTI_THREADED
    /**
	 *request to suspend driver ( dont not call suspend() directly!)
	 */
    virtual void RequestSuspend() = 0;
	/**
	 *request to resume driver ( guarantee that the driver will resume
	 */
    virtual void RequestResume() = 0;
#endif
    
    /** acquire image, this function block the caller until a syncronized imge is  acquired by the kinect
     @return -2: failed to synchronize with depth generator -1: failed to synchronize with rgb generator 0: success
     */
    virtual int32 SyncAcquire() = 0;
    
    /**
     get depth/rgb images for the last acquire call
	 */
    virtual ushort16 *GetDepthBuffer() = 0;
    virtual uchar8 *GetRgbBuffer()  = 0;
    
    /** getters for image size*/
    virtual int32 GetDepthType() = 0;
    virtual int32 GetRgbType() = 0;
    virtual int32 GetDepthWidth() = 0;
    virtual int32 GetDepthHeight() = 0;
    
    virtual int32 GetRgbWidth() = 0;
    virtual int32 GetRgbHeight() = 0;
    
    /** set kinect led color*/
    virtual void SetLEDColor(KINECT_LED_COLOR led_color)  = 0;
    
    /** set tilt angle (control with half angle -28..28)
	 * this function is blocking, the caller will be blocked until tilt reach destination +-2 degrees
	 */
    virtual void SetTiltAngle(int32 tiltAngle) = 0;
    
    /** get tilt angle*/
    virtual int32 GetTiltAngle() = 0;
    
    /** get tilt motor status*/
    virtual KINECT_TILT_MOTOR_STATUS GetTiltMotorStatus() = 0;
   
    /** print last error*/
    virtual void PrintLastError() {}

    /** get the time stamp of the last acquisation*/
    virtual double GetDepthTimeStamp() {return 0.0;}
    virtual double GetRgbTimeStamp() {return 0.0;}
};

#endif
