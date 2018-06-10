//
//  FakeKinect.h
//  Skeleton Tracker
//
//  Created by Abdallah Dib on 4/25/13.
//  Distributed under GPL license
//

#ifndef __denseRgbd__FakeKinect__
#define __denseRgbd__FakeKinect__

//fake kinect is not compatible on win32 TODO: remvove it from here and add it to cmake flags for system wide compilation
#define USE_FAKE_KINECT



#ifdef USE_FAKE_KINECT
    #ifdef _WIN32
        #error sorry, fake kinect does not compile on windows plateform. please remove USE_FAKE_KINECT from ur compiler flags and try again.
    #endif
#endif

#include "IKinectDriver.h"
#include <sstream>
#include <fstream>
#include <highgui.h> //opencv2/highgui/highgui.hpp
#include <stdio.h>
#include <opencv2/opencv.hpp>


#ifdef USE_FAKE_KINECT


#define FAKE_KINECT_NO_ERROR                                0
#define FAKE_KINECT_BAD_FILE                                -1
#define FAKE_KINECT_END_OF_FILE_REACHED                     -2
#define FAKE_KINECT_FAILED_TO_LOAD_DEPTH_IMAGE              -3
#define FAKE_KINECT_FAILED_TO_LOAD_RGB_IMAGE                -4
#define FAKE_KINECT_SYNC_FILE_NOT_FOUND                     -5
#define FAKE_KINECT_RGB_OR_DEPTH_DIRECTORY_NOT_FOUND        -6


/** struct representing ground truth camera pos*/
typedef struct CAMERA_POS_GROUND_TRUTH_
{
    
    /** translation*/
    float32 tx;
    float32 ty;
    float32 tz;
    
    /** quaternion representation*/
    float32 qx;
    float32 qy;//print last error message
    float32 qz;
    float32 qw;
    
    //streaming 
    friend std::ostream& operator<< (std::ostream& stream, const CAMERA_POS_GROUND_TRUTH_& pos) {
    
        stream<<"("<<pos.tx<<" "<<pos.ty<<" "<<pos.tz<<" "<<pos.qx<<" "<<pos.qy<<" "<<pos.qz<<" "<<pos.qw<<" )";
        return stream;
    }
}CAMERA_POS_GROUND_TRUTH;


class CFakeKinectDriver : public IKinectDriver
{
public:
    
    /** base constructor.
     @param path the path that contains the kinect data ( rgb depth ...)
     */
    CFakeKinectDriver(const std::string& path);
    
    /** destructor*/
     ~CFakeKinectDriver();
    
    
#ifdef MULTI_THREADED
    /** @inherited from Thread*/
     void run();
#endif
    
    /*!
     \brief Open and connect to the Kinect device
     \return 0 success
     \return -1 if failed ( check logger for error)
     */
     int32 OpenDevice();
    
	/**
	 Close the driver
	 this function will sync the main thread with the kinect driver.
	 after this call the KinectDriver object can be deleted safely...
	 */
     bool Close();
    
#ifdef MULTI_THREADED
    /**
	 *request to suspend driver ( dont not call suspend() directly!)
	 */
     void RequestSuspend();
	/**
	 *request to resume driver ( guarantee that the driver will resume
	 */
     void RequestResume();
#endif
    
    /** acquire image, this function block the caller until a syncronized imge is  acquired by the kinect
     @return -2: failed to synchronize with depth generator -1: failed to synchronize with rgb generator 0: success
     */
     int32 SyncAcquire();
    
    /**
     get depth/rgb images for the last acquire call
	 */
     ushort16 *GetDepthBuffer();
     uchar8 *GetRgbBuffer() ;
    CAMERA_POS_GROUND_TRUTH GetCameraPosition() {return m_CameraPos;}
    
    /** set kinect led color*/
    void SetLEDColor(KINECT_LED_COLOR led_color) {}
    
    /** set tilt angle (control with half angle -28..28)
	 * this function is blocking, the caller will be blocked until tilt reach destination +-2 degrees
	 */
    void SetTiltAngle(int32 tiltAngle) {}
    
    /** get tilt angle*/
    int32 GetTiltAngle() {return -1;}
    
    /** get tilt motor status*/
    KINECT_TILT_MOTOR_STATUS GetTiltMotorStatus() {return kKINECT_TILT_MOTOR_STATUS_UNKNOWN;}
    
    
    /** getters for image size*/
    int32 GetDepthType() {return CV_16UC1;}
    int32 GetRgbType() {return CV_8UC3;}
    int32 GetDepthWidth() {return KINECT_IMAGE_WIDTH;}
    int32 GetDepthHeight() {return KINECT_IMAGE_HEIGHT;}
    
    int32 GetRgbWidth() {return KINECT_IMAGE_WIDTH;}
    int32 GetRgbHeight() {return KINECT_IMAGE_HEIGHT;}
    
    /** return the last error ( check different error in the top of file*/
    inline int32 GetLastError() {return m_LastError;}
    
    /** @inherited from IKinectDriver*/
    virtual void PrintLastError();

    /** @inherited from IKinectDriver*/
    virtual double GetDepthTimeStamp() {return timeStampDepth;}
    virtual double GetRgbTimeStamp() {return timeStampRgb;}

protected:
    
    /** directory path*/
    std::ostringstream m_Path;

    /** text file path that holds all rgb depth and pose values */
    std::ostringstream m_TextFilePath;
    
    /** rgb path*/
    std::ostringstream m_RgbPath;
    
    /** depth path*/
    std::ostringstream m_DepthPath;
    
    /** file stream */
    std::ifstream m_Stream;
    
    /** 0 if the file_name exist on the drive*/
    int32 FileExist(const std::string& file_name);
    
    /** true if the directory exist*/
    int32 DirectoryExist(const std::string& file_name);
    
    /** parse a line from an ascii file
     @return 0 if success
     */
    int32 ReadLine(std::ifstream & file_stream, char8* output, int32 max_size_line);

    /** parse a line. return 0 if success*/
    int32  ParseLine(char8* line, char8* rgb_image, char8* depth_image, CAMERA_POS_GROUND_TRUTH& camera_pos);
    
    /** extract images and camera pos, return 0 on success*/
    int32 ExtractImagesAndCameraPos( );
    //holds depth image
    cv::Mat m_Rgb;
    std::string m_RgbName;
    
    //holds rgb imag
    cv::Mat m_Depth;
    std::string m_DepthName;
    
    CAMERA_POS_GROUND_TRUTH m_CameraPos;
    
    //holds last error call GetLastError to get the last error
    int32 m_LastError;
    
    /** frames number*/
    int32 m_FramesNumber;

    /** acquisation time*/
    float64 timeStampDepth;
    float64 timeStampRgb;

};


#endif /* defined(__denseRgbd__FakeKinect__) */

#endif
