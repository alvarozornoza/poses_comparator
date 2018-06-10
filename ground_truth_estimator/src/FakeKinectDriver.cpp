//
//  FakeKinect.cpp
//  Skeleton Tracker
//
//  Created by Abdallah Dib on 4/25/13.
//  Distributed under GPL license
//


#include "FakeKinectDriver.h"


#include <sys/types.h>  // for stat().
#include <sys/stat.h>   // for stat().

#define FAKE_KINECT_ACQUISATION_FILE_NAME "rgb_depth_pos.txt"
#define FAKE_KINECT_RGB_DIRECTORY "rgb"
#define FAKE_KINECT_DEPTH_DIRECTORY "depth"


#define FAKE_KINECT_CHECK_LAST_ERROR \
                            if(m_LastError !=0) \
                                    return m_LastError;


#ifdef FAKE_KINECT_ENABLE_VERBOSE
    #define FAKE_KINECT_LOG(msg...) std::cerr<<msg
#else
    #define FAKE_KINECT_LOG(msg...)
#endif

CFakeKinectDriver::CFakeKinectDriver(const std::string& path)
:IKinectDriver(),
  timeStampDepth(0.0),
  timeStampRgb(0.0)
{
    m_Path << path;
    m_TextFilePath<< path<<"/"<<FAKE_KINECT_ACQUISATION_FILE_NAME;
    m_RgbPath <<path <<"/"<<FAKE_KINECT_RGB_DIRECTORY<<"/";
    m_DepthPath <<path <<"/"<<FAKE_KINECT_DEPTH_DIRECTORY<<"/";
    m_LastError = FAKE_KINECT_NO_ERROR;
    m_FramesNumber = 0;
    
}
CFakeKinectDriver::~CFakeKinectDriver()
{
    Close();
}


int32 CFakeKinectDriver::OpenDevice()
{
    m_FramesNumber = 0;
    //check if the directory is valid
    
    //sync file exist
    m_LastError = FileExist(m_TextFilePath.str());
    FAKE_KINECT_CHECK_LAST_ERROR
    
    //rgb directory exist
    m_LastError = DirectoryExist(m_RgbPath.str());
    FAKE_KINECT_CHECK_LAST_ERROR
    
    //deph directory exist
    m_LastError = DirectoryExist(m_DepthPath.str());
    FAKE_KINECT_CHECK_LAST_ERROR
    
    //parse a line and try to open the first file in the sequence
    m_Stream.close();
    m_Stream.open(m_TextFilePath.str().c_str());
    m_LastError = ExtractImagesAndCameraPos();
    FAKE_KINECT_CHECK_LAST_ERROR
    
    //rewind
    m_Stream.seekg(0);
    
    return FAKE_KINECT_NO_ERROR;
}

bool CFakeKinectDriver::Close()
{
    m_Stream.close();
    m_Rgb.release();
    m_Depth.release();
    m_FramesNumber = 0;
    return true;
}


int32 CFakeKinectDriver::SyncAcquire()
{
    return ExtractImagesAndCameraPos();
    
}
ushort16* CFakeKinectDriver::GetDepthBuffer()
{
    return m_Depth.ptr<ushort16>();    
}
uchar8* CFakeKinectDriver::GetRgbBuffer()
{
    return m_Rgb.ptr<uchar8>();
}
int32 CFakeKinectDriver::FileExist(const std::string& file_name)
{
    ifstream file;
    file.open(file_name.c_str());
    
    if (file.is_open()) {
        file.close();
        return FAKE_KINECT_NO_ERROR;
    }
    return FAKE_KINECT_SYNC_FILE_NOT_FOUND;
}
int32 CFakeKinectDriver::DirectoryExist(const std::string& file_name)
{
#ifdef __linux__
    struct stat sb;
    if (stat(file_name.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
    {
        //...it is a directory...
        return FAKE_KINECT_NO_ERROR;
    }
    else
    {
        return FAKE_KINECT_RGB_OR_DEPTH_DIRECTORY_NOT_FOUND;
    }

#elif __APPLE__

    //the code bellow is not portable.
    if ( access( file_name.c_str(), 0 ) == 0 )
    {
        struct stat status;
        stat( file_name.c_str(), &status );
        
        if ( status.st_mode & S_IFDIR )
        {
            //directory
            return FAKE_KINECT_NO_ERROR;
        }
        else
        {
            //file
            return FAKE_KINECT_RGB_OR_DEPTH_DIRECTORY_NOT_FOUND;
            
        }
    }
    else
    {
        //doesnt exist
        return FAKE_KINECT_RGB_OR_DEPTH_DIRECTORY_NOT_FOUND;
    }

#else

    //TOOD handle windows
    return FAKE_KINECT_NO_ERROR;

#endif




}
int32 CFakeKinectDriver::ReadLine(std::ifstream & file_stream_, char8* output, int32 max_size_line)
{
    assert(output != NULL);
    
    file_stream_.getline(output, max_size_line);
    
    if(strlen(output) <=0 )
    {
        return FAKE_KINECT_END_OF_FILE_REACHED;
    }
    
    //end of file reached
    if(file_stream_.failbit & std::ios_base::eofbit)
    {
        return FAKE_KINECT_BAD_FILE;
    }
    
    if(file_stream_.failbit & std::ios_base::badbit)
    {
        return FAKE_KINECT_BAD_FILE;
    }

    return FAKE_KINECT_NO_ERROR;
}
int32 CFakeKinectDriver::ParseLine(char8* line, char8* rgb_image, char8* depth_image, CAMERA_POS_GROUND_TRUTH& camera_pos)
{
    assert(line != NULL);
    
    //float64 timestampDepth;
    //float64 timestampRgb;
    float64 timestampPos;

    sscanf(line, "%lf %s \
                    %lf %s \
                    %lf %f %f %f %f %f %f %f",
           &timeStampDepth,
           depth_image,
           
           &timeStampRgb,
           rgb_image,
           
           &timestampPos,
           &camera_pos.tx,
           &camera_pos.ty,
           &camera_pos.tz,
           &camera_pos.qx,
           &camera_pos.qy,
           &camera_pos.qz,
           &camera_pos.qw
           );
    
    return FAKE_KINECT_NO_ERROR;
}
int32 CFakeKinectDriver::ExtractImagesAndCameraPos( )
{
    
    char8 line[512];
    char8 rgb[128];
    char8 depth[128];
    
    if( !m_Stream.good())
    {
        m_LastError = FAKE_KINECT_BAD_FILE;
        return m_LastError;
    }
    
    m_LastError = ReadLine(m_Stream, line, 512);
    FAKE_KINECT_CHECK_LAST_ERROR
    
    FAKE_KINECT_LOG("CFakeKinectDriver: [INFO] parsing ("<<m_FramesNumber<<"): "<<line<<". size = "<<strlen(line)<<"\n");
    
    m_LastError = ParseLine(line, rgb, depth, m_CameraPos);
    FAKE_KINECT_CHECK_LAST_ERROR
    
    //try to load images
    m_RgbName = m_Path.str() + "/" + rgb;
    m_Rgb = cv::imread(m_RgbName, 1);
    if(m_Rgb.data == NULL)
    {
        m_LastError = FAKE_KINECT_FAILED_TO_LOAD_RGB_IMAGE;
        return m_LastError;
    }
 
    m_DepthName = m_Path.str() + "/" + depth;
    m_Depth = cv::imread(m_DepthName, -1);
    if(m_Depth.data == NULL)
    {
        m_LastError = FAKE_KINECT_FAILED_TO_LOAD_DEPTH_IMAGE;
        return m_LastError;
    }

    m_FramesNumber++;
    return FAKE_KINECT_NO_ERROR;
}
void CFakeKinectDriver::PrintLastError()
{
    switch (m_LastError) {
        case FAKE_KINECT_NO_ERROR:
            break;
        case FAKE_KINECT_BAD_FILE:
            FAKE_KINECT_LOG("CFakeKinectDriver: [ERROR] bad file. this generaly means that the stream contains wrong data formats.\n");
            break;
        case FAKE_KINECT_END_OF_FILE_REACHED:
            FAKE_KINECT_LOG("CFakeKinectDriver: [INFO] end of file reached.\n");
            break;
        case FAKE_KINECT_FAILED_TO_LOAD_DEPTH_IMAGE:
            FAKE_KINECT_LOG("CFakeKinectDriver: [ERROR] failed to load depth image: "<<m_DepthName<<"\n");
            break;
        case FAKE_KINECT_FAILED_TO_LOAD_RGB_IMAGE:
             FAKE_KINECT_LOG("CFakeKinectDriver: [ERROR] failed to load rgb image: "<<m_RgbName<<"\n");
            break;
        case FAKE_KINECT_SYNC_FILE_NOT_FOUND:
            FAKE_KINECT_LOG("CFakeKinectDriver: [ERROR] sync file '"<< FAKE_KINECT_ACQUISATION_FILE_NAME<<"' not found in: "<<m_Path.str()<<"\n");
            break;
        case FAKE_KINECT_RGB_OR_DEPTH_DIRECTORY_NOT_FOUND:
            FAKE_KINECT_LOG("CFakeKinectDriver: [ERROR] '"<<FAKE_KINECT_RGB_DIRECTORY<<"' or '"<<FAKE_KINECT_DEPTH_DIRECTORY<<"' directory not found in:"<<m_Path.str()<<"\n");
            break;
        default:
            break;
    }
}
