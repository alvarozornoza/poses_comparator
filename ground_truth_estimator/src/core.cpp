//
//  core.cpp
//  Skeleton Tracker
//
//  Created by Abdallah Dib on 4/25/13.
//  Distributed under GPL license
//

#include "FakeKinectDriver.h"
#include <nlohmann/json.hpp>


using namespace std;
using namespace cv;

using json = nlohmann::json;

/** @brief camera intrinsics params*/

#define ROBONECT_CAMERA_WIDTH   640
#define ROBONECT_CAMERA_HEIGHT  480

#define ROBONECT_CAMERA_FX      521.179233
#define ROBONECT_CAMERA_FY      493.033034

#define ROBONECT_CAMERA_CX      322.515987
#define ROBONECT_CAMERA_CY      259.055966

#define K1  5.858325e-02
#define K2  3.856792e-02
#define P1  0
#define P2  0
#define K3  0

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

/** homogeneous transformation matrix between Qualisys and kinect*/
cv::Mat transformationMatrix;


/** vec2f struct*/
struct vec2f
{
    float x;
    float y;
};

/** vec3f struct */
struct vec3f
{
    float x;
    float y;
    float z;
    vec3f()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    vec3f(float x_, float y_, float z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }
    vec3f(const vec3f& oth)
    {
        x = oth.x;
        y = oth.y;
        z = oth.z;
    }
    vec3f operator=(const vec3f& oth)
    {
        x = oth.x;
        y = oth.y;
        z = oth.z;
        return *this;
    }
    vec3f operator+(const vec3f& oth)
    {
        return vec3f(this->x + oth.x,
                     this->y + oth.y,
                     this->z + oth.z);
    }
    vec3f operator-(const vec3f& oth)
    {
        return vec3f(this->x - oth.x,
                     this->y - oth.y,
                     this->z - oth.z);
    }
    bool exists()
    {
        return (((this->x)>0.0) && ((this->y)>0.0) && ((this->z)>0.0));
    }

    float length()
    {
        return sqrtf(x * x +  y * y + z *z) ;
    }

    bool operator ==(const vec3f& rhs)
    {
        const double EPSILON = 4.37114e-05;

        return (std::abs(x - rhs.x) < EPSILON) && (std::abs(y - rhs.y) < EPSILON);

    }
};

/** struct that holds different marker positions*/
struct Markers
{
    //time stamp
    double timeSec;

    //markers
    vec3f head;
    vec3f leftShoulder;
    vec3f leftElbow;
    vec3f leftHand;

    vec3f rightShoulder;
    vec3f rightElbow;
    vec3f rightHand;

    vec3f neck;
    vec3f torso;

    vec3f leftHip;
    vec3f leftKnee;
    vec3f LeftFoot;

    vec3f rightHip;
    vec3f rightKnee;
    vec3f rightFoot;
};

/** convert opencv vector to vec3 format*/
vec3f opencvVecToVec3(const Mat& vert)
{
    vec3f vec;
    vec.x = vert.at<float>(0, 0) ;
    vec.y = vert.at<float>(1, 0) ;
    vec.z = vert.at<float>(2, 0) ;
    return vec;
}


/** convert vec3 to opencv format*/
cv::Mat vec3fToOpencv(const vec3f& vec)
{
    Mat vert = Mat::zeros(4, 1, CV_32FC1);
    vert.at<float>(0, 0) = vec.x;
    vert.at<float>(1, 0) = vec.y;
    vert.at<float>(2, 0) = vec.z;
    vert.at<float>(3, 0) = 1;
    return vert;
}

/** unidstort pixel*/
Point2i UndistortPixel(Point2i point)
{
    float cx = ROBONECT_CAMERA_CX;
    float cy = ROBONECT_CAMERA_CY;
    float fx = ROBONECT_CAMERA_FX;
    float fy = ROBONECT_CAMERA_FY;

    float ifx = 1.0 / fx;
    float ify = 1.0 / fy;

    float k1 = K1;
    float k2 = K2;
    float k3 = K3;
    float p1 = P1;
    float p2 = P2;

    float x, y, x0, y0;
    x = point.x;
    y = point.y;
    x = (x - cx) * ifx;	x0 = x;
    y = (y - cy) * ify;	y0 = y;

    float r2 = x*x + y*y;
    float icdist = 1.0 / (1.0 + ((k3 * r2 + k2)*r2 +k1)*r2);
    float deltaX = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x *x);
    float deltaY = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

    x = (x0 - deltaX) * icdist;
    y = (y0 - deltaY) * icdist;

    Point2i final;
    final.x = x * fx + cx;
    final.y = y * fy + cy;
    return final;

}

/** perspective projection of a 3D point to a 2D pixel*/
Point2f project3DPoint(Mat vert)
{
    float cx = ROBONECT_CAMERA_CX;
    float cy = ROBONECT_CAMERA_CY;
    float fx = ROBONECT_CAMERA_FX;
    float fy = ROBONECT_CAMERA_FY;

    Point2f  pixel;
    pixel.x = fx * vert.at<float>(0, 0) / vert.at<float>(2, 0) + cx;
    pixel.y = fy * vert.at<float>(1, 0) / vert.at<float>(2, 0) + cy;
    return pixel;

}


/** convert depth image returned by the kinect from 16 bit to float 32 bits*/
void convertDepthFrom16UC1To32FC1(const cv::Mat& input, cv::Mat& output, float scale)
{
    output.create(input.rows, input.cols, CV_32FC1);

    const unsigned short* inputPtr = input.ptr<unsigned short>();
    float* outputPtr = output.ptr<float>();

    for(int idx = 0; idx < input.size().area(); idx++, inputPtr++, outputPtr++)
    {
        if(*inputPtr == 0)
        {
            *outputPtr = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            float d = (float)*inputPtr;
            *outputPtr = ((float) *inputPtr) * scale;
        }
    }
}

/** parse markers position from a string
    @param marker the output struct that holds the markers positions
    @param index the time stamp of the parsed frame
    @line the input string
*/
void scanMarkerFromString(Markers& marker, double &index, char* line)
{
    sscanf(line,
           "%lf\t%lf\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f",
           &index, &marker.timeSec,
           &marker.head.x, &marker.head.y, &marker.head.z,
           &marker.leftShoulder.x, &marker.leftShoulder.y, &marker.leftShoulder.z,
           &marker.leftElbow.x, &marker.leftElbow.y, &marker.leftElbow.z,
           &marker.leftHand.x, &marker.leftHand.y, &marker.leftHand.z,

           &marker.rightShoulder.x, &marker.rightShoulder.y, &marker.rightShoulder.z,
           &marker.rightElbow.x, &marker.rightElbow.y, &marker.rightElbow.z,
           &marker.rightHand.x, &marker.rightHand.y, &marker.rightHand.z,

           &marker.neck.x, &marker.neck.y, &marker.neck.z,
           &marker.torso.x, &marker.torso.y, &marker.torso.z,

           &marker.leftHip.x, &marker.leftHip.y, &marker.leftHip.z,
           &marker.leftKnee.x, &marker.leftKnee.y, &marker.leftKnee.z,
           &marker.LeftFoot.x, &marker.LeftFoot.y, &marker.LeftFoot.z,

           &marker.rightHip.x, &marker.rightHip.y, &marker.rightHip.z,
           &marker.rightKnee.x, &marker.rightKnee.y, &marker.rightKnee.z,
           &marker.rightFoot.x, &marker.rightFoot.y, &marker.rightFoot.z
           );
}


/** print a marker to a string
    @param marker the marker to print to the string
    @string the output string where marker is printed
    @param the rgb time stamp
    @param the depth time stamp
*/
void printMarkerToString(Markers& marker, char* string, double timestampD, double timestampR)
{

    sprintf(string,
           "%lf\t%lf\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\t\
           %f\t%f\t%f\n",
           timestampD, timestampR,
           marker.head.x, marker.head.y, marker.head.z,
           marker.leftShoulder.x, marker.leftShoulder.y, marker.leftShoulder.z,
           marker.leftElbow.x, marker.leftElbow.y, marker.leftElbow.z,
           marker.leftHand.x, marker.leftHand.y, marker.leftHand.z,

           marker.rightShoulder.x, marker.rightShoulder.y, marker.rightShoulder.z,
           marker.rightElbow.x, marker.rightElbow.y, marker.rightElbow.z,
           marker.rightHand.x, marker.rightHand.y, marker.rightHand.z,

           marker.neck.x, marker.neck.y, marker.neck.z,
           marker.torso.x, marker.torso.y, marker.torso.z,

           marker.leftHip.x, marker.leftHip.y, marker.leftHip.z,
           marker.leftKnee.x, marker.leftKnee.y, marker.leftKnee.z,
           marker.LeftFoot.x, marker.LeftFoot.y, marker.LeftFoot.z,

           marker.rightHip.x, marker.rightHip.y, marker.rightHip.z,
           marker.rightKnee.x, marker.rightKnee.y, marker.rightKnee.z,
           marker.rightFoot.x, marker.rightFoot.y, marker.rightFoot.z
           );
}

int getJointIndex(std::string &Joint)
{
    std::vector<std::string> joints = {"torso", "head", "leftShoulder", "leftElbow", "leftHand", "rightShoulder",
     "rightElbow", "rightHand", "neck", "leftHip", "leftKnee", "leftFoot", "rightHip", "rightKnee", "rightFoot"};
    return find(joints.begin(), joints.end(), Joint) - joints.begin();
}

template <size_t rows, size_t cols>
void getIndexes(int(&array)[rows][cols])
{
    std::string connectedJointsStrings[][2] = {
    {"head", "neck"},
    {"neck","torso"},
    {"leftShoulder", "neck"},
    {"leftElbow", "leftShoulder"},
    {"leftElbow", "leftHand"},
    {"leftHip", "torso"},
    {"leftHip", "leftKnee"},
    {"leftKnee", "leftFoot"},
    {"rightShoulder", "neck"},
    {"rightElbow", "rightShoulder"},
    {"rightElbow", "rightHand"},
    {"rightHip", "torso"},
    {"rightHip", "rightKnee"},
    {"rightKnee", "rightFoot"},
    {"leftShoulder", "rightShoulder"}
    };
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            array[i][j] = getJointIndex(connectedJointsStrings[i][j]);
        }
    }
}

void ptsPushBack(Markers& marker, std::vector<vec3f>& pts)
{
    pts.push_back(marker.torso);
    pts.push_back(marker.head);
    pts.push_back(marker.leftShoulder);
    pts.push_back(marker.leftElbow);
    pts.push_back(marker.leftHand);
    pts.push_back(marker.rightShoulder);
    pts.push_back(marker.rightElbow);
    pts.push_back(marker.rightHand);
    pts.push_back(marker.neck);
    pts.push_back(marker.leftHip);
    pts.push_back(marker.leftKnee);
    pts.push_back(marker.LeftFoot);
    pts.push_back(marker.rightHip);
    pts.push_back(marker.rightKnee);
    pts.push_back(marker.rightFoot);
}

/** display marker position on a opencv image. markers are displayed as circles
    @param the marker to display on the image
    @param image the opencv image where the marker will be displayed on
    @param transformation the transformation matrix that project the marker positions from the Qualisys coordinate system to the camera coordinate system (obtained by rigid calibration)
    */

void displayPose(std::vector<vec3f>& pts, cv::Mat& image, const cv::Mat& transformation)
{
    int connectedJointIndices[14][2];
    getIndexes(connectedJointIndices);

    //cerr<<connectedJointIndices[0][0]<<"and"<<connectedJointIndices[0][1]<<"\n";

    int colors[18][3] = {{255, 0, 0}, {255, 85, 0}, {255, 170, 0}, {255, 255, 0}, {170, 255, 0}, {85, 255, 0}, {0, 255, 0}, 
          {0, 255, 85}, {0, 255, 170}, {0, 255, 255}, {0, 170, 255}, {0, 85, 255}, {0, 0, 255}, {85, 0, 255}, 
          {170, 0, 255}, {255, 0, 255}, {255, 0, 170}, {255, 0, 85}};


    for(int i = 0; i < pts.size();i++)
    {
        Mat pt = vec3fToOpencv(pts[i]);
        Mat transformedPt = transformation * pt;
        Point2i pixel = project3DPoint(transformedPt);
        cv::circle(image, pixel, 5,  Scalar(colors[i][0], colors[i][1], colors[i][2] ), -1, 8);
    }
    for(int i = 0; i < 15; i++)
    {
        int indices[2];
        indices[0] = connectedJointIndices[i][0];
        indices[1] = connectedJointIndices[i][1];
        if(pts[indices[0]].exists() && pts[indices[1]].exists())
        {
            Mat pt1 = vec3fToOpencv(pts[indices[0]]);
            Mat transformedPt1 = transformation * pt1;
            Point2i pixel1 = project3DPoint(transformedPt1);
            Mat pt2 = vec3fToOpencv(pts[indices[1]]);
            Mat transformedPt2 = transformation * pt2;
            Point2i pixel2 = project3DPoint(transformedPt2);
            cv::line(image, pixel1, pixel2, Scalar(colors[i][0], colors[i][1], colors[i][2]));
        }
    }
    cv::imshow("image", image);
}

json generateJSON(std::vector<vec3f>& pts, const cv::Mat& transformation, int indexImage)
{
    std::vector<std::string> joints = {"torso", "head", "leftShoulder", "leftElbow", "leftHand", "rightShoulder",
     "rightElbow", "rightHand", "neck", "leftHip", "leftKnee", "leftFoot", "rightHip", "rightKnee", "rightFoot"};

    json skeleton = {
        {"frameNumber", indexImage},
        {"score", 0.0},
        {"keypoints", json::array()}
    };

    for(int i = 0; i < pts.size();i++)
    {
        if(pts[i].exists())
            continue;
        Mat pt = vec3fToOpencv(pts[i]);
        Mat transformedPt = transformation * pt;
        Point2f pixel = project3DPoint(transformedPt);
        json keypoint = {
                    {"position",{
                        {"x", pixel.x},
                        {"y", pixel.y}
                    }},
                    {"partName",joints[i]},
                    {"score", 0.0}
                }; 
        skeleton["keypoints"][i] = keypoint;
    }
    return skeleton;
}

/** returns the transformation matrix between the qualisys and the kinect camera. it is a 4x4 homogeneous matrix (unit in meters)*/
Mat GetTransformationBetweenKinectAndGroundTruth()
{
    Mat transform = Mat::zeros(4, 4, CV_32FC1);
    transform.at<float>(3,3) = 1;

    transform.at<float>(0, 0) = 0.8954580151977138;
    transform.at<float>(0, 1) = 0.5103237861353643;
    transform.at<float>(0, 2) = -0.0508805743920705;
    transform.at<float>(0, 3) = -4507.420151100658;//meter

    transform.at<float>(1, 0) = -0.02123062193180733;
    transform.at<float>(1, 1) = -0.04180508717005156;
    transform.at<float>(1, 2) = -1.052294108532836;
    transform.at<float>(1, 3) = 1019.225762091929;//meetr

    transform.at<float>(2, 0) = -0.5224123887620519;
    transform.at<float>(2, 1) = 0.8888625338840541;
    transform.at<float>(2, 2) = -0.08742371986806169;
    transform.at<float>(2, 3) = 3469.696756240115;//meter

    return transform;
}

/** play a sequence. this function parse marker positions from groundTruth.txt file and display them on the corresponding synchronized rgb image of the kinect
    @param path the directory path of the sequence
    */
int playSequence(const std::string &path)
{

    cerr<<"[INFO] playing '"<<path<<"' sequence."<<endl;

    char line[1024];
    Markers marker;
    double index;
    ifstream gt;
    string groundTruthFileName = path + "/groundTruth.txt";
    json skeletonsData = json::array();
    ofstream myfile;
    myfile.open(path+"/json/example.txt");

    setlocale(LC_NUMERIC, "C");
    gt.open((groundTruthFileName).c_str());

    if(!gt.is_open() )
    {
        cerr<<"[ERROR] could not open ground truth file.. job canceled\n";
        return -2;
    }

    CFakeKinectDriver driver(path);
    cerr<<driver.OpenDevice();
    if(driver.OpenDevice() != FAKE_KINECT_NO_ERROR)
    {
        cerr<<"[ERROR] could not open fake kinect device.. job canceled\n";
        return -1;
    }

    int indexImage = 0;
    cv::Mat rgb_indDisplay;

    while(true)
    {
        cerr<<"[INFO] processing image at index: "<<indexImage<<endl;

        if(gt.eof())
        {
            break;
        }

        setlocale(LC_NUMERIC, "C");
        int res =  driver.SyncAcquire();

        if(res != FAKE_KINECT_NO_ERROR)
        {
            driver.PrintLastError();
            break;
        }

        uchar8* data_rgb = driver.GetRgbBuffer();
        //ushort16* data_depth = driver.GetDepthBuffer();

        cv::Mat rgb_in = cv::Mat(driver.GetRgbHeight(), driver.GetRgbWidth(), driver.GetRgbType(), data_rgb);
        rgb_indDisplay = rgb_in.clone();
        cv::cvtColor(rgb_indDisplay, rgb_indDisplay, CV_BGR2RGB);
        gt.getline(line, 1024);
        setlocale(LC_NUMERIC, "C");
        scanMarkerFromString(marker, index, line);
        std::vector<vec3f> pts;
        ptsPushBack(marker, pts);
        displayPose(pts, rgb_indDisplay, transformationMatrix);
        json skeleton;
        skeleton = generateJSON(pts ,transformationMatrix, indexImage);
        skeletonsData[indexImage] = skeleton;

        char key = cv::waitKey(20);

        if(key == 'q')
        {
            cerr<<"[INFO] process canceled by user"<<endl;
            break;
        }
        if(key == 'p')
        {
            cerr<<"[INFO] process paused by user. Press any key to continue"<<endl;
            cv::waitKey(0);
        }
        indexImage++;

    }
    myfile << skeletonsData.dump(4);
    myfile.close();
    cerr<<"[SUCCESS] finished playing '"<<path<<"' sequence."<<endl;
    return 0;
}

int main(int argc, char** argv)
{
    if(argc < 2 )
    {
        cerr<<"[INFO] usage: "<<argv[0]<<" 'sequenceDirectoryPath'"<<endl;
        return EXIT_FAILURE;
    }

    //1. must be called first to acquire the transformation matrix between the ground truth and the kinect(calibration must be done first)
    transformationMatrix = GetTransformationBetweenKinectAndGroundTruth();


    //2. play the sequence and project marker positions on the corresponding image
    //std::vector<string> sequences;
    //sequences.push_back("/home/dibabdal/Bureau/MySpace/INRIA/testDevs/calibOpencv/sequences/abdallah2BodyWalk1/");
    playSequence(argv[1]);


    cerr<<"[INFO] exit now...\n";
    return EXIT_SUCCESS;
}
