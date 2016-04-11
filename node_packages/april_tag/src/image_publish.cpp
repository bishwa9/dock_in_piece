#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

// following header files have been added for April Tags detection
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "april_tag/AprilTag.h" // rosmsg
#include "april_tag/AprilTagList.h" // rosmsg
#include <sys/time.h>

AprilTags::TagDetector* m_tagDetector;
const double PI = 3.14159265358979323846;
const double TWOPI = 2.0*PI;
//const char* windowName = "apriltags_demo";
/**
 * Normalize angle to be within the interval [-pi,pi].
 */

inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}


// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}



  bool m_draw=true; // draw image and April tag detections?
 
  int m_width=320; // image size in pixels
  int m_height=240;
  double m_tagSize=0.165; // April tag side length in meters of square black frame
  double m_fx=263.612; // camera focal length in pixels
  double m_fy=259.694;
  double m_px=160.7; // camera principal point
  double m_py=108.8;

  int m_deviceId=1; // camera id (in case of multiple cameras)
 
    //  cv::namedWindow(windowName, 1);
 

void processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    double t0;
      t0 = tic();
  
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
      double dt = tic()-t0;
      cout << "time taken " << dt << " seconds." << endl;
   

    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
    // for (int i=0; i<detections.size(); i++) {
    //   print_detection(detections[i]);
    // }

    // show the current image including any detections
    if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
      // imshow(windowName, image); // OpenCV call
    }
  }
  
// main code for image publishing starts below
int main(int argc, char** argv)
{
  
  m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
   cv::VideoCapture cap(1);
    if(!cap.isOpened()) return 1;
    cv::Mat image;
    cv::Mat image_gray;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(100);
    while (nh.ok()) {
    cap >> image;
    // Calculations done here
     processImage(image, image_gray);
    //Calculations end here
         msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
      ros::spinOnce();
    loop_rate.sleep();
  }
  
}
