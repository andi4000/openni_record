/**
 * 
 * Quick and dirty way to record OpenNI Camera stream
 * 
 * References:
 * - http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * - http://docs.opencv.org/doc/tutorials/highgui/video-write/video-write.html
 * 
 * 
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ctime>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "RGB";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	
	cv::VideoWriter outputVideo;
	
	char* m_fileName;
	bool m_display;
	
	public:
		ImageConverter(char* fileName, bool display):it_(nh_), m_fileName(fileName), m_display(display){
			image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
			if (m_display)
				cv::namedWindow(WINDOW);
			//TODO: cant get this from input video since its not yet converted
			outputVideo.open(m_fileName, CV_FOURCC('P','I','M','1'), 30, cv::Size(640, 480), true);
		}
		~ImageConverter(){
		if (m_display)
			cv::destroyWindow(WINDOW);
		}
		
		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			}catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			
			// stuff
			if (outputVideo.isOpened()){
				outputVideo.write(cv_ptr->image);
			} else {
				ROS_ERROR("ERROR: can't write to %s", m_fileName);
				return;
			}
			
			if (m_display){
				cv::imshow(WINDOW, cv_ptr->image);
				cv::waitKey(3);
			}
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "openni_record_rgb");
	
	bool show = false;
	
	time_t rawtime;
	struct tm* timeinfo;
	char filename[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(filename, 80, "./openni_rgb_%Y-%m-%d_%H-%M.avi", timeinfo);
	
	ROS_INFO("RGB recording will start in 5 seconds ...");
	ros::Time::init();
	ros::Duration(4).sleep();
	ROS_INFO("... and ...");
	ros::Duration(1).sleep();
	ROS_INFO("now!");
	ROS_WARN("Recording to: %s", filename);
	
	ImageConverter ic(filename, show);
	ros::spin();
	
	return 0;
}
