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

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "RGB";
//std::string g_outputFile = ros::package::getPath("openni_record") + "/youbot_openni_rgb.avi";
static const char g_outputFile[] = "/home/ariandy/youbot_openni_rgb.avi";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	
	cv::VideoWriter outputVideo;
	
	public:
		ImageConverter():it_(nh_){
			image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
			cv::namedWindow(WINDOW);
			//TODO: cant get this from input video since its not yet converted
			outputVideo.open(g_outputFile, CV_FOURCC('P','I','M','1'), 30, cv::Size(640, 480), true);
		}
		~ImageConverter(){
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
				ROS_ERROR("ERROR: can't write to %s", g_outputFile);
				return;
			}
			
			
			//cv::imshow(WINDOW, cv_ptr->image);
			//cv::waitKey(3);
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "openni_record_rgb");
	ROS_INFO("RGB recording will start in 5 seconds ...");
	ros::Time::init();
	ros::Duration(4).sleep();
	ROS_INFO("... and ...");
	ros::Duration(1).sleep();
	ROS_INFO("now!");
	
	ImageConverter ic;
	ros::spin();
	
	return 0;
}
