/**
 * 
 * Quick and dirty way to record OpenNI Camera stream
 * 
 * References:
 * - http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * - http://docs.opencv.org/doc/tutorials/highgui/video-write/video-write.html
 * - http://answers.ros.org/question/10222/openni_camera-depth-image-opencv/
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

#define SENSOR_MIN_RANGE 0.5
#define SENSOR_MAX_RANGE 5

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "DEPTH";

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
			image_sub_ = it_.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb, this);
			if (m_display)
				cv::namedWindow(WINDOW);

			//TODO: file path location 
			//std::string fullpath = ros::package::getPath("openni_record") + filename;
			//g_outputFile = fullpath.c_str();
			
			//TODO: cant get this from input video since its not yet converted
			outputVideo.open(m_fileName, CV_FOURCC('P','I','M','1'), 30, cv::Size(640, 480), false);
		}
		~ImageConverter(){
			if (m_display)
				cv::destroyWindow(WINDOW);
		}
		
		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
			}catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			
			// since data from ros is actual distance (in meter), we need to do 
			// some processing to be able to display and record it properly
			cv::Mat scaledMat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
			
			for (int i = 0; i < cv_ptr->image.rows; i++){
				float* Di = cv_ptr->image.ptr<float>(i);
				char* Ii = scaledMat.ptr<char>(i);
				for(int j = 0; j < cv_ptr->image.cols; j++){
					Ii[j] = (char) (255*((Di[j] - SENSOR_MIN_RANGE)/(SENSOR_MAX_RANGE - SENSOR_MIN_RANGE)));
				}
			}
			
			// stuff
			if (outputVideo.isOpened()){
				outputVideo.write(scaledMat);
			} else {
				ROS_ERROR("ERROR: can't write to %s", m_fileName);
				return;
			}
			if (m_display){
				cv::imshow(WINDOW, scaledMat);
				cv::waitKey(3);
			}
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "openni_record_depth");
	
	bool show = false;
	
	time_t rawtime;
	struct tm* timeinfo;
	char filename[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(filename, 80, "./openni_depth_%Y-%m-%d_%H-%M.avi", timeinfo);
	
	ROS_INFO("depth recording will start in 5 seconds ...");
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
