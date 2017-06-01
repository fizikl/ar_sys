/**
* @file single_board.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief ROS version of the example named "simple_board" in the Aruco software package.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/linklist.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_sys/utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <string>
#include <std_msgs/Int8.h>

using namespace aruco;

class ArSysSingleBoard
{
	private:
		cv::Mat inImage, resultImg;
		aruco::CameraParameters camParam;
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
        bool publish_tf;
		MarkerDetector mDetector;
		vector<Marker> markers;
		BoardConfiguration the_board_config;
		BoardDetector the_board_detector;
		Board the_board_detected;
		ros::Subscriber cam_info_sub;
		ros::Subscriber cam_trigger_sub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		image_transport::Publisher debug_pub;
		ros::Publisher pose_pub;
		ros::Publisher transform_pub; 
		ros::Publisher position_pub;
		ros::Publisher center_pos;
		std::string board_frame;

		double marker_size;
		std::string board_config;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		tf::TransformListener _tfListener;

	public:
		ArSysSingleBoard()
			: cam_info_received(false), 
			nh("~"), 
			it(nh)
		{
			image_sub = it.subscribe("/image", 1, &ArSysSingleBoard::image_callback, this);
			cam_info_sub = nh.subscribe("/camera_info", 1, &ArSysSingleBoard::cam_info_callback, this);
			cam_trigger_sub = nh.subscribe("/cam_triger", 10, &ArSysSingleBoard::cam_trigger_callback, this);

			image_pub = it.advertise("result", 1);
			debug_pub = it.advertise("debug", 1);
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
			position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
			center_pos = nh.advertise<geometry_msgs::Point>("offset", 100);

			nh.param<double>("marker_size", marker_size, 0.05);
			nh.param<std::string>("board_config", board_config, "boardConfiguration.yml");
			nh.param<std::string>("board_frame", board_frame, "");
			nh.param<bool>("image_is_rectified", useRectifiedImages, true);
			nh.param<bool>("draw_markers", draw_markers, true);
			nh.param<bool>("draw_markers_cube", draw_markers_cube, true);
			nh.param<bool>("draw_markers_axis", draw_markers_axis, true);
            nh.param<bool>("publish_tf", publish_tf, true);

			the_board_config.readFromFile(board_config.c_str());

			ROS_INFO("ArSys node started with marker size of %f m and board configuration: %s",marker_size, board_config.c_str());
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
            static tf::TransformBroadcaster br;
            
			if(!cam_info_received) return;

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;
				resultImg = cv_ptr->image.clone();

				//detection results will go into "markers"
				markers.clear();
				//Detecting markers,
				mDetector.detect(inImage, markers, camParam, marker_size, false);
				//Detection of the board
				float probDetect=the_board_detector.detect(markers, the_board_config, the_board_detected, camParam, marker_size);
				if (probDetect > 0.0)
				{
					tf::Transform transform = ar_sys::getTf(the_board_detected.Rvec, the_board_detected.Tvec);

					tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, board_frame);
                    
                    if (publish_tf) 
                        br.sendTransform(stampedTransform);

					geometry_msgs::PoseStamped poseMsg;
					tf::poseTFToMsg(transform, poseMsg.pose);
					poseMsg.header.frame_id = msg->header.frame_id;
					poseMsg.header.stamp = msg->header.stamp;
					pose_pub.publish(poseMsg);

					geometry_msgs::TransformStamped transformMsg;
					tf::transformStampedTFToMsg(stampedTransform, transformMsg);
					transform_pub.publish(transformMsg);

					geometry_msgs::Vector3Stamped positionMsg;
					positionMsg.header = transformMsg.header;
					positionMsg.vector = transformMsg.transform.translation;
					position_pub.publish(positionMsg);

				}
				//for each marker, draw info and its boundaries in the image
				//cout << "number of markers " << markers.size() << "\n";
				// put text:
				cv::Point pt = cv::Point(20,20);
				putText(resultImg, "Fizikl Aruco Test", pt, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,150,150), 2);
				if (markers.size() != 0)
				{
					Marker mrkarry [4] = {markers[0], markers[0], markers[0], markers[0]};
					//cout << mrkarry[0][0] << "hahahah" << "\n";
				
					for(size_t i=0; i < markers.size(); i++)
					{
						// void Marker::draw(Mat &in, Scalar color, int lineWidth ,bool writeId)const
						// void MarkerDetector::draw ( Mat out,const vector<Marker> &markers )
						//markers[i].draw(resultImg,cv::Scalar(0,0,255),3, true);
						//cout << "I drew something! heyhey" << i << "\n";
						//cout << "so this is the left upper corner" << markers[i][0].x << "\n";

						// getting distance from marker to center
						//cout << "0.0\n";
						//cout << getDistance(markers[i]) << "  " << i << markers[i].id << "\n";
						//cout << mrkarry[0][0] << "\n";
						//cout << getDistance(mrkarry[0]) << "  " << mrkarry[0].id << "\n";
						//cout << getDistance(mrkarry[1]) << "  " << mrkarry[1].id << "\n";
						//cout << getDistance(mrkarry[2]) << "  " << mrkarry[2].id << "\n";
						//cout << getDistance(mrkarry[3]) << "  " << mrkarry[3].id << "\n";
						
						for (size_t j = 0; j < 4; j++) {
							if (j == 0 && getDistance(markers[i]) < getDistance(mrkarry[0])) {
						//		cout << "0\n";
								mrkarry[0] = markers[i];
						//		cout << "1\n";
							}
							if (j == 1 && getDistance(mrkarry[0]) < getDistance(mrkarry[1])) {
								Marker temp;
								temp = mrkarry[0];
								mrkarry[0] = mrkarry[1];
								mrkarry[1] = temp;
						//		cout << "2\n";
							}
							if (j == 2 && getDistance(mrkarry[1]) < getDistance(mrkarry[2])) {
								Marker temp;
								temp = mrkarry[1];
								mrkarry[1] = mrkarry[2];
								mrkarry[2] = temp;
						//		cout << "3\n";
							}
							if (j == 3 && getDistance(mrkarry[2]) < getDistance(mrkarry[3])) {
								Marker temp;
								temp = mrkarry[2];
								mrkarry[2] = mrkarry[3];
								mrkarry[3] = temp;
						//		cout << "4\n";
							}
						}
						if (markers[i].id == 111) {
							//cv::Point c0 = getCenter(markers[i]);
							cv::Point c0 = markers[i].getCenter();
							cout << "Single board, center is at: " << c0 << "\n";
							//cv::Point c0 = cv::Point(14.5,46.67);
							float center [2] = {c0.x, c0.y};
							geometry_msgs::Point offsetMsg;
							offsetMsg.x = center[0] - 100;
							offsetMsg.y = center[1] - 100;
							offsetMsg.z = getSize(markers[i]);
							center_pos.publish(offsetMsg);
							// draw a line to the center
							cv::Point origin = cv::Point(100,100);
							cv::line( resultImg, origin, c0,cv::Scalar(255,255,0),4,CV_AA);
						} 
						/*
						if (markers[i].id == 101) {
							mrkarry[0] = markers[i];
						} 
						if (markers[i].id == 111) {
							mrkarry[1] = markers[i];
						} 
						if (markers[i].id == 121) {
							mrkarry[2] = markers[i];
						} 
						if (markers[i].id == 131) {
							mrkarry[3] = markers[i];
						} */ 
					}
					// draw box arround 4 aruco found,
					for(int i = 0; i < 4; i++) {
						mrkarry[i].draw(resultImg,cv::Scalar(0,0,255),3, true);
						CvDrawingUtils::draw3dCube(resultImg, mrkarry[i], camParam);
						//CvDrawingUtils::draw3dAxis(resultImg, mrkarry[i], camParam);
					}
					// Draw boundary box:
					cv::Point c1 = getCenter(mrkarry[0]);
					cv::Point c2 = getCenter(mrkarry[1]);
					cv::Point c3 = getCenter(mrkarry[2]);
					cv::Point c4 = getCenter(mrkarry[3]);
					// Put arucos in order:
					Marker topLeft = mrkarry[0];
					Marker bottomRight = mrkarry[1];
					for(size_t i = 0; i < 4; i++) {
						int sumXY_TL = getCenter(topLeft).x + getCenter(topLeft).y;
						int sumXY_BR = getCenter(bottomRight).x + getCenter(bottomRight).y;
						int sumXY = getCenter(mrkarry[i]).x + getCenter(mrkarry[i]).x;
						if(sumXY < sumXY_TL){
							topLeft = mrkarry[i];
						}
						if(sumXY > sumXY_BR){
							bottomRight = mrkarry[i];
						}
					}
					// define four points:, Clockwise
					cv::Point p1 = getCenter(topLeft);
					cv::Point p2 = cv::Point(getCenter(bottomRight).x, getCenter(topLeft).y);
					cv::Point p3 = getCenter(bottomRight);
					cv::Point p4 = cv::Point(getCenter(topLeft).x, getCenter(bottomRight).y);
					cv::line( resultImg,p1, p2,cv::Scalar(0,255,0),4,CV_AA);
					cv::line( resultImg,p2, p3,cv::Scalar(0,255,0),4,CV_AA);
					cv::line( resultImg,p3, p4,cv::Scalar(0,255,0),4,CV_AA);
					cv::line( resultImg,p4, p1,cv::Scalar(0,255,0),4,CV_AA);

					// Define center point:
					cv::Point centerPoint = cv::Point(c1.x+(c3.x-c1.x)/2, c1.y+(c3.y-c1.y)/2);
					//cout << "the center is at " << centerPoint.x << " and " << centerPoint.y << "\n";
					// How to get Velocity?? How to access previous state?? speedometer node?
					// Puting text:
					// info
					cv::Point pt2 = cv::Point(20,40);
					putText(resultImg, "Aruco INFO:", pt2, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100,100,255,255), 2);
					// aruco1
					cv::Point pt3 = cv::Point(20,60);
					putText(resultImg, "Aruco 1:", p3, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100,100,255,255), 2);
					// aruco2
					cv::Point pt4 = cv::Point(20,80);
					putText(resultImg, "Aruco 2:", pt4, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100,100,255,255), 2);
					// aruco3
					cv::Point pt5 = cv::Point(20,100);
					putText(resultImg, "Aruco 3:", pt5, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100,100,255,255), 2);
					// aruco4
					cv::Point pt6 = cv::Point(20,120);
					putText(resultImg, "Aruco 4:", pt6, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100,100,255,255), 2);
					// center
					cv::Point pt7 = cv::Point(20,140);
					putText(resultImg, "Center Point:", pt7, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100,100,255,255), 2);
					// velocity x,y
					cv::Point pt8 = cv::Point(20,160);
					putText(resultImg, "Center velocity:", pt8, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(100,100,255,255), 2);
				}
				
				// draw lines bounding box of 4 aruco closest to the center
				// added by shirman
				for(size_t i=1; i < markers.size(); i++)
				{
					//cv::line( resultImg,markers[i][0],markers[i-1][0],cv::Scalar(0,255,255),4,CV_AA);
				}


				if(camParam.isValid() && marker_size != -1)
				{
					//draw a 3d cube in each marker if there is 3d info
					for(size_t i=0; i<markers.size(); i++)
					{
						//if (draw_markers_cube) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
						//if (draw_markers_axis) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
						//CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
						//CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
					}
					//draw board axis
					if (probDetect > 0.0) CvDrawingUtils::draw3dAxis(resultImg, the_board_detected, camParam);
				}

				if(image_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}

				if(debug_pub.getNumSubscribers() > 0)
				{
					//show also the internal image resulting from the threshold operation
					cv_bridge::CvImage debug_msg;
					debug_msg.header.frame_id = msg->header.frame_id;
					debug_msg.header.stamp = msg->header.stamp;
					debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
					debug_msg.image = mDetector.getThresholdedImage();
					debug_pub.publish(debug_msg.toImageMsg());
				}
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg)
		{
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
			cam_info_sub.shutdown();
		}

		// subscribe to camera trigger from micro-guidance
		// save image frames using trigger
		int i = 0; 
		void cam_trigger_callback(const std_msgs::Int8 &msg) 
		{
			if (msg.data) {

				imwrite( cv::format("/home/alzuhair/Gray_Image%d.jpg", i), inImage); // alzuhair here should be chagned to your honmme directory
				i++;
			}
		}
		// input: A marker,
		// output: the distance between the centor of the marker to centor of the image
		// Added by shirman
		float getDistance(const Marker &M)
		{
			// get center pont of marker
			float Cx = (M[0].x + M[1].x) / 2;
			float Cy = (M[0].y + M[3].y) / 2;
			//cout << "The marker is at " << Cx << "and" << Cy << "\n";
			float distance = sqrt((Cx - 320)*(Cx - 320) + (Cy - 240)*(Cy - 240));
			//cout << "The marker" << M.id << "is at " << Cx << "and" << Cy << "distance is " << distance << "\n";
			return distance;
		}

		cv::Point getCenter(const Marker &M)
		{
			cv::Point center;
			//cout << M[0] << "\n";
			center.x = (M[0].x + M[1].x + M[2].x + M[3].x) / 4;
			center.y = (M[0].y + M[1].y + M[2].y + M[3].y) / 4;
			return center;
		}

		float getSize(const Marker &M)
		{
			float x = M[0].x - M[2].x;
			float y = M[0].y - M[2].y;
			return x*y;
		}
};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "ar_single_board");

	ArSysSingleBoard node;

	ros::spin();
}
