#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <map>
#include <cmath>
#include <list>

#include <math.h>
#include <std_msgs/String.h>
#include <sstream>
#include "bitathome_hardware_control/VectorMove.h"

using namespace std;

enum SkeletonState{
		NONE,					// 没有检测到骨架
		STATIC,					// OpenNI Tracker 认为人没丢，但无法计算骨架位置
		TRACKING,				// 正在被跟踪
};




enum Posture{
        FOLLOW_ME,
        STOP,
        PUASE,
};

// 骨架类
class KinectSkeleton
{
		public:
				map<string, cv::Point3d> points3D;					// 三维坐标点
				map<string, cv::Point2d> points2D;					// 二维坐标点（图像上）
				int userID;											// 用户 ID
				SkeletonState state;								// 骨架当前状态
};


class Velocity{
public:
    double vx;
    double vy;
    double w;
};

// 骨架视觉操作类
class KinectSkeletonVision
{
		ros::NodeHandle nh_;						// 用于操控节点
		image_transport::ImageTransport it_;		// 用于订阅每张更新的图像
		image_transport::CameraSubscriber sub_;		// 订阅返回对象
		tf::TransformListener tf_listener_;			// 监听骨架 tf
		image_geometry::PinholeCameraModel cam_model_;			// 坐标转换
		list<KinectSkeleton> skeletons;				// 骨架
		list<KinectSkeleton> lastSkeletons;

		// 运动相关
        ros::ServiceClient client;
        bitathome_hardware_control::VectorMove srv;

		// OpenCV 相关

		public :
		// 构造函数
		KinectSkeletonVision():it_(nh_){
				client = nh_.serviceClient<bitathome_hardware_control::VectorMove>("/hc_cmd_interface/vector_move");

				// 订阅图片主题
				string image_topic = nh_.resolveName("/camera/rgb/image_color");
				sub_ = it_.subscribeCamera(image_topic, 1, &KinectSkeletonVision::imageCb, this);
		}


        void move(cv::Point3d position){
            Velocity speed;
            speed = this -> get_move_action(position.x, position.y, position.z);
            srv.request.vx = speed.vx;
            srv.request.vy = speed.vy;
            srv.request.omega = speed.w;
            cout << speed.vx << " " << speed.vy << " " << speed.w << endl;
            client.call(srv);
        }

        Velocity get_move_action(double x, double y, double z)
        {
            double s = sqrt(x*x + z*z);
            double V = 600*(s-1);
            Velocity v;
            v.vx = z/s*V;
            v.vy = x/s*V;
            v.w = 800*x/s;
            return v;
        }

        /*
        Posture skeleton_check(KinectSkeleton s)
        {
            Posture pose;
            if (s.points3D["left_hand_"].y > s.points3D["left_shoulder"])
            {
                return pose.FOLLOW_ME
            }
            else if(s.points3D["right_hand_"].y > s.points3D["right_shoulder"])
            {
                return pose.STOP
            }
            else if()
        }
        */

		// 每收到一张图像
		void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
				// 图片转换为 OpenCV 格式
				cv_bridge::CvImagePtr cv_ptr;
				try
				{
						cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
				}
				catch (cv_bridge::Exception& e)
				{
						ROS_ERROR("cv_bridge exception: %s", e.what());
						return;
				}

				// 获取骨架
				cam_model_.fromCameraInfo(info_msg);				// 初始化转换器
				skeletons.clear();									// 清空骨架存储

				// 在此仅获取头部信息
				tf::StampedTransform transform;
				ros::Time acquisition_time = info_msg -> header.stamp;
				ros::Duration timeout(1.0 / 10);

				// 获取当前所有的帧
				vector<string> frame_ids;
				tf_listener_.getFrameStrings(frame_ids);

				// 获取user_id，以及所有的骨架
				BOOST_FOREACH(const string& frame_id, frame_ids) {
						if(!checkTF(frame_id))
								continue;
						vector<int> userIDs;
						int userID;
						char pos[100] = {0};
						sscanf(frame_id.c_str(), "%[^0123456789]%d", pos, &userID);		// 获取身体位置和用户 ID

						// 获取 tf
						tf_listener_.lookupTransform("openni_depth_frame", frame_id,
										ros::Time(0), transform);
						// 输出当前时间，以及 transform 的时间，以及差值
						double s1 = transform.stamp_.toSec();
						double s2 = ros::Time::now().toSec();
						// cout << s1 << " " << s2 << " " << s2 - s1 << endl;
						if (s2 - s1 > 0.1){
								cout << "pass : " << frame_id << " " << s2 - s1 << endl;
								continue;
						}
						// 检测时间，如果长久不更新，代表骨架已经丢失
						tf::Point pt = transform.getOrigin();
						cv::Point3d pt_cv_new(-pt.y(), -pt.z(), pt.x());
						cv::Point2d uv;
						uv = cam_model_.project3dToPixel(pt_cv_new);
						cv::Point3d pt_cv(pt.y(), pt.z(), pt.x());
                        // cv::Point3d pt_cv(pt.z(), -pt.x(), -pt.y());
						// 存储
						list<KinectSkeleton>::iterator v;
						for(v = skeletons.begin(); v != skeletons.end(); ++v){
								KinectSkeleton s = *v;
								if(s.userID == userID){
										(*v).points3D[pos] = pt_cv;
										(*v).points2D[pos] = uv;
										break;
								}
						}
						// 该 UserID 之前没有存储过
						if (v == skeletons.end()){
								KinectSkeleton skeleton;
								skeleton.userID = userID;
								skeleton.points3D[pos] = pt_cv;
								skeleton.points2D[pos] = uv;
								skeletons.push_back(skeleton);
						}
				}

				// 检测是否存在静态骨架
				for(list<KinectSkeleton>::iterator v = skeletons.begin(); v != skeletons.end(); ++v){
						KinectSkeleton s = *v;
						if(checkIsStatic(s, lastSkeletons)){
								(*v).state = STATIC;
						}
						else{
								(*v).state = TRACKING;
						}
				}

				// 注意，此处得到的值为 z, x, y
				if (!skeletons.empty()){
						// cout << "检测到骨架" << endl;
						list<KinectSkeleton>::iterator v;
						for(v = skeletons.begin(); v != skeletons.end(); ++v){
								KinectSkeleton s = *v;
								TaskState state = TaskState.NONE
								if(s.state == STATIC){
										// cout << "骨架" << s.userID << " 静止" << endl;
								}
								else{

										// cout << "骨架" << s.userID << " : ";
										cv::Point3d points = s.points3D["torso_"];
										// cout << "(" << points.x << ", " << points.y << ", " << points.z << ")" << endl;
										// cout << s.points2D["torso_"].x << " " << s.points2D["torso_"].y << endl;
										this -> move(points);
										// cout << "位置 : " << points << endl;
										cv::circle(cv_ptr -> image, s.points2D["torso_"], 3, CV_RGB(255,0,0), -1);
								}
						}
				}
				else{
						cout << "无骨架" << endl;
				}

				// 存储骨架，用于检测骨架是否静态丢失
				lastSkeletons = skeletons;
				cv::imshow("image", cv_ptr->image);
				cv::waitKey(3);
		}

		// 检测骨架是否处于静态状态
		bool checkIsStatic(KinectSkeleton skeleton, list<KinectSkeleton> lastSkeletons){
				list<KinectSkeleton>::iterator v;
				for(v = lastSkeletons.begin(); v != lastSkeletons.end(); ++v){
						KinectSkeleton s = *v;
						cv::Point3d point = s.points3D["torso_"];
						cv::Point3d lastPoint = skeleton.points3D["torso_"];
						if(fabs(point.x - lastPoint.x) <= 0.000001 && fabs(point.y - lastPoint.y) <= 0.0000001 && fabs(point.z - lastPoint.z) <= 0.0000001){
								return true;
						}
				}
				return false;
		}

		// 输出 map
		void printMap(map<string, cv::Point3d> mymap){
				map<string, cv::Point3d>::const_iterator itr;
				for(itr = mymap.begin(); itr != mymap.end(); ++itr){
						cout << "Key: " << (*itr).first << " Value: " << (*itr).second;
				}
		}

		// 判断是否是骨架 TF
		bool checkTF(string str){
				string body[] = {"head", "neck", "torso", "left_shoulder", "left_elbow", "left_hand", "right_shoulder", "right_elbow", "right_hand", "left_hip", "left_knee", "left_foot", "right_hip", "right_knee", "right_foot"};
				for(int k = 0; k < 15; ++k){
						if (str.find(body[k]) != string::npos)
								return true;
				}
				return false;
		}
};



int main(int argc, char **argv){
		ros::init(argc, argv, "draw_frames");
		KinectSkeletonVision kv;
		ros::spin();
		return 0;
}
