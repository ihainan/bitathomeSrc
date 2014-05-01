#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <boost/foreach.hpp>
#include <map>
#include <list>

#include <math.h>
#include <std_msgs/String.h>
#include <sstream>

using namespace std;

// 骨架类
class KinectSkeleton
{
	public:
		map<string, cv::Point3d> points3D;					// 三维坐标点
		map<string, cv::Point2d> points2D;					// 二维坐标点（图像上）
		int userID;											// 用户 ID
};


struct Velocity{
    double vx;
    double vy;
    double w;
};

Velocity speed;

Velocity get_move_action(int x,int y,int z)
{
    double s = sqrt(x*x + z*z);
	double V = 500*(s-1);
	Velocity v;
	v.vx = z/s*V;
	v.vy = x/s*V;
	v.w = 600*x/s;
	return v;
}

// 骨架视觉操作类
class KinectSkeletonVision
{
		ros::NodeHandle nh_;						// 用于操控节点
		image_transport::ImageTransport it_;		// 用于订阅每张更新的图像
		image_transport::CameraSubscriber sub_;		// 订阅返回对象
		tf::TransformListener tf_listener_;			// 监听骨架 tf
		image_geometry::PinholeCameraModel cam_model_;			// 坐标转换
		// KinectSkeleton skeletons[20];				// 骨架
		list<KinectSkeleton> skeletons;

	public :
		// 构造函数
		KinectSkeletonVision():it_(nh_){
				// 订阅图片主题
				string image_topic = nh_.resolveName("/camera/rgb/image_raw");
				sub_ = it_.subscribeCamera(image_topic, 1, &KinectSkeletonVision::imageCb, this);
		}


		// 每收到一张图像
		void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
				cam_model_.fromCameraInfo(info_msg);				// 初始化转换器
				skeletons.clear();									// 清空骨架存储

				// 在此仅获取头部信息
				// printf("收到图片.\n");
				tf::StampedTransform transform;
				ros::Time acquisition_time = info_msg -> header.stamp;
				ros::Duration timeout(1.0 / 10);

				// 获取当前 fixed_frame 下的所有 child_frames
				vector<string> frame_ids;
				tf_listener_.waitForTransform("openni_depth_frame", "openni_depth_frame",
					acquisition_time, timeout);
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
						tf::Point pt = transform.getOrigin();
						cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
						cv::Point2d uv;
						uv = cam_model_.project3dToPixel(pt_cv);

						// 存储
						list<KinectSkeleton>::iterator v;
						//cout << pos << endl;
						for(v = skeletons.begin(); v != skeletons.end(); ++v){
								KinectSkeleton s = *v;
								if(s.userID == userID){
										s.points3D[pos] = pt_cv;
										s.points2D[pos] = uv;
										// cout << "旧：" << userID << " " << pos << endl;
								}
						}
						if (v == skeletons.end()){
								KinectSkeleton skeleton;
								skeleton.userID = userID;
								skeleton.points3D[pos] = pt_cv;
								skeleton.points2D[pos] = uv;
								skeletons.push_back(skeleton);
								// cout << "新：" << userID << " " << pos << endl;
						}
				}

				// skeletons
				if(skeletons.empty() == false)
				{
                    KinectSkeleton skeleton = skeletons.front();
                    cv::Point3d position = skeleton.points3D["head_"];
                    speed = get_move_action(position.x,position.y,position.z);
                    cout << skeleton.userID <<" : " << position.x << " " << position.y << " " <<position.z << endl;
                    cout << speed.vx << " "<< speed.vy << " " << speed.w <<endl;
				}
				else
				{
                    cout << "没有骨架"<<endl;
				}
				skeletons.clear();

		}

		bool checkTF(string str){
				string body[] = {"head", "neck", "torso", "left_shoulder", "left_elbow", "left_hand", "right_shoulder", "right_elbow", "right_hand", "left_hip", "left_knee", "left_foot", "right_hip", "right_knee", "right_foot"};
				for(int k = 0; k < 15; ++k){
						if (str.find(body[k]) != string::npos)
								return true;
				}
				return false;
		}
};


void set_speed_loop()
{

}

int main(int argc, char **argv){
		ros::init(argc, argv, "draw_frames");
		KinectSkeletonVision kv;
        set_speed_loop();
		ros::spin();
		return 0;
}
