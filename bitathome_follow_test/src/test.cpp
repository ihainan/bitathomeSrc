#include "KinectSkeleton.hpp"
#include "KinectVision.hpp"
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


// 骨架视觉操作类
class KinectSkeletonVision
{
		ros::NodeHandle nh_;						// 用于操控节点
		image_transport::ImageTransport it_;		// 用于订阅每张更新的图像
		image_transport::CameraSubscriber sub_;		// 订阅返回对象
		tf::TransformListener tf_listener_;			// 监听骨架 tf
		image_geometry::PinholeCameraModel cam_model_;			// 坐标转换

		list<KinectSkeleton> skeletons;				// 骨架
		list<KinectSkeleton> lastSkeletons;			// 上一次存储的骨架

		int lockUserID;								// 锁定用户 ID
		KinectVision vision;						// 视觉图像处理对象
		LockedUserState lockedUserState;			// 跟随用户的状态
		char bodyStr[16][30] = {"head_", "neck_" "torso_", "left_shoulder_", "left_elbow_", "left_hand_", "right_shoulder_", "right_elbow_", "right_hand_", "left_hip_", "left_knee_", "left_foot_", "right_hip_", "right_knee_", "right_foot_"};

        // 运动相关
        ros::ServiceClient client;
        bitathome_hardware_control::VectorMove srv;
        double currentTime;

		public :
		// 构造函数
		KinectSkeletonVision():it_(nh_){
				// 初始化状态
				lockedUserState = UNLOCKED;			// 最开始，尚未锁定
				lockUserID = -1;
                //初始化监听
                client = nh_.serviceClient<bitathome_hardware_control::VectorMove>("/hc_cmd_interface/vector_move");

				// 订阅图片主题
				string image_topic = nh_.resolveName("/camera/rgb/image_color");
				sub_ = it_.subscribeCamera(image_topic, 1, &KinectSkeletonVision::imageCb, this);
		}

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

						// 检测时间，如果长久不更新，代表骨架已经丢失
						double s1 = transform.stamp_.toSec();
						double s2 = ros::Time::now().toSec();
						if (s2 - s1 > 0.1){
								continue;
						}

						// 坐标转换
						tf::Point pt = transform.getOrigin();
						cv::Point3d pt_cv_t(-pt.y(), -pt.z(), pt.x());
						cv::Point2d uv;
						uv = cam_model_.project3dToPixel(pt_cv_t);
						cv::Point3d pt_cv(pt.y(), pt.z(), pt.x());

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

				// 做一些事情
				doSomething(skeletons, cv_ptr -> image);

				// 存储骨架，用于检测骨架是否静态丢失
				lastSkeletons = skeletons;
		}

		// 获取到骨架和图片之后，做一些事情
		void doSomething(list<KinectSkeleton> skeletons, cv::Mat image){
				// 监控拿到的骨架，进行状态转换
				Velocity speed;
				if (!skeletons.empty()){
						list<KinectSkeleton>::iterator v;
						// 遍历骨架
						for(v = skeletons.begin(); v != skeletons.end(); ++v){
								KinectSkeleton s = *v;
								if(s.state == STATIC){
										// cout << "骨架" << s.userID << " 静止" << endl;
										// 骨架丢失
										if(lockUserID == s.userID){
												this -> lockedUserState = LOST;
												cout << "跟踪丢失" << endl;
										}
								}
								else{
										// 在图像中显示骨架
										cv::circle(image, s.points2D["torso_"], 3, CV_RGB(255,0,0), -1);
										cv::circle(image, s.points2D["left_shoulder_"], 3, CV_RGB(255,0,0), -1);
										cv::circle(image, s.points2D["right_shoulder_"], 3, CV_RGB(255,0,0), -1);

										// 如果之前骨架丢失，判断当前所检测到的骨架是不是原来的锁定者
										if(this -> lockedUserState == LOST){
														cv::Mat currentUserImage = vision.getClipedImage(image, s);
														if(currentUserImage.rows > 0){

																double confidence = vision.getDifference(currentUserImage);
																cout << "相似度：" << confidence << endl;
																if(confidence >= 90){
																		this -> lockedUserState = FOLLOWING;
																		this -> lockUserID = s.userID;
																		cout << "找回目标" << endl;
																}
														}
										}


										// 如果当前状态是跟随，并且当前骨架是被跟随着，则运动
										if(this -> lockedUserState == FOLLOWING && lockUserID == s.userID){
												// 更新图像
												cv::Mat currentUserImage = vision.getClipedImage(image, s);

												// 检查是否到时间，是的话更新图片
												double nowTime = ros::Time::now().toSec();
												if(nowTime - currentTime > 3){
                                                        // 获取图片，检测图片是否合格
                                                        if(currentUserImage.rows > 0){
                                                                double confidence = vision.getDifference(currentUserImage);
                                                                if(confidence >= 90){
																		vision.newKinectVision(image, s);
																		currentTime = ros::Time::now().toSec();
																		cout << "更新图片" << endl;
																}
                                                        }
												}

												// 如果收到了暂停的手势，则暂停，计时
												// 如果当前收到了识别的手势，则暂停，等待视野中出现两个人
												// 如果收到了停止的手饰，则停止
												if(checkSkeletonGesture(s) == RAISERIGHTHAND)
												{
                                                    this -> lockUserID = -1;
                                                    this -> lockedUserState = UNLOCKED;
												}
												// 设置速度，正常运动
												else
												{
                                                    cv::Point3d points = s.points3D["torso_"];
                                                    speed = this -> GetMoveAction(points.x, points.y, points.z);
												}
										}

										// 如果当前状态是暂停，则计算时间是否到达
										if(this -> lockedUserState == PAUSE && lockUserID == s.userID){
										}


										// 如果当前状态是未锁定，且有人举手，则锁定，存储图像特征
										if(this -> lockedUserState == UNLOCKED  && checkSkeletonGesture(s) == RAISELEFTHAND){
												lockUserID = s.userID;
												this -> lockedUserState = FOLLOWING;
												vision.newKinectVision(image, s);
												// 计时
												currentTime =ros::Time::now().toSec();
										}
								}
						}
				}
				else{
						// 骨架丢失
						if(lockUserID != -1){
								this -> lockedUserState = LOST;
								cout << "跟踪丢失" << endl;
						}
				}

                // 控制运动
                srv.request.vx = speed.vx;
                srv.request.vy = speed.vy;
                srv.request.omega = speed.w;
                client.call(srv);
                //cout << speed.vx << " " << speed.vy << " " << speed.w << endl;

				// 显示当前所看到的图片
				cv::imshow("image", image);
				cv::waitKey(3);
		}

        //计算运动速度函数
        Velocity GetMoveAction(double x, double y, double z)
        {
            double s = sqrt(x*x + z*z);
            double V = 600*(s-1);
            Velocity v;
            v.vx = z/s*V;
            v.vy = x/s*V;
            v.w = 800*x/s;
            return v;
        }

		// 检测姿态
		SkeletonGesture checkSkeletonGesture(KinectSkeleton skeleton){
				map<string, cv::Point2d> points2D = skeleton.points2D;					// 二维坐标点（图像上）
				if(-points2D["left_hand_"].y > -points2D["torso_"].y){
						return RAISELEFTHAND;
				}
				if(-points2D["right_hand_"].y > -points2D["torso_"].y){
                        return RAISERIGHTHAND;
				}
				return NOGESTURE;
		}

		// 检测骨架是否处于静态状态
		bool checkIsStatic(KinectSkeleton skeleton, list<KinectSkeleton> lastSkeletons){
				list<KinectSkeleton>::iterator v;
				for(v = lastSkeletons.begin(); v != lastSkeletons.end(); ++v){
						int sum = 0;
						KinectSkeleton s = *v;
						for(int i = 0; i < 15; ++i){
								cv::Point3d point = s.points3D[bodyStr[i]];
								cv::Point3d lastPoint = skeleton.points3D[bodyStr[i]];
								if(fabs(point.x - lastPoint.x) <= 0.000001 && fabs(point.y - lastPoint.y) <= 0.0000001 && fabs(point.z - lastPoint.z) <= 0.0000001){
										sum++;
								}
						}
						if(sum == 15){
								return true;
						}
				}
				return false;
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

		// 输出 map
		void printMap(map<string, cv::Point3d> mymap){
				map<string, cv::Point3d>::const_iterator itr;
				for(itr = mymap.begin(); itr != mymap.end(); ++itr){
						cout << "Key: " << (*itr).first << " Value: " << (*itr).second;
				}
		}

};

int main(int argc, char **argv){
		ros::init(argc, argv, "draw_frames");
		KinectSkeletonVision kv;
		ros::spin();
		return 0;
}
