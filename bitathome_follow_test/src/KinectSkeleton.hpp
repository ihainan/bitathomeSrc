#ifndef _KINECT_SKELETON_HPP_
#define _KINECT_SKELETON_HPP_
#include <map>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

// 被跟踪者状态
enum LockedUserState{
	UNLOCKED,			// 尚未锁定
	FOLLOWING,			// 正在跟踪
	LOST,				// 已经丢失
	PAUSE,				// 暂停状态
	REC,				// 识别状态
};

// 骨架状态枚举
enum SkeletonState{
	NONE,					// 没有检测到骨架
	STATIC,					// OpenNI Tracker 认为人没丢，但无法计算骨架位置
	TRACKING,				// 正在被跟踪
};

// 人体姿态
enum SkeletonGesture{
	NOGESTURE,				// 默认，无
	RAISELEFTHAND,			// 举起左手
	RAISERIGHTHAND,         // 举起右手
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


// 速度类
class Velocity{
public:
    double vx;
    double vy;
    double w;
    Velocity(){
        this->vx = this->vy = this->w = 0.0f;
    }
};
#endif
