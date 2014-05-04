#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cvaux.h>
#include <cxcore.hpp>
#include <sys/stat.h>
#include <termios.h>
#include <term.h>
#include <unistd.h>
using namespace std;
static const char *faceCascadeFilename = "haarcascade_frontalface_alt.xml";

CvHaarClassifierCascade* faceCascade;

void loadCascade(){
    // Load the HaarCascade classifier for face detection.
     faceCascade = (CvHaarClassifierCascade*)cvLoad(faceCascadeFilename, 0, 0, 0 );
     if( !faceCascade )
     {
       ROS_INFO("Could not load Haar cascade Face detection classifier in '%s'.", faceCascadeFilename);
       exit(1);
     }
}

IplImage* convertImageToGreyscale(const IplImage *imageSrc)
{
	IplImage *imageGrey;
	// Either convert the image to greyscale, or make a copy of the existing greyscale image.
	// This is to make sure that the user can always call cvReleaseImage() on the output, whether it was greyscale or not.
	if (imageSrc->nChannels == 3) {
		imageGrey = cvCreateImage( cvGetSize(imageSrc), IPL_DEPTH_8U, 1 );
		cvCvtColor( imageSrc, imageGrey, CV_BGR2GRAY );
	}
	else {
		imageGrey = cvCloneImage(imageSrc);
	}
	return imageGrey;
}

CvSeq* detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade )
{
	const CvSize minFeatureSize = cvSize(20, 20);
	const int flags = CV_HAAR_DO_CANNY_PRUNING;
	const float search_scale_factor = 1.1f;
	IplImage *detectImg;
	IplImage *greyImg = 0;
	CvMemStorage* storage;
	//double t;
	CvSeq* rects;


	storage = cvCreateMemStorage(0);
	cvClearMemStorage( storage );

	// If the image is color, use a greyscale copy of the image.
	detectImg = (IplImage*)inputImg;	// Assume the input image is to be used.
	if (inputImg->nChannels > 1)
	{
		greyImg = cvCreateImage(cvSize(inputImg->width, inputImg->height), IPL_DEPTH_8U, 1 );
		cvCvtColor( inputImg, greyImg, CV_BGR2GRAY );
		detectImg = greyImg;	// Use the greyscale version as the input.
	}

	// Detect all the faces.
	//$$$$$t = (double)cvGetTickCount();
	rects = cvHaarDetectObjects( detectImg, (CvHaarClassifierCascade*)cascade, storage,
				search_scale_factor, 3, flags, minFeatureSize );
	//$$$$$t = (double)cvGetTickCount() - t;
	//$$$$$ROS_INFO("[Face Detection took %d ms and found %d objects]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ), rects->total );

	//cvReleaseHaarClassifierCascade( &cascade );
	//cvReleaseImage( &detectImg );
	if (greyImg)
	cvReleaseImage( &greyImg );
	cvReleaseMemStorage( &storage );

	return rects;
}

void doSomething(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO("start dosomething");
    cv_bridge::CvImagePtr cv_ptr;
    //convert from ros image format to opencv image format
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ros::Rate r(4);

    IplImage img_input = cv_ptr->image;
    IplImage *img= cvCloneImage(&img_input);
    IplImage *greyImg;
    CvSeq* faceSeq;
    // 转成灰度图片
    greyImg = convertImageToGreyscale(img);

    // 检测人脸，得到人脸矩形
    faceSeq = detectFaceInImage(greyImg, faceCascade);

    if(faceSeq->total > 0){
        for(int i=0; i<faceSeq->total; i++){
            CvRect faceRect = *(CvRect*)cvGetSeqElem( faceSeq, i );
            cvRectangle(img, cvPoint(faceRect.x, faceRect.y), cvPoint(faceRect.x + faceRect.width-1, faceRect.y + faceRect.height-1), CV_RGB(0,255,0), 1, 8, 0);
        }
    }
    cvShowImage("Input", img);
    cvWaitKey(1);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "face_detector");
    ROS_INFO("ros init end");
    loadCascade();
    ROS_INFO("loadCascade");
    ros::NodeHandle nh_;
    cvNamedWindow("Input", CV_WINDOW_AUTOSIZE); 	// output screen
    ROS_INFO("Start......");
    ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_color", 1, &doSomething);
    //doSomething();
    ros::spin();
    return 0;
}
