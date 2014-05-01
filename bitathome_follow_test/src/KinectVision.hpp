#ifndef _KINECT_VISION_H_
#define _KINECT_VISION_H_

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "KinectSkeleton.hpp"

using namespace std;

// 图像处理类
class KinectVision
{
		public:
				cv::Mat img;
				double fingerPrint[4][8][8][8];

				KinectVision(){
				}

				KinectVision(cv::Mat image, KinectSkeleton skeleton){
						img = getClipedImage(image, skeleton);
						getFingerPrint(img, fingerPrint);
				}

				void newKinectVision(cv::Mat image, KinectSkeleton skeleton){
						// cout << "newKinectVision : " << image.rows << endl;
						this ->img = getClipedImage(image, skeleton);
						// cout << "newKinectVision - img: " << this -> img.rows << endl;
						getFingerPrint(img, fingerPrint);
				}

				// 获取图像
				cv::Mat getImage(){
						return img;
				}

				// 获取指定用户的图像
				static cv::Mat getClipedImage(cv::Mat image, KinectSkeleton skeleton){
						cv::Mat cImage;
						// 顶点坐标
						cv::Point2d leftShoulderColorPoint = skeleton.points2D["left_shoulder_"];
						cv::Point2d rightShoulderPoint = skeleton.points2D["right_shoulder_"];
						cv::Point2d leftKneeColorPoint = skeleton.points2D["left_knee_"];
						cv::Point2d headPoint = skeleton.points2D["head_"];
						int x = leftShoulderColorPoint.x;
						int y = leftShoulderColorPoint.y;
						int width = rightShoulderPoint.x - leftShoulderColorPoint.x;
						int height = 470 - y;

						// 复制图像
						if (!(width < 80 || x <= 0 || y <= 0 || x >= image.cols || y >= image.rows || x + width <= 0 || x + width > image.cols || y + height <= 0 || y + height > image.rows || height <= 0 || width <= 0))
						{
								cv::Rect roi(x, y, width, height);
								cImage = image(roi);
						}
						return cImage;
				}

				// 生成fingerPrint
				void getFingerPrint(cv::Mat aImg, double finger[4][8][8][8]){
						for (int p = 0; p < 4; p++)
						{
								for (int i = 0; i < 8; i++)
								{
										for (int j = 0; j < 8; j++)
										{
												for (int k = 0; k < 8; k++)
												{
														finger[p][i][j][k] = 0;
												}
										}
								}
						}
						int rows = aImg.rows;
						int cols = aImg.cols;
						int numOfPix[4] = { 0, 0, 0, 0 };
						uchar* bgrData = aImg.data;
						for (int i = 0; i < rows / 2; i++)
						{
								uchar* ptr = (uchar*)(aImg.data + i * aImg.step);	//第row行数据的起始指针
								for (int j = 0; j < cols / 2; j++)
								{
										// 获取某点元素的bgr值
										int b = *(ptr + 3 * j);
										int g = *(ptr + 3 * j + 1);
										int r = *(ptr + 3 * j + 1);
										if ((b <= 5) && (g >= 250) && (r <= 5)) { }
										else
										{
												numOfPix[0]++;
												finger[0][b / 32][g / 32][r / 32]++;
										}
								}
						}

						for (int i = rows / 2; i < rows; i++)
						{
								uchar* ptr = (uchar*)(aImg.data + i * aImg.step);	//第row行数据的起始指针
								for (int j = 0; j < cols / 2; j++)
								{
										// 获取某点元素的bgr值
										int b = *(ptr + 3 * j);
										int g = *(ptr + 3 * j + 1);
										int r = *(ptr + 3 * j + 1);
										if ((b <= 5) && (g >= 250) && (r <= 5)) { }
										else
										{
												numOfPix[1]++;
												finger[1][b / 32][g / 32][r / 32]++;
										}
								}
						}

						for (int i = 0; i < rows / 2; i++)
						{
								uchar* ptr = (uchar*)(aImg.data + i * aImg.step);	//第row行数据的起始指针
								for (int j = cols / 2; j < cols; j++)
								{
										// 获取某点元素的bgr值
										int b = *(ptr + 3 * j);
										int g = *(ptr + 3 * j + 1);
										int r = *(ptr + 3 * j + 1);
										if ((b <= 5) && (g >= 250) && (r <= 5)) { }
										else
										{
												numOfPix[2]++;
												finger[2][b / 32][g / 32][r / 32]++;
										}
								}
						}

						for (int i = rows / 2; i < rows; i++)
						{
								uchar* ptr = (uchar*)(aImg.data + i * aImg.step);	//第row行数据的起始指针
								for (int j = cols / 2; j < cols; j++)
								{
										// 获取某点元素的bgr值
										int b = *(ptr + 3 * j);
										int g = *(ptr + 3 * j + 1);
										int r = *(ptr + 3 * j + 1);
										if ((b <= 5) && (g >= 250) && (r <= 5)) { }
										else
										{
												numOfPix[3]++;
												finger[3][b / 32][g / 32][r / 32]++;
										}
								}
						}


						//  对fingerPrint进行归一化
						for (int p = 0; p < 4; p++)
						{
								for (int i = 0; i < 8; i++)
								{
										for (int j = 0; j < 8; j++)
										{
												for (int k = 0; k < 8; k++)
												{
														finger[p][i][j][k] = finger[p][i][j][k] / numOfPix[p];
												}
										}
								}
						}
				}

				// 获取图像区别
				double getDifference(cv::Mat aImage)
				{
						//cout << "aImage : " << aImage.rows << ", " << aImage.cols << endl;
						//cout << "aImage : " << img.rows << ", " << img.cols << endl;
						double result = 0;
						double aFingerPrint[4][8][8][8];

						if(aImage.rows == 0 || aImage.cols == 0){
								return 0.00;
						}

						// 获取指纹
						getFingerPrint(aImage, aFingerPrint);
						for (int p = 0; p < 4; p++)
						{
								// 获取暂时结果, 为sin值
								double tempResult;
								// 计算被除数
								double dividend = 0;
								for (int i = 0; i < 8; i++)
								{
										for (int j = 0; j < 8; j++)
										{
												for (int k = 0; k < 8; k++)
												{
														dividend += this -> fingerPrint[p][i][j][k] * aFingerPrint[p][i][j][k];
												}
										}
								}

								// 计算除数
								double divisor = 0; // divisor = divisor1 * divisor2
								double divisor1 = 0;
								double divisor2 = 0;

								for (int i = 0; i < 8; i++)
								{
										for (int j = 0; j < 8; j++)
										{
												for (int k = 0; k < 8; k++)
												{
														if ((this -> fingerPrint[p][i][j][k] == 0) && (aFingerPrint[p][i][j][k] == 0))
														{ }
														else
														{
																divisor1 += this -> fingerPrint[p][i][j] [k] * this -> fingerPrint[p][i][j][k];
														}
												}
										}
								}
								divisor1 = sqrt(divisor1);

								for (int i = 0; i < 8; i++)
								{
										for (int j = 0; j < 8; j++)
										{
												for (int k = 0; k < 8; k++)
												{
														if ((this -> fingerPrint[p][i][j][k] == 0) && (aFingerPrint[p][i][j][k] == 0))
														{ }
														else
														{
																divisor2 += aFingerPrint[p][i][j][k] * aFingerPrint[p][i][j][k];
														}
												}
										}
								}
								divisor2 = sqrt(divisor2);
								divisor = divisor1 * divisor2;

								// 获取结果值
								tempResult = dividend / divisor;
								result += tempResult;
						}
						result = result / 4;

						// 返回结果
						return result * 100;
				}
};


#endif
