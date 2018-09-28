#include "opencv2/opencv.hpp"
#include <sstream>
#include <iostream>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>

using namespace cv;
using namespace std;

const float PI = 3.14159265;

int main(){

	int c_start = 237;
	int r_start = 0;
	int c_length = 1649;
	int r_length = 700;
	
	int fd;
	int thresh_level = 150;
	
	int init = 0;

	float theta[1649] = {0};
	float radius[1649] = {0};
	float value_out[1649] = {1080};
	
	
	float ground;
	


	int c_size = 101;

	Mat tempFrame1;
	Mat frame1;
	Mat sub_img;
	Mat gray_img;
	Mat resized_img;
	Mat filtered_img;
	Mat bin_img;
	Mat distorted_pts = cv::Mat(1, c_length, CV_32FC2);
	Mat undistorted_pts = cv::Mat(1, c_length, CV_32FC2);

	Mat_<float> camMat (3, 3);
	Mat_<float> distCoeffs(1, 4);

	Rect ROI(c_start, r_start, c_length, r_length);

	camMat << 1079.167, 0, 964.587, 0, 1091.033, 638.202, 0, 0, 1;
    distCoeffs << -0.307362, 0.06706, 0.000422, 0.005664;
    double fx = 1079.167; double fy = 1091.033; double cx = 964.587; double cy = 638.202;


	VideoCapture video(0);
    video.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    video.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    
  //------------------------------v4l2 setup----------------------//
  fd = open("/dev/video0", O_RDWR);

  v4l2_control c;

  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = V4L2_EXPOSURE_MANUAL;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
  c.value = 0;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value = 150;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_AUTO_WHITE_BALANCE;
  c.value = 0;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_GAMMA;
  c.value = 72;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  //------------------------------v4l2 setup----------------------//
  
	Mat frame;
	while(1){
		
		float sum = 0;
		float avg_dist = 0;
		
		
		video>>tempFrame1;
		if(!tempFrame1.empty())
       {

        sub_img = tempFrame1(ROI);

        cvtColor(sub_img, gray_img, CV_RGB2GRAY);
        resize(gray_img, resized_img, Size(c_size, r_length));
        medianBlur(resized_img, filtered_img, 5);
        threshold(filtered_img, bin_img, thresh_level, 255, THRESH_TOZERO);            // 단순히 threshold로 라인레이져 잡은거 --> 다른 필터로 바꿀 수 있을듯
                                                                             // HSV로...???
//        imshow("img", bin_img);
        for (int c = 0; c < c_size; c++)
        {
            float num = 0;
            float den = 0;
            int count = 0;

            for (int  r = 0; r < r_length; r++) {
              int value = bin_img.at<uchar>(r, c);
              if (value != 0) {
                count = count+1;
                num = num + (r+1)*value;
                den = den + value;
              }
            }
            if (count == 0) {
              value_out[c] = 1;
            }
            else {
              value_out[c] = num/den;
            }
             //------ 1920 * 1080 ----------//
             distorted_pts.at<Vec2f>(0, c)[0] = c*(c_length-1)/(c_size-1)+c_start;
             distorted_pts.at<Vec2f>(0, c)[1] = r_start+value_out[c]-1;
             //------ 1920 * 1080 ----------//

            // ------------  640 * 480 --------------//
          //  distorted_pts.at<Vec2f>(0, c)[0] = 2*(c*(c_length-1)/(c_size-1)+c_start);
          //  distorted_pts.at<Vec2f>(0, c)[1] = 2*(r_start+value_out[c]-1);
            // ------------  640 * 480 --------------//
          }     // 레이져 부분만 찾아서 캘리브레이션

        undistortPoints(distorted_pts, undistorted_pts, camMat, distCoeffs);			// calibration

        //-------------------------------------- 거리값 계산하는 부분 --------------------------------------------//
        for (int c = 0; c < c_size; c++)
        {
          //theta[c] = (-50 + c*100/(c_size-1))*PI/180;
          theta[c] = (-50 + c)*PI/180;
          if (value_out[c_size-1-c] == 1)
            radius[c] = 60000;
          else
           {
           //------ 1920 * 1080 ----------//
             radius[c] = ((-31440/((fy*undistorted_pts.at<Vec2f>(0, c_size-1-c)[1]+cy)/1.5-432.1)+56.36)/cos(theta[c])); //for undistorted points
           //------ 1920 * 1080 ----------//
	         }

          if (radius[c] > 3000)
              radius[c] = 0;		

 //         scan_.ranges[c] = radius[c]/1000.0;      // 왼쪽에서붙터라 100-c 로 바꿔줘야할듯

        }
        
        //calculation of the average distance
        for ( int c = 0; c < c_size; c++){
			sum += radius[c];
		}
		avg_dist = sum / c_size;
		
		
		
		
		cout<<avg_dist<<endl;
		
		
		if(init == 0){
			cout<<"Start Initiation by pressing 1"<<endl;
			cin>>init;
			ground = avg_dist;
			cout<<"Ground set to be "<<avg_dist<<endl;
			
		}
		
		if(avg_dist / ground > 1.05){
			cout<<"Detected!"<<endl;
		
		}


      }



		
		if (waitKey(10)==27) break;
	}
	return 0;
}
