#include <iostream>
#include <queue>
#include <unistd.h>
#include <wiringPi.h> //라즈베리파이 Input/Output 라이브러리
#include <raspicam/raspicam_cv.h> //라즈베리파이 Camera 라이브러리

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define PI 3.141592

#define STARTPIN 21 
//라즈베리파이 첫번째 PIN
#define ENDPIN 28 
//라즈베리파이 마지막 PIN

using namespace cv;
using namespace std;

cv::Mat myPerspective(Mat frame,Point TopLeft, Point TopRight, Point BottomRight, Point BottomLeft); //화면 왜곡시키기 (사다리꼴 -> 직사각형으로)
cv::Point CrossPoint(cv::Point x1, Point x2, Point x3, Point x4);// 두 직선간의 만나는 점 구하기
double angle(int x1, int y1, int x2, int y2);// 두 직선간의 각도 구하기
bool CompareAvgHoughlines(Mat result, Mat perspective, std::vector<cv::Vec2f> lines, float CompareX, bool LessTrue);
void AngleSumThread();
double AvgHoughlinesAngle(Mat result, std::vector<cv::Vec2f> lines);//검출된 직선의 평균 각도 (y축이 90도로 봄)
void DrawHoughlines(Mat result, Mat frame, std::vector<cv::Vec2f> lines);//검출된 직선 frame에 그리기

bool initialize(Mat frame,int rectmoveX, int rectmoveY, int rectsizeX, int rectsizeY,
				uint persLeftTopX, uint persRightTopX, uint persRightBottomX, uint persLeftBottomX);
//내가 설정한 직선과 차선 일치할 때까지 기다리는 함수

int hough(Mat frame, int rectmoveX, int rectmoveY, int rectsizeX, int rectsizeY,
		  uint persLeftTopX, uint persRightTopX, uint persRightBottomX, uint persLeftBottomX, int BeforeValue);
//검출된 차선의 각도 반환 함수

void digitalWriteBinary(int start,int num);//(라즈베리파이 전용)2진수로 값 output에 보내기


int main ( int argc,char **argv ) {
	//라즈베리파이 메인함수
	raspicam::RaspiCam_Cv Camera;

	Mat frame;

	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set( CV_CAP_PROP_FRAME_WIDTH, 640 );
	Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 480);


	if (!Camera.open()) {
		cerr<<"Error opening the camera"<<endl;return -1;
	}
	if (wiringPiSetup()==-1) {
		cerr<<"Error setting up the wiringPi"<<endl;return -1;
	}

	for(int i = STARTPIN; i<=ENDPIN ; i++)
		pinMode(i,OUTPUT);

	cv::Size frameSize(640,480);

	string myText = "Initializing ";
	string myText2 = "q,w : LEFTTOP / e,r : RIGHTTOP";
	string myText3 = "z,x : LEFTBTM / c,v : RIGHTBTM";

	cv::Point myPoint;
	myPoint.x = 10;
	myPoint.y = 40;
	int myFontFace = 2;
	double myFontScale = 0.5;

	int i =0;

	uint persLTX = 53, persLBX = 10, persRTX = 53, persRBX =10;
	
	int houghvalue = 0;

	int beforevalue = 0;

	std::queue<int> HoughQueue;

	while(1){
		Camera.grab();

		Camera.retrieve(frame);

		int rectmoveX =20, rectmoveY = frame.rows*13/20, rectsizeX = frame.cols-40, rectsizeY = frame.rows*3/20;

		houghvalue = hough(frame, rectmoveX, rectmoveY, rectsizeX, rectsizeY, persLTX, persRTX, persRBX, persLBX, beforevalue);

		cv::circle(frame, Point(frame.cols/2,frame.rows),70,Scalar(255,0,0),8);

		cv::line(frame, Point(frame.cols/2,frame.rows), Point(frame.cols/2+70*cos(houghvalue*PI/180),frame.rows-70*sin(houghvalue*PI/180)),Scalar(0,0,255),4);

		myText = "degree : " + to_string(houghvalue);
		cv::putText( frame, myText, Point(frame.cols/2-50, frame.rows-80), myFontFace, myFontScale, Scalar(255,0,0),1.5);

		printf("\n\nhoughvalue = %d\n",houghvalue);
		
		HoughQueue.push(houghvalue);

		if(HoughQueue.size() > 2){
			digitalWriteBinary(STARTPIN, HoughQueue.front());
			HoughQueue.pop();
		}

		imshow("frame",frame);

		beforevalue = houghvalue;

		if ( waitKey(1) == 27 ) break; //ESC  
	}
	Camera.release();
}

cv::Mat myPerspective(Mat frame,Point TopLeft, Point TopRight, Point BottomRight, Point BottomLeft){ //화면 왜곡시키기 (사다리꼴 -> 직사각형으로)

	Mat dstframe;

	std::vector<cv::Point> rect;

	rect.push_back(TopLeft);
	rect.push_back(TopRight);
	rect.push_back(BottomRight);
	rect.push_back(BottomLeft);

	double w1 = sqrt(pow(BottomRight.x - BottomLeft.x, 2) + pow(BottomRight.x - BottomLeft.x, 2));
	double w2 = sqrt(pow(TopRight.x - TopLeft.x, 2) + pow(TopRight.x - TopLeft.x, 2));
	double h1 = sqrt(pow(TopRight.y - BottomRight.y, 2) + pow(TopRight.y - BottomRight.y, 2));
	double h2 = sqrt(pow(TopLeft.y - BottomLeft.y, 2) + pow(TopLeft.y - BottomLeft.y, 2));
	double maxWidth = (w1 < w2) ? w1 : w2;
	double maxHeight = (h1 < h2) ? h1 : h2;

	Point2f src[4], dst[4];

	src[0] = Point2f(TopLeft.x, TopLeft.y);
	src[1] = Point2f(TopRight.x, TopRight.y);
	src[2] = Point2f(BottomRight.x, BottomRight.y);
	src[3] = Point2f(BottomLeft.x, BottomLeft.y);

	dst[0] = Point2f(0, 0);
	dst[1] = Point2f(maxWidth - 1, 0);
	dst[2] = Point2f(maxWidth - 1, maxHeight - 1);
	dst[3] = Point2f(0, maxHeight - 1);

	Mat transformMatrix = getPerspectiveTransform(src, dst);

	warpPerspective(frame, dstframe, transformMatrix, Size(maxWidth, maxHeight));

	return dstframe;
}

cv::Point CrossPoint(cv::Point x1, Point x2, Point x3, Point x4){// 두 직선간의 만나는 점 구하기
	float fincrease1, fconstant1, fsamevalue1;
	float fincrease2, fconstant2, fsamevalue2;

	if (x1.x == x2.x)
		fsamevalue1 = x1.x;
	else{
		fincrease1 = (float)(x2.y - x1.y) / (x2.x - x1.x);
		fconstant1 = x1.y - fincrease1 * x1.x;
	}
	if (x3.x == x4.x)
		fsamevalue2 = x3.x;
	else{
		fincrease2 = (float)(x4.y - x3.y) / (x4.x - x3.x);
		fconstant2 = x3.y - fincrease2 * x3.x;
	}

	cv::Point Result;
	if (x1.x == x2.x && x3.x == x4.x) return Point(-1, -1);
	if (x1.x == x2.x)
		Result = Point(fsamevalue1, fincrease2*fsamevalue1 + fconstant2);
	else if (x3.x == x4.x)
		Result = Point(fsamevalue2, fincrease1*fsamevalue2 + fconstant1);
	else{
		Result.x = -(fconstant1 - fconstant2) / (fincrease1 - fincrease2);
		Result.y = fincrease1 * Result.x + fconstant1;
	}
	return Result;
}

double angle(int x1, int y1, int x2, int y2)// 두 직선간의 각도 구하기
{
	double dx, dy, da;

	dx = x2 - x1;
	dy = y2 - y1;

	if (!dx) dx = 1e-6;

	da = atan(dy / dx);

	if (dx<0) da += PI;

	da = da * 180 / PI;
	return da;
}

bool CompareAvgHoughlines(Mat result, Mat perspective, std::vector<cv::Vec2f> lines, int CompareX, bool LessTrue){
	//initialize 함수에서 직선이 내가 설정한 직선과 맞는지 비교

	std::vector<cv::Vec2f>::const_iterator it = lines.begin();

	float sumpt1x=0;
	float sumpt2x=0;

	float pt1x, pt1y;
	float pt2x, pt2y;
	int i = 0;

	for (;it != lines.end();++it) {
		float rho = (*it)[0];   //    rho 
		float theta = (*it)[1]; //     
		printf("theta %d)%.4lf\n", i++, theta);
		if (theta < PI / 4.) { //   1/4
			pt1x = (rho/cos(theta));
			pt2x = (rho-result.rows*sin(theta))/cos(theta);
		}
		else if (theta > 3.*PI / 4.) { //   3/4 
			pt1x = (rho/cos(theta));
			pt2x = (rho-result.rows*sin(theta))/cos(theta);
		}
		else if (theta > PI / 4.){  
			if (theta < 2.*PI / 4.){  
				pt1x = 0;
				pt2x = result.cols;
			}
			else{
				pt1x = 0;
				pt2x = result.cols;
			}
		}
		sumpt1x = sumpt1x + pt1x;
		sumpt2x = sumpt2x + pt2x;
		std::cout << "line: (" << rho << "," << theta << ")\n";
	}
	if(LessTrue){
		if(sumpt1x/lines.size() <= CompareX && sumpt2x/lines.size() <= CompareX)
			return true;
	}else{
		if(sumpt1x/lines.size() >= CompareX && sumpt2x/lines.size() >= CompareX)
			return true;
	}
	printf("Didn't Initialize\n");
	return false;
}

double AvgHoughlinesAngle(Mat result, std::vector<cv::Vec2f> lines){
	//검출된 직선의 평균 각도 (y축이 90도로 봄)
	std::vector<cv::Vec2f>::const_iterator it = lines.begin();
	int i = 0;
	double anglesum = 0;
	double anglevalue;
	int cases = 0;
	for (;it != lines.end();++it){

		float rho = (*it)[0];   //    rho 
		float theta = (*it)[1]; //     

		if (theta < PI / 4.) { //   1/4
			cases = 1;
			anglevalue = 180 - (theta/PI * 180 + 90);
			//anglevalue = angle(rho / cos(theta) , 0 , (rho - result.rows*sin(theta)) / cos(theta) , result.rows);			

		}
		else if (theta > 3.*PI / 4.) { //   3/4
			anglevalue = 180 - (theta/PI * 180 - 90);
			//anglevalue = angle(rho / cos(theta) , 0 , (rho - result.rows*sin(theta)) / cos(theta) , result.rows);			
			//angle((rho - result.rows*sin(theta)) / cos(theta) , result.rows, rho / cos(theta) , 0);

			cases = 2;
		}
		else if (theta > PI / 4.){ 
			if(theta < 2.*PI / 4.){
				anglevalue = angle( 0, rho / sin(theta),result.cols, (rho - result.cols*sin(theta)) / sin(theta)  );
				cases = 3;
			}	
			else{ 
				anglevalue = angle(result.cols, (rho - result.cols*sin(theta)) / sin(theta) , 0, rho / sin(theta) );
				cases = 4;
			}
		}	

		printf("theta %d)%.4lf, case = %d\n", i++, theta,cases);

		anglesum = anglesum + anglevalue; 
		printf("angle%d : %.3lf\n",i,anglevalue);
		std::cout << "line: (" << rho << "," << theta << ")\n";
	}

	printf("angle average = %.3lf\n", anglesum/lines.size());

	return anglesum/(double)lines.size();
} 

void digitalWriteBinary(int start,int num){
	//2진수로 값 output에 보내기
	int decimal = num;
	int binary[8] = { 0, };

	int position = 0;
	while (1)
	{
		binary[position] = decimal % 2;    // 2로 나누었을 때 나머지를 배열에 저장
		decimal = decimal / 2;             // 2로 나눈 몫을 저장

		position++;    // 자릿수 변경

		if (decimal == 0)    // 몫이 0이 되면 반복을 끝냄
			break;
	}
	//for(int i=0;i<5000;i++);
	// 배열의 요소를 역순으로 출력
	for (int i = 7; i >= 0; i--)
	{
		digitalWrite(start+i, binary[i]);
	}
}

void DrawHoughlines(Mat result, Mat frame, std::vector<cv::Vec2f> lines){
	//검출된 직선 frame에 그리기
	std::vector<cv::Vec2f>::const_iterator it = lines.begin();
	int i = 0;
	for (;it != lines.end();++it) {
		float rho = (*it)[0];   //    rho 
		float theta = (*it)[1]; //     
		printf("theta %d)%.4lf\n", i++, theta);
		if (theta < PI / 4.) { //   1/4
			cv::Point pt1((rho / cos(theta)), 0); //        
			cv::Point pt2((rho - result.rows*sin(theta)) / cos(theta), result.rows);

			cv::line(frame, pt1, pt2, cv::Scalar(255, 255, 0), 1); //   
			printf("angle%d : %.3lf\n",i,angle(pt1.x,pt1.y,pt2.x,pt2.y));
		}
		else if (theta > 3.*PI / 4.) { //   3/4 
			cv::Point pt1((rho / cos(theta)), 0); //        
			cv::Point pt2((rho - result.rows*sin(theta)) / cos(theta), result.rows);
			cv::line(frame, pt1, pt2, cv::Scalar(255, 255, 255), 1); //   	
			printf("angle%d : %.3lf\n",i,angle(pt1.x,pt1.y,pt2.x,pt2.y));
		}
		else if (theta > PI / 4.){  
			if (theta < 2.*PI / 4.){  
				cv::Point pt1(0, rho / sin(theta));       
				cv::Point pt2(result.cols, (rho - result.cols*cos(theta)) / sin(theta));
				cv::line(frame, pt1, pt2, cv::Scalar(255, 0, 0), 1); 
				printf("angle%d : %.3lf\n",i,angle(pt1.x,pt1.y,pt2.x,pt2.y));
			}
			else{
				cv::Point pt1(0, rho / sin(theta));        
				cv::Point pt2(result.cols, (rho - result.cols*cos(theta)) / sin(theta));
				//     

				cv::line(frame, pt1, pt2, cv::Scalar(0, 255, 0), 1); //   
				printf("angle%d : %.3lf\n",i,angle(pt1.x,pt1.y,pt2.x,pt2.y));
			}
		}
		std::cout << "line: (" << rho << "," << theta << ")\n";
	}
} 

bool initialize(Mat frame,int rectmoveX, int rectmoveY, int rectsizeX, int rectsizeY,
				uint persLeftTopX, uint persRightTopX, uint persRightBottomX, uint persLeftBottomX){
					//내가 설정한 직선과 차선 일치할 때까지 기다리는 함수
					//지금은 사용 X

					cv::Mat contours;
					cv::Mat perspective;

					Rect rect(rectmoveX, rectmoveY, rectsizeX, rectsizeY);

					Mat frame1 = frame(rect);
					Mat frame2 = frame1.clone();

					resize(frame2, frame2, Size(frame2.cols, frame2.rows * 5), 0, 0, 1);

					perspective = myPerspective(frame2,Point(persLeftTopX,0),Point(rectsizeX-persRightTopX,0),Point(rectsizeX-persRightBottomX,frame2.rows),Point(persLeftBottomX,frame2.rows));

					cv::Canny(perspective, contours, 100, 140);

					Rect perspectiveLeft(20,20,perspective.cols/2-40,perspective.rows-40);
					Rect perspectiveRight(perspective.cols/2+20,20,perspective.cols/2-40,perspective.rows-4);

					Mat PersCannyLeft = contours(perspectiveLeft);
					Mat PersCannyRight = contours(perspectiveRight);

					std::vector<cv::Vec2f> Leftlines;
					std::vector<cv::Vec2f> Rightlines;

					cv::HoughLines(PersCannyLeft, Leftlines, 1, PI / 180.0, 200);  // (vote)  
					cv::Mat resultLeft(PersCannyLeft.rows, PersCannyLeft.cols, CV_8U, cv::Scalar(255));
					std::cout << "Left Lines detected: " << Leftlines.size() << std::endl;
					printf("\n");

					bool LeftCompare = CompareAvgHoughlines(resultLeft, PersCannyLeft, Leftlines, PersCannyLeft.cols/10, true);

					if(LeftCompare)
						cv::line(frame1, Point(persLeftBottomX,rectsizeY), Point(persLeftTopX,0), Scalar(0,255,0), 3);   	
					else
						cv::line(frame1, Point(persLeftBottomX,rectsizeY), Point(persLeftTopX,0), Scalar(0,255,255), 3);

					cv::HoughLines(PersCannyRight, Rightlines, 1, PI / 180.0, 100);  // (vote)  
					cv::Mat resultRight(PersCannyRight.rows, PersCannyRight.cols, CV_8U, cv::Scalar(255));
					std::cout << "Right Lines detected: " << Rightlines.size() << std::endl;
					printf("\n");

					bool RightCompare = CompareAvgHoughlines(resultRight, PersCannyRight,Rightlines, PersCannyRight.cols*9/10, false);
					if(RightCompare)
						cv::line(frame1, Point(rectsizeX-persRightBottomX,rectsizeY), Point(rectsizeX-persRightTopX,0),Scalar(0,255,0), 3);
					else
						cv::line(frame1, Point(rectsizeX-persRightBottomX,rectsizeY), Point(rectsizeX-persRightTopX,0),Scalar(0,255,255), 3);

					if(LeftCompare&RightCompare){
						cv::line(frame1, Point(persLeftBottomX,rectsizeY), Point(rectsizeX-persRightBottomX,rectsizeY), Scalar(0,255,0), 3); //   
						cv::line(frame1, Point(persLeftTopX,0), Point(rectsizeX-persRightTopX,0), Scalar(0,255,0), 3); //   
						return true;
					}
					cv::line(frame1, Point(persLeftBottomX,rectsizeY), Point(rectsizeX-persRightBottomX,rectsizeY), Scalar(0,255,255), 3); //   
					cv::line(frame1, Point(persLeftTopX,0), Point(rectsizeX-persRightTopX,0), Scalar(0,255,255), 3); //   

					return false;
}


int hough(Mat frame, int rectmoveX, int rectmoveY, int rectsizeX, int rectsizeY,
		  uint persLeftTopX, uint persRightTopX, uint persRightBottomX, uint persLeftBottomX,
		  int BeforeValue)
{
	//검출된 차선의 각도 반환 함수
	cv::Mat contours;
	cv::Mat perspective;

	std::vector<cv::Vec2f> lines;     

	Rect rect(rectmoveX, rectmoveY, rectsizeX, rectsizeY);

	cv::Mat frame1 = frame(rect);

	cv::Mat frame2 = frame1.clone();

	resize(frame2, frame2, Size(frame2.cols, frame2.rows * 2), 0, 0, 1);

	perspective = myPerspective(frame2,Point(persLeftTopX,0),Point(rectsizeX-persRightTopX,0),Point(rectsizeX-persRightBottomX,frame2.rows),Point(persLeftBottomX,frame2.rows));

	cv::Canny(perspective, contours, 150, 200);

	Rect perspectiveLeft(2,2,perspective.cols/2-30,perspective.rows-4);
	Rect perspectiveRight(perspective.cols/2+30,2,perspective.cols/2-34,perspective.rows-4);

	cv::Mat PersCannyLeft = contours(perspectiveLeft);
	cv::Mat PersCannyRight = contours(perspectiveRight);

	std::vector<cv::Vec2f> Leftlines;
	std::vector<cv::Vec2f> Rightlines;

	cv::imshow("LeftLines",PersCannyLeft);
	cv::imshow("RightLines",PersCannyRight);

	cv::HoughLines(PersCannyLeft, Leftlines, 1, PI / 180.0, 30);  // (vote)  
	cv::Mat resultLeft(PersCannyLeft.rows, PersCannyLeft.cols, CV_8U, cv::Scalar(255));
	std::cout << "Left Lines detected: " << Leftlines.size() << std::endl;
	printf("\n");

	int LeftAvg = AvgHoughlinesAngle(resultLeft,Leftlines);

	cv::HoughLines(PersCannyRight, Rightlines, 1, PI / 180.0, 30);  // (vote)  
	cv::Mat resultRight(PersCannyRight.rows, PersCannyRight.cols, CV_8U, cv::Scalar(255));
	std::cout << "Right Lines detected: " << Rightlines.size() << std::endl;
	printf("\n");

	int RightAvg = AvgHoughlinesAngle(resultRight,Rightlines);

	int AvgAngle = 90;
	if(Rightlines.size() == 0 && Leftlines.size() == 0){
		AvgAngle = BeforeValue;
	}else if(Rightlines.size() == 0){
		AvgAngle = LeftAvg;
		AvgAngle = AvgAngle - 10;
	}else if(Leftlines.size() == 0){
		AvgAngle = RightAvg;
		AvgAngle = AvgAngle + 10;
	}else{
		AvgAngle = (LeftAvg + RightAvg)/2;
	}
	imshow("Perspective With Canny", contours);

	cv::rectangle(frame, Point(rectmoveX, rectmoveY), Point(rectmoveX + rectsizeX, rectmoveY + rectsizeY), Scalar(0, 0, 255), 1.5, 8, 0);

	return AvgAngle;
}