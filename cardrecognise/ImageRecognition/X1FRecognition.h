#ifndef _X1FRECOGNITION_H
#define _X1FRECOGNITION_H

//#include "ImgRecognition.h"
//#include "opencv2/opencv.hpp"
#include "ImgRecognition.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>


using namespace cv;
using namespace std;
typedef std::pair<CvSVM*, string> SVMXML;

class X1FRecognition :public IRecognition, public CvSVM
{
public:
	X1FRecognition();
	virtual ~X1FRecognition();
	virtual int StartRecon(std::vector<Mat>& ImgArr);
	Mat asRowMatrix(InputArrayOfArrays src, int rtype, double alpha = 1, double beta = 0);
protected:
	CvMat* m_pSampleTestMat;
private:
	void Sharpening(const Mat& myImage, Mat& Result);  //����ͼ����ǿ
	Mat FindContours(const Mat& imgSrc, float minPix, float maxPix);
	int EliminateBoundaryDisturb(Mat& BinaryImage);   //�����Ƕ�ֵͼ�������߽����
	int EliminateUpDownDisturb(const int threadValue, Mat& BinaryImage);  
	bool HogFeatureOfOneImageForSVM(const Mat& imgSrc, Mat &outimg);
	std::vector<float> descriptors;
	HOGDescriptor* m_pHog;
	//CString path;
	CvSVMParams params;                     //SVM��������
	void read_params_set_CH();
	void read_CH(CvFileStorage* fs, CvFileNode* svm_node);
	long int nu;
	int nub;
	//void  readlocale(int& num);
	int getThreshVal_optic(const Mat& _src);
	bool ellipse_calc_bin_imgMat(const Mat img_cor, Mat & img_bin, const int meanThreshold);  //ѡ�������Զ���ֵ�����߽�����
	bool RealReadLocale(const int SpecialNum, const int CardNum, const int SelectCard);
	void  Seed_Filling(const Mat& binImg, Mat& lableImg, int &num);   //������䷨
	int  Label_connect(const Mat& labelImg, int &num, int &countnum, int &heightLabel, int &realSecheight, bool &addlabel);
private:                 //����ʶ���ʶ
	struct recongniseLable
	{
		BOOL R_MidEmptyUp;   //�������м�ձ��
		BOOL R_MidEmptyDown;  //�������м�ձ��
		BOOL R_MidEmptyMid;  //�������м�ձ��
		BOOL C_MidEmptyLeft;   //�������м�ձ��
		BOOL C_MidEmptyRight;  //�������м�ձ��
		BOOL C_MidEmptyMid;  //�������м�ձ��
		BOOL R_AreaNum1;        //����ͶӰ������
		BOOL R_AreaNum2;
		BOOL R_AreaNum3;
		BOOL C_AreaNum1;    //����ͶӰ������
		BOOL C_AreaNum2;
		BOOL C_AreaNum3;
		BOOL charcter1;    //һ��
		BOOL charcter2;
		BOOL charcter3;
		BOOL Giveupcharcter; //������ʶ����
		int  C_Width;

	}crossdetect1;// , crossdetect2;
	int TagLableBrand(const Mat& imgSrc);
	int SecondPredict(int response);
private: // ͼ�������
	void findX(Mat &imgSrc, int* min, int* max);
	void findY(Mat &imgSrc, int* min, int* max);
	CvRect findBB(Mat &imgSrc);
	Mat preprocessing(Mat &imgSrc, int new_width, int new_height);

};

#endif


