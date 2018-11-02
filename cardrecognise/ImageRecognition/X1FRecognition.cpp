#include <afx.h>
#include <afxwin.h>
#include "X1FRecognition.h"
#include "ImageWindowTestTool.h"
#include <stack>
#include"IRecognitionDll.h"

#include "DataTemplateAtemp.h"

extern bool save2D;
extern bool save2C;
extern bool savebinery2C;
extern bool write2txt;
extern int selectbandflag;
extern float Gammapara ;
extern int unigipara ;
extern int unitepara ;
extern int openhandle ;
#define   MAXNUMTEST 999999.0
const int n_AllCardNum = 4;  //特殊字体

static const char* SpecialCardName[5] = { "TrainCard", "TrainCardA", "TrainCardB", "TrainCardC", "TrainCardD" };
static const char* xmlModelname[74] = { "", "一万", "二万", "三万", "四万", "五万", "六万", "七万", "八万", "九万", \
"一筒", "二筒", "三筒", "四筒", "五筒", "六筒", "七筒", "八筒", "九筒", "一条", "二条", \
"三条", "四条", "五条", "六条", "七条", "八条", "九条", "东风", "南风", "西风", "北风", \
"红中", "发财", "白板", "春", "夏", "秋", "冬", "梅", "兰", "菊", "竹", "一万倒", "二万倒", "三万倒", "四万倒", "五万倒", "六万倒", "七万倒", "八万倒", \
"九万倒", "六筒倒", "七筒倒", "一条倒", "三条倒", "七条倒", "东风倒", "南风倒", "西风倒", "北风倒", "红中倒", "发财倒", "春倒", "夏倒", "秋倒", "冬倒", "梅倒", "兰倒", "菊倒", "竹倒", "百搭", "百搭倒", "纯白" };

X1FRecognition::X1FRecognition()
{
	//m_element = getStructuringElement(MORPH_RECT, Size(3, 3));
	params.svm_type = C_SVC;        // SVM类型设置
	params.C = 0.1;
	params.kernel_type = LINEAR;            //核函数类型：线性
	params.term_crit = TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 10000, 1e-6); // 终止准则函数：当迭代次数达到最大值时终止
	m_pHog = new cv::HOGDescriptor(cvSize(64, 128), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);   //hog初始化化
	crossdetect1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

}

X1FRecognition::~X1FRecognition()
{
	if (m_pHog != NULL)delete m_pHog;
}
/********
二次识别标记，必须是灰度图，最好是二值化后的图
*********/
int X1FRecognition::TagLableBrand(const Mat& imgSrc)
{
	Mat imForForm;
	if (imgSrc.rows != 128 || imgSrc.cols != 64)
	{
		cv::resize(imgSrc, imForForm, cv::Size(64, 128), 0, 0, CV_INTER_LINEAR);
	}
	else
	{
		imForForm = imgSrc;
	}
	crossdetect1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	/**********
	转换为矩阵
	***********/
	int nl = imForForm.rows; //行数
	int nc = imForForm.cols;       //  imForForm.cols*imForForm.channels(); //每行元素的总元素数量
	if (nl < 2)
		return 0;
	uchar ImageDataSets[128][64];
	//ImageDataSets = new uchar*[nl* sizeof(uchar)];

	for (int j = 0; j<nl; j++)       //转换到矩阵
	{
		long int ColSum = 0;
		uchar* data = imForForm.ptr<uchar>(j);
		for (int i = 0; i < nc; i++)
		{
			ImageDataSets[j][i] = data[i];
		}
	}
	/////////////////
	//imshow("1", imForForm);
	//waitKey(0);
	/*******
	矩阵横向搜索，标记空白区域，预测时使用
	********/
	BOOL ProjectorVectorR[128] = { 0 };
	int largelab = 0;
	int savelargelab = 0;
	int lastsaverow = 0;
	int countlableR = 0;
	bool startcountflag = 0;
	BOOL startcount = 0;
	BOOL charcterup1 = 0;
	BOOL charcterup2 = 0;
	BOOL charcterup3 = 0;
	BOOL charcterdown1 = 0;
	BOOL charcterdown2 = 0;
	BOOL charcterdown3 = 0;
	int giveupcharacter = 0;
	BOOL giveuplabelup = 0;
	BOOL giveuplabeldown = 0;
	for (int i = 0; i < nl; i++)
	{
		int colSum = 0;
		int midleflag = 0;    //表示中间区域黑白变化次数
		for (int j = 2; j < nc - 2; j++)
		{
			colSum += ImageDataSets[i][j];
			if ((j>nc / 5) && (j < (4 * nc / 5)))
			{
				if (ImageDataSets[i][j] - ImageDataSets[i][j - 1] > 180)
				{
					if (ImageDataSets[i][j + 1] - ImageDataSets[i][j - 2] > 180)
						midleflag += 1;
				}

			}
		}
		//中空标记
		if (i < (nl / 3) && (midleflag>2))
		{
			crossdetect1.R_MidEmptyUp = 1;
		}
		else if (i > (nl - (nl / 3)) && (midleflag > 2))
		{
			crossdetect1.R_MidEmptyDown = 1;
		}
		else if (midleflag>2)
		{
			crossdetect1.R_MidEmptyMid = 1;
		}
		//计算投影,
		if (colSum > 2 * 255)
		{
			ProjectorVectorR[i] = 1;
		}
		else
		{
			ProjectorVectorR[i] = 0;
		}
		//标记一，二 ，三万

		if (i < (nl / 2)) //&& (midleflag == 1)
		{
			if (colSum >= 8 * 255)
			{
				if (largelab < colSum)
				{
					largelab = colSum;
				}
				giveupcharacter++;
			}
			else
			{
				if (giveupcharacter > 15)
				{
					giveuplabelup = true;
				}
				giveupcharacter = 0;
				if (largelab > 7 * 255) //最小误差点数
				{
					countlableR++;
					if ((countlableR == 1) && (largelab > 30 * 255))
					{
						charcterup1 = 1;
					}
					else if ((countlableR == 2) && (savelargelab < largelab) && (largelab > 28 * 255))
					{
						charcterup2 = 1;
						charcterup1 = 0;
						if (i - lastsaverow < 4)
						{
							giveuplabelup = true;
						}

					}
					else if ((countlableR>2) && (savelargelab < largelab) && (largelab > 28 * 255) && (i - lastsaverow<3))
					{
						charcterup3 = 1;
						charcterup1 = 0;
						charcterup2 = 0;
						if (i - lastsaverow < 4)
						{
							giveuplabelup = true;
						}
					}
					savelargelab = largelab;
					lastsaverow = i;
				}

				largelab = 0;
			}
		}
		if (i == (nl / 2)) //下半部分清零
		{
			largelab = 0;
			savelargelab = 0;
			countlableR = 0;
			giveupcharacter = 0;
			lastsaverow = 0;

		}
		if (i > (nl - (nl / 2)) && (startcountflag != true))
		{
			if (colSum >= 30 * 255)
				startcountflag = true;
		}
		if (startcountflag)
		{
			if (colSum >= 8 * 255)
			{
				if (largelab < colSum)
				{
					largelab = colSum;
				}
				giveupcharacter++;
			}
			else
			{
				if (giveupcharacter > 17)
				{
					giveuplabeldown = true;
				}
				giveupcharacter = 0;
				if (largelab > 4 * 255) //最小误差点数
				{
					countlableR++;
					charcterdown1 = 1;
					if ((countlableR == 2) && (savelargelab > largelab))
					{
						charcterdown2 = 1;
						charcterdown1 = 0;
						if (i - lastsaverow < 4)
						{
							giveuplabeldown = true;
						}
					}
					else if ((countlableR >= 3) && (savelargelab<27 * 255))
					{
						charcterdown3 = 1;
						charcterdown2 = 0;
						charcterdown1 = 0;
						if (i - lastsaverow < 4)
						{
							giveuplabeldown = true;
						}
					}
					savelargelab = largelab;
				}
				largelab = 0;
				lastsaverow = i;
			}
		}
	}
	int lastprojectorlabe = 0;
	int coutprojectorlabe = 0;
	int selectcountwide = 0; //统计阴影宽度
	int savenum1 = 0;    //保存每个阴影宽度
	int savenum2 = 0;
	int savenum3 = 0;
	int savenum4 = 0;
	for (int i = 1; i < nl - 2; i++)
	{
		selectcountwide++;
		if ((lastprojectorlabe != ProjectorVectorR[i]) && (ProjectorVectorR[i] == ProjectorVectorR[i + 1]) && (ProjectorVectorR[i + 1] == ProjectorVectorR[i + 2]))
		{
			coutprojectorlabe++;
			if (savenum1 == 0)
			{
				savenum1 = selectcountwide;
				selectcountwide = 0;
			}
			else if (savenum2 == 0)
			{
				savenum2 = selectcountwide;
				selectcountwide = 0;
			}
			else if (savenum3 == 0)
			{
				savenum3 = selectcountwide;
				selectcountwide = 0;
			}
			else //if (savenum4 == 0)
			{
				savenum4 = selectcountwide;
				//selectcountwide = 0;
			}
		}
		lastprojectorlabe = ProjectorVectorR[i];
	}
	switch (coutprojectorlabe)
	{
	case 1:
	case 2:
		crossdetect1.R_AreaNum1 = 1;
		break;
	case 3:
	case 4:
		crossdetect1.R_AreaNum2 = 1;
		break;
	default:
		crossdetect1.R_AreaNum3 = 1;
		break;
	}
	if (savenum1<40 && savenum2<40)//&&crossdetect1.R_MidEmptyDown)   //注释掉上下空是因为一，二有些是弯曲的也会被判断为空
	{
		crossdetect1.Giveupcharcter = giveuplabelup;
		crossdetect1.charcter1 = charcterup1;
		crossdetect1.charcter2 = charcterup2;
		crossdetect1.charcter3 = charcterup3;
	}
	else// if (crossdetect1.R_MidEmptyUp)
	{
		crossdetect1.Giveupcharcter = giveuplabeldown;
		crossdetect1.charcter1 = charcterdown1;
		crossdetect1.charcter2 = charcterdown2;
		crossdetect1.charcter3 = charcterdown3;
	}
	/*******
	矩阵纵向搜索，标记空白区域，预测时使用
	********/
	uchar ProjectorVectorC[64] = { 0 };
	int largelabC = 0;
	int savelargelabC = 0;
	int countlableC = 0;
	for (int i = 0; i < nc; i++)
	{
		int RowSum = 0;
		int midleflag = 0;    //表示中间区域黑白变化次数
		for (int j = 2; j < nl - 2; j++)
		{
			RowSum += ImageDataSets[j][i];
			if ((j>nl / 5) && (j < (4 * nl / 5)))
			{
				if (ImageDataSets[j][i] - ImageDataSets[j - 1][i - 1] > 180)
				{
					if (ImageDataSets[j][i + 1] - ImageDataSets[j][i - 2] > 180)
						midleflag += 1;
				}
			}
		}
		//中空标记
		if (i < (nc / 3) && (midleflag>2))
		{
			crossdetect1.C_MidEmptyLeft = 1;
		}
		else if (i > (nc - (nc / 3)) && (midleflag > 2))
		{
			crossdetect1.C_MidEmptyRight = 1;
		}
		else if (midleflag>2)
		{
			crossdetect1.C_MidEmptyMid = 1;
		}
		//计算投影
		if (RowSum > 3 * 255)
		{
			ProjectorVectorC[i] = 1;
		}
		else
		{
			ProjectorVectorC[i] = 0;
		}

	}
	lastprojectorlabe = 0;
	coutprojectorlabe = 0;
	int rememberLocale = 0;
	for (int i = 1; i < nc - 1; i++)
	{
		if (lastprojectorlabe != ProjectorVectorC[i])
		{
			if (i - rememberLocale < nc / 10)
			{
				crossdetect1.C_Width = 0;
			}
			else if (i - rememberLocale<nc / 3)
			{
				coutprojectorlabe++;
				crossdetect1.C_Width++;   //一定宽的投影个数
			}
			else
			{
				coutprojectorlabe++;
				crossdetect1.C_Width = 10;   //表示足够宽
			}
			rememberLocale = i;
		}
		lastprojectorlabe = ProjectorVectorC[i];
	}
	switch (coutprojectorlabe)
	{
	case 1:
	case 2:
		crossdetect1.C_AreaNum1 = 1;
		break;
	case 3:
	case 4:
		crossdetect1.C_AreaNum2 = 1;
		break;
	default:
		crossdetect1.C_AreaNum3 = 1;
		break;
	}
	return 1;
}

void X1FRecognition::Sharpening(const Mat& myImage, Mat& Result)  //掩码图像增强，主要是解决图像轮廓不明显，过白，且噪点少的图像增强
{
	CV_Assert(myImage.depth() == CV_8U);  // 仅接受uchar图像
	//Result.create(myImage.size(), myImage.type());
	Result = myImage.clone();
	const int nChannels = myImage.channels();

	for (int j = 1; j < myImage.rows - 1; ++j)
	{
		const uchar* previous = myImage.ptr<uchar>(j - 1);
		const uchar* current = myImage.ptr<uchar>(j);
		const uchar* next = myImage.ptr<uchar>(j + 1);

		uchar* output = Result.ptr<uchar>(j);

		for (int i = nChannels; i < nChannels*(myImage.cols - 1); ++i)
		{
			*output++ = saturate_cast<uchar>(5 * current[i]
				- current[i - nChannels] - current[i + nChannels] - previous[i] - next[i]);
		}
	}

	//Result.row(0).setTo(Scalar(200));
	//Result.row(Result.rows - 1).setTo(Scalar(0));
	//Result.col(0).setTo(Scalar(0));
	//Result.col(Result.cols - 1).setTo(Scalar(0));
}

int X1FRecognition::getThreshVal_optic(const Mat& _src)
{
	Size size = _src.size();
#if 0
	if (_src.isContinuous())
	{
		size.width *= size.height;
		size.height = 1;
	}
#endif
	const int N = 256;
	int i, j, h[N] = { 0 };
	for (i = 0; i < size.height; i++)
	{
		const uchar* src = _src.data + _src.step*i;
		j = 0;
		for (; j < size.width; j++)
			h[src[j]]++;
	}

	double mu = 0, scale = 1. / (size.width*size.height);
	for (i = 0; i < N; i++)
		mu += i*(double)h[i];

	mu *= scale;
	double mu1 = 0, q1 = 0;
	int max_sigma = 0, max_val = 0;

	for (i = 0; i < N; i++)
	{
		double p_i, q2, mu2, sigma;

		p_i = h[i] * scale;
		mu1 *= q1;
		q1 += p_i;
		q2 = 1. - q1;

		if (std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON)
			continue;

		mu1 = (mu1 + i*p_i) / q1;
		mu2 = (mu - q1*mu1) / q2;
		sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
		if (sigma > max_sigma)
		{
			max_sigma = sigma;
			max_val = i;
		}
	}

	return max_val;
}


void X1FRecognition::findX(Mat &imgSrc, int* min, int* max)
{
	int i;
	int minFound = 0;
	CvMat data;
	IplImage* dest = &IplImage(imgSrc);
	CvScalar val = cvRealScalar(0);
	int minVal = 3 * 255;
	int maxVal = (dest->height / 25) * 255;
	int nextMaxval = maxVal * 6;
	int firsMax = 0;
	int firsMin = 0;
	int firsFound = 0;
	bool changemax = false;
	int  countadd = 0;
	for (i = 0; i< dest->width; i++)
	{
		cvGetCol(dest, &data, i);
		val = cvSum(&data);
		if (val.val[0]> minVal)
		{
			firsMax = i;
			if (!firsFound)
			{
				firsMin = i;
				firsFound = 1;
			}
			if (changemax)
			{
				*max = firsMax;
			}
			if ((val.val[0] > maxVal && countadd>2) || (val.val[0] > nextMaxval) && countadd > 0)
			{
				changemax = true;
				*max = i;
				if (!minFound)
				{
					*min = firsMin;
					minFound = 1;
				}
			}
			++countadd;
		}
		else
		{
			changemax = false;
			firsFound = 0;
			countadd = 0;
		}
	}
}

void X1FRecognition::findY(Mat &imgSrc, int* min, int* max)
{
	int i;
	int minFound = 0;
	CvMat data;
	IplImage* dest = &IplImage(imgSrc);
	CvScalar val = cvRealScalar(0);
	int minVal = 2 * 255;
	int maxVal = (dest->width / 15) * 255;
	int firsMax = 0;
	int firsMin = 0;
	int firsFound = 0;
	bool changemax = false;
	int  countadd = 0;
	for (i = 0; i< dest->height; i++)
	{
		cvGetRow(dest, &data, i);
		val = cvSum(&data);
		if (val.val[0]> minVal)
		{
			firsMax = i;
			if (!firsFound)
			{
				firsMin = i;
				firsFound = 1;
			}
			if (changemax)
			{
				*max = firsMax;
			}
			if (val.val[0] > maxVal && countadd>1)
			{
				changemax = true;
				*max = i;
				if (!minFound)
				{
					*min = firsMin;
					minFound = 1;
				}
			}
			++countadd;
		}
		else
		{
			countadd = 0;
			changemax = false;
			firsFound = 0;
		}
	}
}
CvRect X1FRecognition::findBB(Mat &imgSrc)
{
	CvRect aux;
	int xmin, xmax, ymin, ymax;
	xmin = xmax = ymin = ymax = 0;
	findX(imgSrc, &xmin, &xmax);
	findY(imgSrc, &ymin, &ymax);
	int nl = imgSrc.rows;//使得返回区域有一个余量
	int nc = imgSrc.cols;
	if (xmin > 1)
		xmin -= 1;
	if (ymin > 2)
		ymin -= 2;
	if (nl - 4 > ymax)
		ymax += 4;
	if (nc - 2 > xmax)
		xmax += 2;
	aux = cvRect(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1);//此处代码为改进后
	//aux = cvRect(0, ymin, nc, ymax - ymin + 1);//此处代码为改进后
	return aux;

}
Mat X1FRecognition::preprocessing(Mat &imgSrc, int new_width, int new_height)
{
	assert(imgSrc.depth() != sizeof(uchar));
	int nc = imgSrc.rows;
	int nl = imgSrc.cols;
	CvRect bb;//bounding box
	bb = findBB(imgSrc);
	if (bb.height <  nc / 3)  //百搭纯白
	{
		if (bb.width < nl / 2)  //二条不被消
		{
			bb.x = nl / 4;
			bb.width = nl / 2;
		}
		bb.y = nc / 6;
		bb.height = nc / 3;
	}
	Mat result = imgSrc(bb);
	Mat scaledResult;
	int nly = nl / 8;
	int ncx = nc / 8;
	if (bb.width < nl / 4 || bb.height < nl / 4)
	{
		scaledResult = result;
	}
	else
	{
		resize(result, scaledResult, cv::Size(bb.width, nc - ncx), 0, 0, CV_INTER_NN);
	}
	Mat addborder;
	int x1 = (nl - scaledResult.cols) / 2;
	int y1 = (nc - scaledResult.rows) / 2;
	copyMakeBorder(scaledResult, addborder, y1, y1, x1, x1, BORDER_CONSTANT, Scalar::all(0));  //加0边界,left,right,up,down
	return addborder;
}

int X1FRecognition::EliminateBoundaryDisturb(Mat& BinaryImage)   //输入是二值图，消除边界干扰
{
	assert(BinaryImage.depth() != sizeof(uchar));
	assert(BinaryImage.data);
	int nc = BinaryImage.rows;  //行
	int nl = BinaryImage.cols;  //列
	int midnumR1 = nc / 8;
	int midnumR2 = nc / 5;
	int midnumC = nl / 6;
	
	int leftUp = 0;
	int leftDown = 0;
	int rightUp = 0;
	int rightDown = 0;
	int CountNum = (255 * 2 * nl / 3)/2;
	for (int i = 0; i < nl / 3; i++)
	{
		leftUp += BinaryImage.ptr<uchar>(0)[i];
		leftUp += BinaryImage.ptr<uchar>(1)[i];
		leftDown += BinaryImage.ptr<uchar>(nc - 1)[i];
		leftDown += BinaryImage.ptr<uchar>(nc - 2)[i];

		rightUp += BinaryImage.ptr<uchar>(0)[nl-i-1];
		rightUp += BinaryImage.ptr<uchar>(1)[nl-i-1];
		rightDown += BinaryImage.ptr<uchar>(nc - 1)[nl-i-1];
		rightDown += BinaryImage.ptr<uchar>(nc - 2)[nl-i-1];
	}
	bool leftUpFlag = false;
	if (leftUp > CountNum )
	{
		leftUpFlag = true;
	}
	bool leftDownFlag = false;
	if (leftDown > CountNum)
	{
		leftDownFlag = true;
	}
	bool rightUpFlag = false;
	if (rightUp > CountNum)
	{
		rightUpFlag = true;
	}
	bool rightDownFlag = false;
	if (rightDown>CountNum)
	{
		rightDownFlag = true;
	}
	/*
	bool leftflag = true;
	if ((BinaryImage.ptr<uchar>(0)[0] > 200 && BinaryImage.ptr<uchar>(0)[1] > 200)\
		|| (BinaryImage.ptr<uchar>(BinaryImage.rows - 1)[0] > 200 && BinaryImage.ptr<uchar>(BinaryImage.rows - 1)[1] > 200))
	{
		leftflag = false;
	}
	bool rightflag = true;
	if ((BinaryImage.ptr<uchar>(0)[BinaryImage.cols - 1] > 200 && BinaryImage.ptr<uchar>(0)[BinaryImage.cols - 2] > 200) \
		|| (BinaryImage.ptr<uchar>(BinaryImage.rows - 1)[BinaryImage.cols - 1] > 200 && BinaryImage.ptr<uchar>(BinaryImage.rows - 1)[BinaryImage.cols - 2] > 200))
	{
		rightflag = false;
	}*/
	for (int i = 0; i < nc; i++)
	{ 
		uchar* ptr_c = BinaryImage.ptr<uchar>(i);
		bool TleftUpFlag = leftUpFlag;
		bool TleftDownFlag = leftDownFlag;
		for (int j = 0; j < nl; j++)  //从左到右
		{
			if (ptr_c[j]>200)
			{
				ptr_c[j] = 0;
			}
			else 
			{
				if (i < midnumR2 && TleftUpFlag )
				{
					if ( j > midnumC / 2)
					TleftUpFlag = false;
				}
				else if (i > nc - midnumR2 && TleftDownFlag)
				{
					if (j > midnumC / 2)
					TleftDownFlag = false;
				}
				else
				{
					j = nl;
				}
			}
			if (((i>midnumR1  && i<midnumR2) || (i>nc - midnumR2 && i < nc - midnumR1)) && j>midnumC)//两侧
			{
				j = nl;
			}
			else if (i>midnumR2  && i < nc - midnumR2 && j>midnumC/2)    //最中间部分
			{
				j = nl;			
			}
		}
		bool TrightUpFlag = rightUpFlag;
		bool TrightDownFlag = rightDownFlag;
		for (int j = nl-1; j >=0; j--)  //从右向左
		{
			if (ptr_c[j]>200)
			{
				ptr_c[j] = 0;
			}
			else
			{
				if (i < midnumR2 && TrightUpFlag)
				{
					if (j < nl - midnumC/2 )
						TrightUpFlag = false;
				}
				else if (i>nc - midnumR2 && TrightDownFlag )
				{
					if (j<nl - midnumC / 2)
					TrightDownFlag = false;
				}
				else
				{
					j = 0;
				}
			}
			if (((i>midnumR1  && i<midnumR2) || (i>nc - midnumR2 && i < nc - midnumR1)) && j<nl - midnumC)
			{
				j = 0;
			}
			else if (i > midnumR2  && i < nc - midnumR2 && j>nl - midnumC / 2)
			{
				j = 0;
			}
		}
	}
	
	return 0 ;
}

int X1FRecognition::EliminateUpDownDisturb(const int threadValue, Mat& BinaryImage)   //输入是二值图，消除边界干扰
{
	assert(BinaryImage.depth() != sizeof(uchar));
	assert(BinaryImage.data);

	int nc = BinaryImage.rows;
	int nl = BinaryImage.cols;
	int upValue = nc / 9;
	int downValue = nc - upValue;
	for (int j = 0; j < nl; j++)
	{
		for (int i = 0; i < upValue; i++)
		{
			uchar* ptr_cu = BinaryImage.ptr<uchar>(i);
			if (ptr_cu[j] > threadValue - 5)
			{
				ptr_cu[j] = 0;
			}
			else
			{
				i = upValue;
			}

		}
		for (int i = nc - 1; i > downValue; i--)
		{
			uchar* ptr_cd = BinaryImage.ptr<uchar>(i);
			if (ptr_cd[j] > threadValue - 5)
			{
				ptr_cd[j] = 0;
			}
			else
			{
				i = downValue;
			}
		}
	}
	return 0;
}

bool X1FRecognition::ellipse_calc_bin_imgMat(const Mat img_cor, Mat & img_bin, const int meanThreshold)  //选定区域自动二值化，边界扩大
{
	if (img_cor.rows != 128 && 64 != img_cor.cols && img_bin.rows != 128 && img_bin.cols != 64)
		return false;
	int i, j, threshold_numR = 8, threshold_numC = 4;
	int stepR = img_cor.rows / threshold_numR, stepC = img_cor.cols / threshold_numC;
	int maxThreshold = meanThreshold / 4 + meanThreshold;
	int minThreshold = meanThreshold - meanThreshold / 5;
	for (i = 0; i < threshold_numR; i++)
	{
		for (j = 0; j < threshold_numC; j++)
		{
			Mat imgeRoi = img_cor(Rect(j*stepC, i*stepR, stepC, stepR));
			Mat imgeRoidest = img_bin(Rect(j*stepC, i*stepR, stepC, stepR));
			int m_nThreshold = getThreshVal_optic(imgeRoi);
			if (j == 0 || j == (threshold_numC - 1) || i == 0 || i == (threshold_numR - 1))
			{
				m_nThreshold = meanThreshold - 8;
			}
			else
			{
				if (m_nThreshold > maxThreshold)
				{
					m_nThreshold = maxThreshold;
				}
				else if (m_nThreshold < minThreshold)
				{
					m_nThreshold = minThreshold;
				}
			}
			threshold(imgeRoi, imgeRoidest, m_nThreshold, 255, CV_THRESH_BINARY_INV);
		}
	}

	return true;
}


struct CardContourInf
{
	int UpContourVnum;  //上方垂直的轮廓数，例如二条，上方下方个一个
	int MidContourVnum;
	int DownContourVnum;
	int ContourPnum;  //上方平行的轮廓数，例如一万，上方一个
	int AllContour;
}CardContourNum = { 0, 0, 0, 0, 0 };

// 条的识别，条的个数
//一二万横向个数
int FindContourSureNum(const Mat& BinaryImg)  //通过轮廓可以识别出各种条
{
	if (BinaryImg.empty() || BinaryImg.type() != CV_8UC1) return -1;
	std::vector<vector<Point> > contour;
	std::vector<Vec4i> hierarchy;
	memset(&CardContourNum, 0, sizeof(struct CardContourInf));
	int wthred = BinaryImg.cols;
	int heithred = BinaryImg.rows;
	findContours(BinaryImg, contour, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //输入必须是二值单通道图，灰度图不是
	const double minPix = (heithred / wthred) * 4;
	const int minHeight = heithred / 11;
	Mat imgDst = Mat::zeros(BinaryImg.size(), CV_8U);
	std::vector<double> AreaCount;
	std::vector<Rect> ContourLocation;
	for (int i = 0; i < contour.size(); i++)
	{
		double fTemp = fabs(contourArea(contour[i]));
		if (fTemp < minPix)
		{
			//contour.erase(contour.begin() + i);
			//i--;
			continue;
		}
		Rect loc = boundingRect(contour[i]);
		if (loc.height < minHeight && loc.width < minHeight / 3)
		{
			continue;
		}
		AreaCount.push_back(fTemp);
		ContourLocation.push_back(loc);
		drawContours(imgDst, contour, i, Scalar(255), CV_FILLED);
		//imshow("imgDst", imgDst);
		//waitKey(0);
	}
	//drawContours(BinaryImg, contour, -1, Scalar(255), CV_FILLED);  //还原图像，用findcontour后会改变图像内容

	int n = AreaCount.size();
	double MaxPixV = 0, MaxPixP = 0;
	for (int i = 0; i < n; i++)  //求最大轮廓
	{
		if (ContourLocation[i].width < ContourLocation[i].height)
		{
			if (MaxPixV < AreaCount[i])
			{
				MaxPixV = AreaCount[i];
			}
		}
		else
		{
			if (MaxPixP < AreaCount[i])
			{
				MaxPixP = AreaCount[i];
			}
		}
	}
	int addVFlag = 0;
	int eliminateMark = 0;
	int subnum = 1;
	for (int i = 0; i < n; i++)
	{
		if (ContourLocation[i].width < ContourLocation[i].height && (AreaCount[i]>MaxPixV / 5 || AreaCount[i] > wthred))
		{

			if (CardContourNum.DownContourVnum == 0)
			{
				CardContourNum.DownContourVnum += 1;
				continue;
			}
			int minheight = min(ContourLocation[i - subnum].height, ContourLocation[i].height);
			if (abs(ContourLocation[i - subnum].x - ContourLocation[i].x)>ContourLocation[i - subnum].width*0.8&&\
				abs(ContourLocation[i - subnum].y - ContourLocation[i].y)>minheight*0.3)
			{
				++addVFlag;
			}
			else if (abs(ContourLocation[i - subnum].y - ContourLocation[i].y)>minheight*0.8)
			{
				++addVFlag;
			}
			if (addVFlag == 0)
			{
				CardContourNum.DownContourVnum += 1;
			}
			else if (addVFlag == 1)
			{
				CardContourNum.MidContourVnum += 1;
			}
			else
			{
				CardContourNum.UpContourVnum += 1;
			}
			//粘连问题，让计算条更准确
			if (ContourLocation[i - subnum].height > ContourLocation[i].height && AreaCount[i] > wthred)
			{
				if (ContourLocation[i - subnum].height > minheight*1.8 && (ContourLocation[i - subnum].y - ContourLocation[i].y)< minheight / 2)
				{
					if (i - eliminateMark != 1)
					{
						if (addVFlag == 1)
						{
							CardContourNum.DownContourVnum += 1;
						}
						else if (addVFlag == 2)
						{
							CardContourNum.MidContourVnum += 1;
						}
						else
						{
							CardContourNum.UpContourVnum += 1;
						}
					}
					eliminateMark = i;
				}
			}
			else if (ContourLocation[i - subnum].height < ContourLocation[i].height && AreaCount[i - subnum] > wthred)
			{
				if (ContourLocation[i].height > minheight*1.8 && (ContourLocation[i].y - ContourLocation[i - subnum].y)< minheight / 2)
				{
					if (i - eliminateMark != 1)
					{
						if (addVFlag == 1)
						{
							CardContourNum.DownContourVnum += 1;
						}
						else if (addVFlag == 2)
						{
							CardContourNum.MidContourVnum += 1;
						}
						else
						{
							CardContourNum.UpContourVnum += 1;
						}
					}
					eliminateMark = i;
				}
			}
			subnum = 1;
		}
		else if (ContourLocation[i].width > ContourLocation[i].height && AreaCount[i]>MaxPixP / 2)
		{
			subnum++;
			CardContourNum.ContourPnum += 1;
		}
		else
		{
			subnum++;
		}
	}
	CardContourNum.AllContour = n;
	AreaCount.clear();
	std::vector<double>(AreaCount).swap(AreaCount);
	ContourLocation.clear();
	std::vector<Rect>(ContourLocation).swap(ContourLocation);
	contour.clear();
	std::vector<vector<Point> >(contour).swap(contour);
	hierarchy.clear();
	std::vector<Vec4i>(hierarchy).swap(hierarchy);
	return 0;
}

static float getValidValue(Mat pic, const int maxValue)
{
    int i, j, value = 0;
    int step = pic.rows / 11;
    for (i = 0; i < pic.rows; i++)
    {
        const uchar* src = pic.data + pic.step*i;
        for (j = 0; j < pic.cols; j++)
            if (src[j] < maxValue)
            {
            value++;
            }
    }

    float ratio = float(value) / float(pic.rows*pic.cols);
    return ratio;
}


bool X1FRecognition::HogFeatureOfOneImageForSVM(const Mat& imgSrc, Mat &outimg)
{
	
	Mat imgBin(imgSrc.rows, imgSrc.cols, imgSrc.depth());
	if (m_bThresholdZip)   //已经二值化就不用二值化了
	{
		imgSrc.copyTo(imgBin);
	}
	else
	{
		
		Rect AUTObin(imgSrc.cols / 8, imgSrc.rows / 8, imgSrc.cols - imgSrc.cols / 4, imgSrc.rows - imgSrc.rows / 4);
		Mat AutoOptic = imgSrc(AUTObin);
		bool deleateNoiseFlag = false;
		m_nThreshold = getThreshVal_optic(AutoOptic);
		if (m_nThreshold > 108)  //二值阈值足够了
		{
            float thredvalue = getValidValue(AutoOptic, 60);   // 设置有效必须小于60
            if (thredvalue < 0.05)
            {
                m_nThreshold = m_nThreshold*0.5;
            }

		}
		else if (m_nThreshold < 15)
		{
			m_nThreshold = 15;
		}
	
		if (!ellipse_calc_bin_imgMat(imgSrc, imgBin, m_nThreshold))
		{  
			//GaussianBlur(imgBin, imgBin, Size(3, 3), 0, 0);  // 滤波
			threshold(imgSrc, imgBin, m_nThreshold, 255, CV_THRESH_BINARY_INV); //m_nThreshold 65左右
		}
	}
	EliminateBoundaryDisturb(imgBin);

	for (int i = 0; i < imgBin.cols; i++)
	{
		imgBin.ptr<uchar>(0)[i] = 0;
		//HoleBinaryImage.ptr<uchar>(1)[i] = 0;
		//HoleBinaryImage.ptr<uchar>(HoleBinaryImage.rows - 2)[i] = 0;
		imgBin.ptr<uchar>(imgBin.rows - 1)[i] = 0;
	}
	/***************/
	Mat tempImage = imgBin.clone();
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3)); //形态学核函数
	morphologyEx(tempImage, tempImage, MORPH_OPEN, element); //闭运算使轮廓尽量完整
	Mat dest;
	subtract(imgBin, tempImage, dest);
	int nc = dest.rows;
	int nl = dest.cols;
	Rect roc(nl / 6, nc / 12, nl - nl / 3, nc - nc / 6);
	Mat mask = Mat::zeros(dest.size(), CV_8UC1);
	mask(roc).setTo(255);
	Mat tempImage2;
	dest.copyTo(tempImage2);
	tempImage2.setTo(0, mask);
	Mat lastTestImage;
	subtract(imgBin, tempImage2, lastTestImage);
	//Mat temp = lastTestImage;
	Mat ImageSelect = FindContours(lastTestImage, 0.002*lastTestImage.cols*lastTestImage.rows, 0.98*lastTestImage.cols*lastTestImage.rows);   //如果已经小区域去除处理了，就可以不用处理，细小东西被删掉容易导致识别错误
	Mat testImge = preprocessing(ImageSelect, 0, 0);
	cv::Mat imForHog;
	if (testImge.rows != 128 || testImge.cols != 64)
	{
		cv::resize(testImge, imForHog, cv::Size(64, 128), 0, 0, CV_INTER_LINEAR);
	}
	else
	{
		imForHog = testImge;
	}

	descriptors.clear();
	outimg = imForHog;
	if (savebinery2C)
	{
		IplImage plImageP = IplImage(imForHog);
		CString ax, side;                  //二值图像保存
		side = _T("D:\\Allimg5mmbin\\");
		if (!PathIsDirectory(side))
		{
			::CreateDirectory(side, NULL);
		}
		ax.Format(_T("%d.bmp"), nub);
		ax = side + ax;
		string ss = CT2A(ax.GetBuffer());
		ax.ReleaseBuffer();
		nub++;
		//imwrite(ss, imForHog);         //保存
		cvSaveImage(ss.c_str(), &plImageP);
	}
	m_pHog->compute(imForHog, descriptors, cvSize(16, 16), cv::Size(0, 0));
	TagLableBrand(imForHog);
	Mat findsureNum = imForHog.clone();
	FindContourSureNum(findsureNum);  //通过轮廓可以识别出各种条  //会改变图片内容
	return true;
}
/*********/
Mat X1FRecognition::FindContours(const Mat& imgSrc, float minPix, float maxPix)
{
	std::vector<vector<Point> > contour;
	std::vector<Vec4i> hierarchy;
	if (imgSrc.empty() || imgSrc.type() != CV_8UC1)
	{
		return imgSrc;
	}
	int wthred = imgSrc.cols / 12;
	int heithred = imgSrc.rows / 14;
	findContours(imgSrc, contour, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //输入必须是二值单通道图，灰度图不是
	Mat imgDst = Mat::zeros(imgSrc.size(), CV_8U);

	for (int i = 0; i < contour.size(); i++) //删除过大或过小的轮廓
	{
		double fTemp = fabs(contourArea(contour[i]));
		////Rect loc = boundingRect(contour[i]);
		//if ((fTemp < minPix || fTemp > maxPix) && (loc.y < heithred || loc.y >(imgSrc.rows - heithred)))
		//{
		//	contour.erase(contour.begin() + i);
		//	continue;
		//}
		//else if ((fTemp < minPix || fTemp > maxPix) && (loc.x<wthred || (loc.x>imgSrc.cols - wthred)))
		//{
		//	contour.erase(contour.begin() + i);
		//	//i--;
		//	continue;
		//}
		if ((fTemp < minPix || fTemp > maxPix))
		{
			contour.erase(contour.begin() + i);
			i--;
			continue;
		}
		else if ((fTemp < minPix || fTemp > maxPix))
		{
			contour.erase(contour.begin() + i);
			i--;
			continue;
		}
		
	}
	drawContours(imgDst, contour, -1, Scalar(255), CV_FILLED);
	contour.clear();
	std::vector<vector<Point> >(contour).swap(contour);
	hierarchy.clear();
	std::vector<Vec4i>(hierarchy).swap(hierarchy);
	return imgDst;
}
int X1FRecognition::SecondPredict(int response)
{
	switch (response)
	{
		/*****
		case 2:
		if (crossdetect1.charcter1)
		response = 1;
		else if (crossdetect1.charcter3)
		response = 3;
		if (crossdetect1.Giveupcharcter)
		response = 2;
		break;
		case 3:
		if (crossdetect1.charcter2)
		response = 2;
		if (crossdetect1.Giveupcharcter)
		response = 3;
		break;
		case 44:
		if (crossdetect1.charcter1)
		response = 43;
		else if (crossdetect1.charcter3)
		response = 45;
		if (crossdetect1.Giveupcharcter)
		response = 44;
		break;
		case 45://三万倒猜二万倒
		if (crossdetect1.charcter2)
		response = 44;
		if (crossdetect1.Giveupcharcter)
		response = 45;
		break;
		/*****/
	case 32:  //红中猜二条
		if (crossdetect1.R_AreaNum1 != 1 && crossdetect1.C_Width != 10)
			response = 20;
		break;
	case 61:  //红中猜二条
		if (crossdetect1.R_AreaNum1 != 1 && crossdetect1.C_Width != 10)
			response = 20;
		break;
	case 17://八筒猜六筒
		if (crossdetect1.R_AreaNum1 != 1)
			response = 15;
		break;
	case 15: //六筒猜八筒、四筒
		if (crossdetect1.R_AreaNum1)
			response = 17;
		else if (crossdetect1.C_AreaNum2)
			response = 13;
		break;
	case 52: //六筒猜八筒、四筒
		if (crossdetect1.R_AreaNum1)
			response = 17;
		else if (crossdetect1.C_AreaNum2)
			response = 13;
		break;
	case 20://二条猜七条
		if (crossdetect1.C_AreaNum3&& CardContourNum.DownContourVnum>2)
		{
			//GuessFlag = true;
			response = 25;
		}
		else if (crossdetect1.C_Width == 10 && CardContourNum.AllContour < 2)  // 二条猜红中
		{
			//GuessFlag = true;
			response = 32;
		}
		break;
	case 24:  //六条猜四条
		//六条猜九条
		if (CardContourNum.UpContourVnum == 3 && CardContourNum.MidContourVnum * 3 <= CardContourNum.AllContour)
		{
			//GuessFlag = true;
			response = 27;
		}
		else if (CardContourNum.UpContourVnum == 1)  //六条猜七条
		{
			//GuessFlag = true;
			response = 25;
		}
		else if (crossdetect1.C_AreaNum2&&crossdetect1.R_AreaNum2&&crossdetect1.R_AreaNum3&&crossdetect1.R_AreaNum1)//六条猜四条
		{
			//GuessFlag = true;
			response = 22;
		}
		break;
		/*********/
	case 22:  //四条猜八筒,六条
		if (crossdetect1.C_AreaNum1&&crossdetect1.R_AreaNum1 && (crossdetect1.C_MidEmptyMid || crossdetect1.R_MidEmptyMid))
		{
			//GuessFlag = true;
			response = 17;
		}
		else if (crossdetect1.C_AreaNum3)
		{
			//GuessFlag = true;
			response = 24;
		}
		break;
	case 27: //九条猜六条和四条
		if (CardContourNum.UpContourVnum == 0 && (CardContourNum.DownContourVnum + CardContourNum.MidContourVnum + 1) >= CardContourNum.AllContour)
		{
			//GuessFlag = true;
			response = 24;
		}
		break;
	case 34: // 白板猜四条
		if (CardContourNum.DownContourVnum >= 2 && CardContourNum.MidContourVnum >= 2)
		{
			//GuessFlag = true;
			response = 22;
		}
		break;
	default:
		break;
	}
	return response;
}
void  X1FRecognition::Seed_Filling(const Mat& binImg, Mat& lableImg, int &num)   //种子填充法
{
	// 4邻接方法
	//8邻接更精确，目前还用不着
	if (binImg.empty() || binImg.type() != CV_8UC1)
	{
		return;
	}
	threshold(binImg, binImg, 80, 1, CV_THRESH_BINARY);
	lableImg.release();
	binImg.convertTo(lableImg, CV_32SC1);
	int label = 1;
	int rows = binImg.rows - 1;
	int cols = binImg.cols - 1;
#if 1
	for (int i = 1; i < rows; i++)
	{
		int* data = lableImg.ptr<int>(i);
		for (int j = 1; j < cols; j++)
		{
			if (data[j] == 1)  //选定区域阈值
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(i, j));     // 像素位置: <i,j>
				++label;  // 没有重复的团，开始新的标签
				while (!neighborPixels.empty())
				{
					std::pair<int, int> curPixel = neighborPixels.top(); //如果与上一行中一个团有重合区域，则将上一行的那个团的标号赋给它
					int curX = curPixel.first;
					int curY = curPixel.second;
					lableImg.at<int>(curX, curY) = label;
					neighborPixels.pop();
					if (curX>1 && curY>1 && curX<(rows - 1) && curY<(cols - 1))//边界判断,避免出界
					{
						if (lableImg.at<int>(curX, curY - 1) == 1)
						{//左边
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (lableImg.at<int>(curX, curY + 1) == 1)
						{// 右边
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (lableImg.at<int>(curX - 1, curY) == 1)
						{// 上边
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (lableImg.at<int>(curX + 1, curY) == 1)
						{// 下边
							neighborPixels.push(std::pair<int, int>(curX + 1, curY));
						}
					}
				}
			}
		}
	}
#else
	for (int j = 1; j < cols; j++)
	{
		for (int i = 1; i < rows; i++)
		{
			int* data = lableImg.ptr<int>(i);
			if (data[j] == 1)  //选定区域阈值
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(i, j));     // 像素位置: <i,j>
				++label;  // 没有重复的团，开始新的标签
				while (!neighborPixels.empty())
				{
					std::pair<int, int> curPixel = neighborPixels.top(); //如果与上一行中一个团有重合区域，则将上一行的那个团的标号赋给它
					int curX = curPixel.first;
					int curY = curPixel.second;
					lableImg.at<int>(curX, curY) = label;
					neighborPixels.pop();
					if (curX>1 && curY>1 && curX<(rows - 1) && curY<(cols - 1))//边界判断,避免出界
					{
						if (lableImg.at<int>(curX, curY - 1) == 1)
						{//左边
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (lableImg.at<int>(curX, curY + 1) == 1)
						{// 右边
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (lableImg.at<int>(curX - 1, curY) == 1)
						{// 上边
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (lableImg.at<int>(curX + 1, curY) == 1)
						{// 下边
							neighborPixels.push(std::pair<int, int>(curX + 1, curY));
						}
					}
				}
			}
		}
	}
#endif
	num = label;

}
int X1FRecognition::Label_connect(const Mat& labelImg, int &num, int &countnum, int &heightLabel, int &realSecheight, bool &addlabel)
{
	Mat outLabelImg;
	if (labelImg.empty() ||
		labelImg.type() != CV_32SC1)
	{
		return 0;
	}
	int rows = labelImg.rows;
	int cols = labelImg.cols;
	outLabelImg.release();
	outLabelImg.create(rows, cols, CV_8UC1);
	outLabelImg = cv::Scalar::all(0); //清零
	countnum = 0;
	int maxlinenum = 0; //用于返回单行最大
	bool closelabel = true;
	int lastinum = 0;
	int lasti = 0;
	int closelasti = 0;
	bool startsec = false;
	bool reallabel = false;
	int nextlabel = 0;
	int littlelabel = 255;
	for (int i = 0; i < rows; i++)
	{
		int  maxlinelabel = 0;
		const int* data_src = (int*)labelImg.ptr<int>(i);
		uchar* data_dst = outLabelImg.ptr<uchar>(i);
		for (int j = 0; j < cols; j++)
		{
			int pixelValue = data_src[j];
			if (num == pixelValue)//指定连通域，其他隐藏
			{
				if (maxlinelabel > maxlinenum)
				{
					maxlinenum = maxlinelabel;
				}
				maxlinelabel++;
				if (closelabel)
				{
					closelabel = false;
					realSecheight = i;
				}
				*data_dst++ = 255;
				countnum++;
				heightLabel = i;

			}
			else
			{
				data_dst++;
			}
		}
		if (maxlinelabel > 0)
		{
			if ((maxlinelabel < maxlinenum) && closelasti<2)
			{
				closelasti++;
				lastinum = maxlinenum;
				if (closelasti == 2)
				{
					startsec = true;
				}
			}
			if (startsec && maxlinelabel<nextlabel)
			{
				if (maxlinelabel < littlelabel)
				{
					littlelabel = maxlinelabel;

				}
			}

			if (startsec && (maxlinelabel>nextlabel))
			{
				if (maxlinelabel > lasti)
				{
					lasti = maxlinelabel;
					reallabel = true;
				}

			}
			nextlabel = maxlinelabel;
		}
	}
	if ((lastinum - lasti < 8) && reallabel && (littlelabel< lasti / 3))
	{
		addlabel = true;

	}
	return maxlinenum;
}



void X1FRecognition::read_params_set_CH()
{
	CvSVMParams _params;
	_params = params;
	set_params(_params);
}

void X1FRecognition::read_CH(CvFileStorage* fs, CvFileNode* svm_node)
{
	const double not_found_dbl = DBL_MAX;
	CV_FUNCNAME("CvSVM::read");
	int i, var_count, df_count, class_count;
	int block_size = 1 << 16, sv_size;
	CvFileNode *sv_node, *df_node;
	CvSVMDecisionFunc* df;
	CvSeqReader reader;
	clear();
	read_params_set_CH();
	sv_total = 1;
	var_all = 3780;
	var_count = 3780;
	class_count = 2;
	short datalabe[] = { -1, 1 }; //标签
	CvMat* class_labelsRes = cvCreateMat(1, 2, CV_32SC1);
	CV_MAT_ELEM(*class_labelsRes, short, 0, 0) = -1;
	CV_MAT_ELEM(*class_labelsRes, short, 0, 1) = 1;
	CV_MAT_ELEM(*class_labelsRes, int, 0, 0) = -1;
	CV_MAT_ELEM(*class_labelsRes, int, 0, 1) = 1;
	CV_MAT_ELEM(*class_labelsRes, double, 0, 0) = -1;
	CV_MAT_ELEM(*class_labelsRes, double, 0, 1) = 1;
	//class_labels = (CvMat*)cvReadByName(fs, svm_node, "class_labels");
	class_labels = class_labelsRes;
	//class_weights = (CvMat*)cvReadByName(fs, svm_node, "class_weights");
	//var_idx = (CvMat*)cvReadByName(fs, svm_node, "var_idx");
	// read support vectors
	sv_node = cvGetFileNodeByName(fs, svm_node, "Matrices");
	block_size = MAX(block_size, sv_total*(int)sizeof(CvSVMKernelRow));
	block_size = MAX(block_size, sv_total * 2 * (int)sizeof(double));
	block_size = MAX(block_size, var_all*(int)sizeof(double));
	storage = cvCreateMemStorage(block_size + sizeof(CvMemBlock) + sizeof(CvSeqBlock));
	sv = (float**)cvMemStorageAlloc(storage, sv_total*sizeof(sv[0]));

	cvStartReadSeq(sv_node->data.seq, &reader, 0);
	sv_size = var_count*sizeof(sv[0][0]);
	for (i = 0; i < sv_total; i++)
	{
		CvFileNode* sv_elem = (CvFileNode*)reader.ptr;
		sv[i] = (float*)cvMemStorageAlloc(storage, sv_size);
		cvReadRawData(fs, sv_elem, sv[i], "f");
		CV_NEXT_SEQ_ELEM(sv_node->data.seq->elem_size, reader);
	}
	// read decision functions
	df_count = class_count > 1 ? class_count*(class_count - 1) / 2 : 1;
	df_node = cvGetFileNodeByName(fs, svm_node, "decifunc");
	df = decision_func = (CvSVMDecisionFunc*)cvAlloc(df_count*sizeof(df[0]));
	cvStartReadSeq(df_node->data.seq, &reader, 0);

	for (i = 0; i < df_count; i++)
	{
		CvFileNode* df_elem = (CvFileNode*)reader.ptr;
		CvFileNode* alpha_node = cvGetFileNodeByName(fs, df_elem, "alkfa");
		int sv_count = 1;
		df[i].sv_count = sv_count;
		df[i].rho = cvReadRealByName(fs, df_elem, "ptrho", not_found_dbl);
		df[i].alpha = (double*)cvMemStorageAlloc(storage,
			sv_count*sizeof(df[i].alpha[0]));
		cvReadRawData(fs, alpha_node, df[i].alpha, "d");
		if (class_count > 1)
		{
			CvFileNode* index_node = cvGetFileNodeByName(fs, df_elem, "index");
			df[i].sv_index = (int*)cvMemStorageAlloc(storage,
				sv_count*sizeof(df[i].sv_index[0]));
			cvReadRawData(fs, index_node, df[i].sv_index, "i");
		}
		else
			df[i].sv_index = 0;

		CV_NEXT_SEQ_ELEM(df_node->data.seq->elem_size, reader);
	}
	if (cvReadIntByName(fs, svm_node, "optimize_linear", 1) != 0)
		optimize_linear_svm();
	create_kernel();
}


bool X1FRecognition::RealReadLocale(const int SpecialNum, const int CardNum, const int SelectCard)
{
	//const double not_found_dbl = DBL_MAX;
	//CV_FUNCNAME("CvSVM::read");
	int i, var_count, df_count, class_count;
	int block_size = 1 << 16, sv_size;
	//CvFileNode *sv_node;
	//CvSVMDecisionFunc* df;
	//CvSeqReader reader;
	clear();
	set_params(params);
	sv_total = 1;
	var_all = 3780;
	var_count = 3780;
	class_count = 2;
	short datalabe[] = { -1, 1 }; //标签
	CvMat* class_labelsRes = cvCreateMat(1, 2, CV_32SC1);
	CV_MAT_ELEM(*class_labelsRes, short, 0, 0) = -1;
	CV_MAT_ELEM(*class_labelsRes, short, 0, 1) = 1;
	CV_MAT_ELEM(*class_labelsRes, int, 0, 0) = -1;
	CV_MAT_ELEM(*class_labelsRes, int, 0, 1) = 1;
	CV_MAT_ELEM(*class_labelsRes, double, 0, 0) = -1;
	CV_MAT_ELEM(*class_labelsRes, double, 0, 1) = 1;
	class_labels = class_labelsRes;

	block_size = MAX(block_size, sv_total*(int)sizeof(CvSVMKernelRow));
	block_size = MAX(block_size, sv_total * 2 * (int)sizeof(double));
	block_size = MAX(block_size, var_all*(int)sizeof(double));
	storage = cvCreateMemStorage(block_size + sizeof(CvMemBlock) + sizeof(CvSeqBlock));
	sv = (float**)cvMemStorageAlloc(storage, sv_total*sizeof(sv[0]));

	sv_size = var_count*sizeof(sv[0][0]);
	sv[0] = (float*)cvMemStorageAlloc(storage, sv_size);
	float* tempdate = NULL;
	
	tempdate = (float*)SelectmatrixAtemp(CardNum + SelectCard*SpecialNum, 1);
	
	if (tempdate[0] == MAXNUMTEST)
		return false;
	for (i = 0; i < var_count; i++)
	{
		sv[0][i] = tempdate[i];
	}
	int sv_count = 1;
	df_count = class_count > 1 ? class_count*(class_count - 1) / 2 : 1;
	decision_func = (CvSVMDecisionFunc*)cvAlloc(df_count*sizeof(CvSVMDecisionFunc));
	decision_func[0].sv_count = 1;
	double* rhotempdate = NULL;

	
	rhotempdate = (double*)SelectmatrixAtemp(CardNum + SelectCard*SpecialNum, 2);

	decision_func[0].rho = rhotempdate[0];
	decision_func[0].alpha = (double*)cvMemStorageAlloc(storage, sv_count*sizeof(double));
	decision_func[0].alpha[0] = 1.00000000000000000;
	decision_func[0].sv_index = (int*)cvMemStorageAlloc(storage, sv_count*sizeof(int));
	decision_func[0].sv_index[0] = 0;
	optimize_linear_svm();
	create_kernel();
	return true;

}

//gamma矫正
void MyGammaCorrection(Mat& src, Mat& dst, float fGamma)
{
    if (src.empty() || src.depth() == sizeof(uchar))
    {
        return;
    }
    //CV_Assert(src.data);

    // accept only char type matrices
    //CV_Assert(src.depth() != sizeof(uchar));

    // build look up table
    unsigned char lut[256];
    for (int i = 0; i < 256; i++)
    {
        lut[i] = saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
    }

    dst = src.clone();
    const int channels = dst.channels();
    switch (channels)
    {
    case 1:
    {
        MatIterator_<uchar> it, end;
        for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
            //*it = pow((float)(((*it))/255.0), fGamma) * 255.0;
            *it = lut[(*it)];

        break;
    }
    case 3:
    {
        MatIterator_<Vec3b> it, end;
        for (it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++)
        {
            //(*it)[0] = pow((float)(((*it)[0])/255.0), fGamma) * 255.0;
            //(*it)[1] = pow((float)(((*it)[1])/255.0), fGamma) * 255.0;
            //(*it)[2] = pow((float)(((*it)[2])/255.0), fGamma) * 255.0;
            (*it)[0] = lut[((*it)[0])];
            (*it)[1] = lut[((*it)[1])];
            (*it)[2] = lut[((*it)[2])];
        }

        break;

    }
    }
}
bool imageAggrandize(Mat & srcImg, const int mini, const int maxreach)
{
    if (srcImg.empty())
        return false;
    int nwidth = srcImg.cols;
    int nheight = srcImg.rows;
    unsigned int allcount = 1;
    unsigned int allNum = 0;
    for (int j = 0; j < nheight; ++j)
    {
        uchar* ptr_c = srcImg.ptr<uchar>(j);
        for (int i = 0; i <nwidth; ++i)
        {
            if (ptr_c[i]>mini)
            {
                allcount += ptr_c[i];
                ++allNum;
            }
        }
    }
    if (allNum == 0)return false;
    float meanValue = (float)allcount / (float)allNum;

    float ratio = maxreach / meanValue;
    int midtemp = 125;
    for (int j = 0; j < nheight; ++j)
    {
        uchar* ptr_m = srcImg.ptr<uchar>(j);

        for (int i = 0; i < nwidth; ++i)
        {
            if (ptr_m[i] > mini)
            {
                midtemp = ptr_m[i] * ratio;
                if (midtemp > 255)
                {
                    ptr_m[i] = 255;
                }
                else
                {
                    ptr_m[i] = midtemp;
                }

            }
        }
    }
    return true;
}


int X1FRecognition::StartRecon(std::vector<Mat>& ImgArr)
{
	Mat& imgSrc = ImgArr[0];
	//测试
	//AfxMessageBox(_T("it is 5mm"));
	//int erwanorsanwanflag;
	if (imgSrc.empty() || imgSrc.type() != CV_8UC1)
	{
		return -1;
	}

#if 0
	int temp1 = imgSrc.ptr<uchar>(0)[0];   // + imgSrc.ptr<uchar>(0)[1];
	if (temp1 > 120)
	{
		erwanorsanwanflag = 3;
	}
	else
	{
		erwanorsanwanflag = 2;
	}
#endif
	//int erwanorsanwanflg = Image.ptr<uchar>(1)[1];
	//int erwanorsanwanlag = Image.ptr<uchar>(2)[1];
	transpose(imgSrc, imgSrc); //图片旋转90度
	flip(imgSrc, imgSrc, 1); //图片旋转90度	
	Mat outImg;
	HogFeatureOfOneImageForSVM(imgSrc, outImg);
	Mat sampleFeatureMat = Mat::zeros(1, descriptors.size(), CV_32FC1);
	sampleFeatureMat = Mat(descriptors);
	int response = 0;
	int saveResponse = 0;
	float maxrr = 100;
	float result;
	int numxml = sizeof(xmlModelname) / sizeof(xmlModelname[0]);  //xml个数
	for (int j = 0; j < n_AllCardNum; j++)
	{
		for (int i = 1; i <numxml; i++)
		{
			//CvSVM svm;
			if (RealReadLocale(j, i, numxml))
			{
				result = predict(sampleFeatureMat, true);
			}
			else
			{
				continue;
			}
			//clear();
			if (result <= maxrr)  //判断距离最小的
			{
				maxrr = result;
				response = i + j*numxml;
			}
		}
	}
	saveResponse = response;
	response = response % numxml;   //待继续处理
	//free(ret);
	//ret = NULL;
	if (response == 52 && maxrr > -0.8 && (saveResponse<74 || saveResponse>222))  //六筒倒
	{
		response = SecondPredict(response);  //二次判断
	}
	else if (maxrr > 0 && (saveResponse < 74 || saveResponse>222))
	{
		response = SecondPredict(response);  //二次判断
	}

#if 0  //通过包络线切割判断大小来识别二万和三万，一万

	bool seywanlabel = false;
	bool falllabel = false;
	if (response == 38 || response == 39)
	{
		falllabel = true;
		seywanlabel = true;
	}
	else if (response == 2 || response == 3)
	{
		seywanlabel = true;
	}
	if (seywanlabel)
	{
		Mat borderinput = outImg.clone(); //深度拷贝
		resize(borderinput, borderinput, cv::Size(64, 128), 0, 0, CV_INTER_LINEAR);
		if (falllabel)
		{
			flip(borderinput, borderinput, 0); //图片旋转90度
			flip(borderinput, borderinput, 1); //图片旋转90度
		}
		int num = 1; //连通数
		Mat labelimg;
		Seed_Filling(borderinput, labelimg, num);
		vector <int> sectest;//有效点数
		vector<int>Secheight; //包罗位置
		vector<int>realSecheight;//高度
		vector<int>maxlnum;//单行最大数
		vector<bool>addhead;
		for (int i = 1; i <= num; i++)
		{
			int heightLabel = 0;
			int countnum = 0;
			int realheight = 0;
			int maxlinenum = 0;
			bool addlabel = false;
			maxlinenum = Label_connect(labelimg, i, countnum, heightLabel, realheight, addlabel);
			if (countnum < 8) //5个点，跳过//还可添加中心检测
				continue;
			//	if (heightLabel > ( borderinput.rows / 2))
			//		break;
			addhead.push_back(addlabel);
			maxlnum.push_back(maxlinenum);
			realSecheight.push_back(realheight);
			sectest.push_back(countnum);
			Secheight.push_back(heightLabel);
			//imshow("test3", outlastimg);
			//waitKey(0);
		}
		int colnum = sectest.size();
		if ((colnum >= 3) && (Secheight.at(2)<(borderinput.rows / 2)) && (addhead.at(2) != true))
		{
			int dottest = sectest.at(0) > sectest.at(1) ? sectest.at(0) : sectest.at(1);
			if ((sectest.at(2) >dottest + dottest / 3) && (maxlnum.at(2) - maxlnum.at(1)>maxlnum.at(1) / 3))  //二万猜三万
			{
				if (response == 38)
				{
					response = 39;
				}
				else if (response == 2)
				{
					response = 3;
				}
			}
		}
		else if ((colnum >1) && (Secheight.at(1)<(borderinput.rows / 2))) //三万猜二万//二万猜三万
		{
			if ((sectest.at(1) - sectest.at(0) > sectest.at(0) / 3) && (maxlnum.at(1) - maxlnum.at(0)>maxlnum.at(0) / 2))
			{
				if ((addhead.at(0) == true) || (addhead.at(1) == true))
				{
					if (response == 38)
					{
						response = 39;
					}
					else if (response == 2)
					{
						response = 3;
					}
				}
				/**
				else
				{
				if (response == 39)
				{
				response = 38;
				}
				else if (response == 3)
				{
				response = 2;
				}
				}
				/**/
			}
		}
		if ((maxlnum.at(0)>30) && (maxlnum.at(0) - maxlnum.at(1)>maxlnum.at(1) / 2))              //二万猜一万,maxlnum.at(0)>30一万长度
		{
			if (response == 38)
			{
				response = 37;
			}
			else if (response == 2)
			{
				response = 1;
			}
		}
		maxlnum.clear();
		sectest.clear();
		Secheight.clear();
		realSecheight.clear();
	}

#endif
#if 0
	if (2 == response || 38 == response || 3 == response || 39 == response)
	{
		CString strText = _T("文件名      识别结果         预测值      小机头判断");//strText为要写入txt文件的内容
		//如果是Unicode字符集，则写入中文的使用需要添加下面这一句，ASCII则不用.需要导入头文件locale.h
		char* pOldLocale = setlocale(LC_CTYPE, "chs");
		CString TxtFold_file;
		TxtFold_file.Format(_T("D:\\Allpic\\%d.txt"), nu - 1);
		CStdioFile fOutput(TxtFold_file, CFile::modeCreate | CFile::modeWrite | CFile::typeText);
		CString strTextM;
		strTextM.Format(_T("%f"), maxrr);
		CString realname;
		if (2 == response || 38 == response)
		{
			realname = _T("二万");
		}
		else if (3 == response || 39 == response)
		{
			realname = _T("三万");

		}
		CString numtxt;
		numtxt.Format(_T("%d"), nu - 1);
		CString temp1xdf;
		temp1xdf.Format(_T("%d"), temp1);
		if (fOutput)
		{
			fOutput.WriteString(strText);
			fOutput.WriteString(L"\n");
			fOutput.SeekToEnd();
			fOutput.WriteString(numtxt);
			fOutput.WriteString(L"         ");
			fOutput.SeekToEnd();
			//fOutput.WriteString(L"\n");
			fOutput.WriteString(realname);
			fOutput.SeekToEnd();
			fOutput.WriteString(L"        ");
			fOutput.SeekToEnd();
			fOutput.WriteString(strTextM);
			fOutput.WriteString(L"         ");
			fOutput.SeekToEnd();
			fOutput.WriteString(temp1xdf);
			fOutput.WriteString(L"\n");
			fOutput.SeekToEnd();
			fOutput.Close();
		}
		setlocale(LC_CTYPE, pOldLocale);

	}

	//通过小机判断二万和三万
	if (maxrr > -0.5)
	{
		if (38 == response && 3 == erwanorsanwanflag)
		{
			response = 39;
		}
		else if (39 == response && 2 == erwanorsanwanflag)
		{
			response = 38;
		}
		else if (2 == response && 3 == erwanorsanwanflag)
		{
			response = 3;
		}
		else if (3 == response && 2 == erwanorsanwanflag)
		{
			response = 2;
		}
	}

#endif
#if 1
	IplImage plImageP = IplImage(imgSrc);
	CString ax = L"";
	if (save2D)
	{

		// 临时保存输入图片
		//CString ax;
		if (selectbandflag == 1)
		{
			ax = L"D:\\Allpicx1H\\";
		}
		else
		{
			ax = L"D:\\Allpicx1f\\";
		}
		if (!PathIsDirectory(ax))
		{
			::CreateDirectory(ax, NULL);
		}
		static BOOL stopsavepic = true;
		if (stopsavepic)
		{
			CFileFind   find;
			BOOL   ret = 0;
			if (selectbandflag == 1)
			{
				ret = find.FindFile(L"D:\\Allpicx1H\\*.bmp");
			}
			else
			{
				ret = find.FindFile(L"D:\\Allpicx1f\\*.bmp");
			}
			//int     i = 0;
			while (ret)
			{
				ret = find.FindNextFile();
				if (find.IsDots() || find.IsDirectory())   continue;
				nu++;
			}
			stopsavepic = false;
		}

		CString lastax = ax + CString(xmlModelname[response]);
		CString numax;
		numax.Format(_T("_%d.bmp"), nu);
		lastax = lastax + numax;
		string ss = CT2A(lastax.GetBuffer());
		numax.ReleaseBuffer();
		nu++;
		//imwrite(ss, imgSrc); //保存图片
		cvSaveImage(ss.c_str(), &plImageP);

	}
	else if (save2C)
	{
		if (selectbandflag == 1)
		{
			ax = L"C:\\Allpicx1H\\";
		}
		else
		{
			ax = L"C:\\Allpicx1f\\";
		}
		if (!PathIsDirectory(ax))
		{
			::CreateDirectory(ax, NULL);
		}
		static BOOL stopsavepic = true;
		if (stopsavepic)
		{
			CFileFind   find;
			BOOL   ret = 0;
			if (selectbandflag == 1)
			{
				ret = find.FindFile(L"C:\\Allpicx1H\\*.bmp");
			}
			else
			{
				ret = find.FindFile(L"C:\\Allpicx1f\\*.bmp");
			}
			//int     i = 0;
			while (ret)
			{
				ret = find.FindNextFile();
				if (find.IsDots() || find.IsDirectory())   continue;
				nu++;
			}
			stopsavepic = false;
		}
		CString lastax = ax + CString(xmlModelname[response]);
		CString numax;
		numax.Format(_T("_%d.bmp"), nu);
		lastax = lastax + numax;
		string ss = CT2A(lastax.GetBuffer());
		numax.ReleaseBuffer();
		nu++;
		//imwrite(ss, imgSrc); //保存图片


		cvSaveImage(ss.c_str(), &plImageP);
	}
#endif
#if 1
	if (response > 42)// && maxrr <= 0.1)  //把返回图像旋转180度
	{
		//transpose(ImgArr[0], ImgArr[0]); //图片旋转90度
		if (response != 71)
		{
			flip(ImgArr[0], ImgArr[0], 0); //图片旋转180度
			flip(ImgArr[0], ImgArr[0], 1); //图片旋转90度
		}
		//transpose(ImgArr[0], ImgArr[0]);
	}
	else
	{
		//flip(ImgArr[0], ImgArr[0], 1);
		//transpose(ImgArr[0], ImgArr[0]);
	}
    //float Gammapara = 0.8;
    //int unigipara = 210;
    //int unitepara = 0;
    //int openhandle = 0;
    if (openhandle)
    {
        MyGammaCorrection(ImgArr[0], ImgArr[0], Gammapara);
        if (unitepara)
        {
            int thres = m_nThreshold;
            if (m_nThreshold > 60 || m_nThreshold<160)
            {
                imageAggrandize(ImgArr[0], m_nThreshold - 20, unigipara);
            }
        }
           
    }


#endif
	switch (response)
	{
	case 43:
		response = 1;
		break;
	case 44:
		response = 2;
		break;
	case 45:
		response = 3;
		break;
	case 46:
		response = 4;
		break;
	case 47:
		response = 5;
		break;
	case 48:
		response = 6;
		break;
	case 49:
		response = 7;
		break;
	case 50:
		response = 8;
		break;
	case 51:
		response = 9;
		break;
	case 52:
		response = 15;
		break;
	case 53:
		response = 16;
		break;
	case 54:
		response = 19;
		break;
	case 55:
		response = 21;
		break;
	case 56:
		response = 25;
		break;
	case 57:
		response = 28;
		break;
	case 58:
		response = 29;
		break;
	case 59:
		response = 30;
		break;
	case 60:
		response = 31;
		break;
	case 61:
		response = 32;
		break;
	case 62:
		response = 33;
		break;
	case 63:
		response = 35;
		break;
	case 64:
		response = 36;
		break;
	case 65:
		response = 37;
		break;
	case 66:
		response = 38;
		break;
	case 67:
		response = 39;
		break;
	case 68:
		response = 40;
		break;
	case 69:
		response = 41;
		break;
	case 70:
		response = 42;
		break;
	case 71:     //百搭
	case 72:
		response = 43;
		break;
	case 73:    //百搭纯白
		response = 44;
		break;
	default:
		response = response;
		break;
	}
	return response;
}

