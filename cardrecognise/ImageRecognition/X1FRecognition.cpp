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
const int n_AllCardNum = 4;  //��������

static const char* SpecialCardName[5] = { "TrainCard", "TrainCardA", "TrainCardB", "TrainCardC", "TrainCardD" };
static const char* xmlModelname[74] = { "", "һ��", "����", "����", "����", "����", "����", "����", "����", "����", \
"һͲ", "��Ͳ", "��Ͳ", "��Ͳ", "��Ͳ", "��Ͳ", "��Ͳ", "��Ͳ", "��Ͳ", "һ��", "����", \
"����", "����", "����", "����", "����", "����", "����", "����", "�Ϸ�", "����", "����", \
"����", "����", "�װ�", "��", "��", "��", "��", "÷", "��", "��", "��", "һ��", "����", "����", "����", "����", "����", "����", "����", \
"����", "��Ͳ��", "��Ͳ��", "һ����", "������", "������", "���絹", "�Ϸ絹", "���絹", "���絹", "���е�", "���Ƶ�", "����", "�ĵ�", "�ﵹ", "����", "÷��", "����", "�յ�", "��", "�ٴ�", "�ٴ", "����" };

X1FRecognition::X1FRecognition()
{
	//m_element = getStructuringElement(MORPH_RECT, Size(3, 3));
	params.svm_type = C_SVC;        // SVM��������
	params.C = 0.1;
	params.kernel_type = LINEAR;            //�˺������ͣ�����
	params.term_crit = TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 10000, 1e-6); // ��ֹ׼�����������������ﵽ���ֵʱ��ֹ
	m_pHog = new cv::HOGDescriptor(cvSize(64, 128), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);   //hog��ʼ����
	crossdetect1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

}

X1FRecognition::~X1FRecognition()
{
	if (m_pHog != NULL)delete m_pHog;
}
/********
����ʶ���ǣ������ǻҶ�ͼ������Ƕ�ֵ�����ͼ
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
	ת��Ϊ����
	***********/
	int nl = imForForm.rows; //����
	int nc = imForForm.cols;       //  imForForm.cols*imForForm.channels(); //ÿ��Ԫ�ص���Ԫ������
	if (nl < 2)
		return 0;
	uchar ImageDataSets[128][64];
	//ImageDataSets = new uchar*[nl* sizeof(uchar)];

	for (int j = 0; j<nl; j++)       //ת��������
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
	���������������ǿհ�����Ԥ��ʱʹ��
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
		int midleflag = 0;    //��ʾ�м�����ڰױ仯����
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
		//�пձ��
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
		//����ͶӰ,
		if (colSum > 2 * 255)
		{
			ProjectorVectorR[i] = 1;
		}
		else
		{
			ProjectorVectorR[i] = 0;
		}
		//���һ���� ������

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
				if (largelab > 7 * 255) //��С������
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
		if (i == (nl / 2)) //�°벿������
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
				if (largelab > 4 * 255) //��С������
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
	int selectcountwide = 0; //ͳ����Ӱ���
	int savenum1 = 0;    //����ÿ����Ӱ���
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
	if (savenum1<40 && savenum2<40)//&&crossdetect1.R_MidEmptyDown)   //ע�͵����¿�����Ϊһ������Щ��������Ҳ�ᱻ�ж�Ϊ��
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
	����������������ǿհ�����Ԥ��ʱʹ��
	********/
	uchar ProjectorVectorC[64] = { 0 };
	int largelabC = 0;
	int savelargelabC = 0;
	int countlableC = 0;
	for (int i = 0; i < nc; i++)
	{
		int RowSum = 0;
		int midleflag = 0;    //��ʾ�м�����ڰױ仯����
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
		//�пձ��
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
		//����ͶӰ
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
				crossdetect1.C_Width++;   //һ�����ͶӰ����
			}
			else
			{
				coutprojectorlabe++;
				crossdetect1.C_Width = 10;   //��ʾ�㹻��
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

void X1FRecognition::Sharpening(const Mat& myImage, Mat& Result)  //����ͼ����ǿ����Ҫ�ǽ��ͼ�����������ԣ����ף�������ٵ�ͼ����ǿ
{
	CV_Assert(myImage.depth() == CV_8U);  // ������ucharͼ��
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
	int nl = imgSrc.rows;//ʹ�÷���������һ������
	int nc = imgSrc.cols;
	if (xmin > 1)
		xmin -= 1;
	if (ymin > 2)
		ymin -= 2;
	if (nl - 4 > ymax)
		ymax += 4;
	if (nc - 2 > xmax)
		xmax += 2;
	aux = cvRect(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1);//�˴�����Ϊ�Ľ���
	//aux = cvRect(0, ymin, nc, ymax - ymin + 1);//�˴�����Ϊ�Ľ���
	return aux;

}
Mat X1FRecognition::preprocessing(Mat &imgSrc, int new_width, int new_height)
{
	assert(imgSrc.depth() != sizeof(uchar));
	int nc = imgSrc.rows;
	int nl = imgSrc.cols;
	CvRect bb;//bounding box
	bb = findBB(imgSrc);
	if (bb.height <  nc / 3)  //�ٴ��
	{
		if (bb.width < nl / 2)  //����������
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
	copyMakeBorder(scaledResult, addborder, y1, y1, x1, x1, BORDER_CONSTANT, Scalar::all(0));  //��0�߽�,left,right,up,down
	return addborder;
}

int X1FRecognition::EliminateBoundaryDisturb(Mat& BinaryImage)   //�����Ƕ�ֵͼ�������߽����
{
	assert(BinaryImage.depth() != sizeof(uchar));
	assert(BinaryImage.data);
	int nc = BinaryImage.rows;  //��
	int nl = BinaryImage.cols;  //��
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
		for (int j = 0; j < nl; j++)  //������
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
			if (((i>midnumR1  && i<midnumR2) || (i>nc - midnumR2 && i < nc - midnumR1)) && j>midnumC)//����
			{
				j = nl;
			}
			else if (i>midnumR2  && i < nc - midnumR2 && j>midnumC/2)    //���м䲿��
			{
				j = nl;			
			}
		}
		bool TrightUpFlag = rightUpFlag;
		bool TrightDownFlag = rightDownFlag;
		for (int j = nl-1; j >=0; j--)  //��������
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

int X1FRecognition::EliminateUpDownDisturb(const int threadValue, Mat& BinaryImage)   //�����Ƕ�ֵͼ�������߽����
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

bool X1FRecognition::ellipse_calc_bin_imgMat(const Mat img_cor, Mat & img_bin, const int meanThreshold)  //ѡ�������Զ���ֵ�����߽�����
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
	int UpContourVnum;  //�Ϸ���ֱ��������������������Ϸ��·���һ��
	int MidContourVnum;
	int DownContourVnum;
	int ContourPnum;  //�Ϸ�ƽ�е�������������һ���Ϸ�һ��
	int AllContour;
}CardContourNum = { 0, 0, 0, 0, 0 };

// ����ʶ�����ĸ���
//һ����������
int FindContourSureNum(const Mat& BinaryImg)  //ͨ����������ʶ���������
{
	if (BinaryImg.empty() || BinaryImg.type() != CV_8UC1) return -1;
	std::vector<vector<Point> > contour;
	std::vector<Vec4i> hierarchy;
	memset(&CardContourNum, 0, sizeof(struct CardContourInf));
	int wthred = BinaryImg.cols;
	int heithred = BinaryImg.rows;
	findContours(BinaryImg, contour, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //��������Ƕ�ֵ��ͨ��ͼ���Ҷ�ͼ����
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
	//drawContours(BinaryImg, contour, -1, Scalar(255), CV_FILLED);  //��ԭͼ����findcontour���ı�ͼ������

	int n = AreaCount.size();
	double MaxPixV = 0, MaxPixP = 0;
	for (int i = 0; i < n; i++)  //���������
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
			//ճ�����⣬�ü�������׼ȷ
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
	if (m_bThresholdZip)   //�Ѿ���ֵ���Ͳ��ö�ֵ����
	{
		imgSrc.copyTo(imgBin);
	}
	else
	{
		
		Rect AUTObin(imgSrc.cols / 8, imgSrc.rows / 8, imgSrc.cols - imgSrc.cols / 4, imgSrc.rows - imgSrc.rows / 4);
		Mat AutoOptic = imgSrc(AUTObin);
		bool deleateNoiseFlag = false;
		m_nThreshold = getThreshVal_optic(AutoOptic);
		if (m_nThreshold > 108)  //��ֵ��ֵ�㹻��
		{
            float thredvalue = getValidValue(AutoOptic, 60);   // ������Ч����С��60
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
			//GaussianBlur(imgBin, imgBin, Size(3, 3), 0, 0);  // �˲�
			threshold(imgSrc, imgBin, m_nThreshold, 255, CV_THRESH_BINARY_INV); //m_nThreshold 65����
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
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3)); //��̬ѧ�˺���
	morphologyEx(tempImage, tempImage, MORPH_OPEN, element); //������ʹ������������
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
	Mat ImageSelect = FindContours(lastTestImage, 0.002*lastTestImage.cols*lastTestImage.rows, 0.98*lastTestImage.cols*lastTestImage.rows);   //����Ѿ�С����ȥ�������ˣ��Ϳ��Բ��ô���ϸС������ɾ�����׵���ʶ�����
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
		CString ax, side;                  //��ֵͼ�񱣴�
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
		//imwrite(ss, imForHog);         //����
		cvSaveImage(ss.c_str(), &plImageP);
	}
	m_pHog->compute(imForHog, descriptors, cvSize(16, 16), cv::Size(0, 0));
	TagLableBrand(imForHog);
	Mat findsureNum = imForHog.clone();
	FindContourSureNum(findsureNum);  //ͨ����������ʶ���������  //��ı�ͼƬ����
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
	findContours(imgSrc, contour, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); //��������Ƕ�ֵ��ͨ��ͼ���Ҷ�ͼ����
	Mat imgDst = Mat::zeros(imgSrc.size(), CV_8U);

	for (int i = 0; i < contour.size(); i++) //ɾ��������С������
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
		case 45://���򵹲¶���
		if (crossdetect1.charcter2)
		response = 44;
		if (crossdetect1.Giveupcharcter)
		response = 45;
		break;
		/*****/
	case 32:  //���в¶���
		if (crossdetect1.R_AreaNum1 != 1 && crossdetect1.C_Width != 10)
			response = 20;
		break;
	case 61:  //���в¶���
		if (crossdetect1.R_AreaNum1 != 1 && crossdetect1.C_Width != 10)
			response = 20;
		break;
	case 17://��Ͳ����Ͳ
		if (crossdetect1.R_AreaNum1 != 1)
			response = 15;
		break;
	case 15: //��Ͳ�°�Ͳ����Ͳ
		if (crossdetect1.R_AreaNum1)
			response = 17;
		else if (crossdetect1.C_AreaNum2)
			response = 13;
		break;
	case 52: //��Ͳ�°�Ͳ����Ͳ
		if (crossdetect1.R_AreaNum1)
			response = 17;
		else if (crossdetect1.C_AreaNum2)
			response = 13;
		break;
	case 20://����������
		if (crossdetect1.C_AreaNum3&& CardContourNum.DownContourVnum>2)
		{
			//GuessFlag = true;
			response = 25;
		}
		else if (crossdetect1.C_Width == 10 && CardContourNum.AllContour < 2)  // �����º���
		{
			//GuessFlag = true;
			response = 32;
		}
		break;
	case 24:  //����������
		//�����¾���
		if (CardContourNum.UpContourVnum == 3 && CardContourNum.MidContourVnum * 3 <= CardContourNum.AllContour)
		{
			//GuessFlag = true;
			response = 27;
		}
		else if (CardContourNum.UpContourVnum == 1)  //����������
		{
			//GuessFlag = true;
			response = 25;
		}
		else if (crossdetect1.C_AreaNum2&&crossdetect1.R_AreaNum2&&crossdetect1.R_AreaNum3&&crossdetect1.R_AreaNum1)//����������
		{
			//GuessFlag = true;
			response = 22;
		}
		break;
		/*********/
	case 22:  //�����°�Ͳ,����
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
	case 27: //����������������
		if (CardContourNum.UpContourVnum == 0 && (CardContourNum.DownContourVnum + CardContourNum.MidContourVnum + 1) >= CardContourNum.AllContour)
		{
			//GuessFlag = true;
			response = 24;
		}
		break;
	case 34: // �װ������
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
void  X1FRecognition::Seed_Filling(const Mat& binImg, Mat& lableImg, int &num)   //������䷨
{
	// 4�ڽӷ���
	//8�ڽӸ���ȷ��Ŀǰ���ò���
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
			if (data[j] == 1)  //ѡ��������ֵ
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(i, j));     // ����λ��: <i,j>
				++label;  // û���ظ����ţ���ʼ�µı�ǩ
				while (!neighborPixels.empty())
				{
					std::pair<int, int> curPixel = neighborPixels.top(); //�������һ����һ�������غ���������һ�е��Ǹ��ŵı�Ÿ�����
					int curX = curPixel.first;
					int curY = curPixel.second;
					lableImg.at<int>(curX, curY) = label;
					neighborPixels.pop();
					if (curX>1 && curY>1 && curX<(rows - 1) && curY<(cols - 1))//�߽��ж�,�������
					{
						if (lableImg.at<int>(curX, curY - 1) == 1)
						{//���
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (lableImg.at<int>(curX, curY + 1) == 1)
						{// �ұ�
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (lableImg.at<int>(curX - 1, curY) == 1)
						{// �ϱ�
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (lableImg.at<int>(curX + 1, curY) == 1)
						{// �±�
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
			if (data[j] == 1)  //ѡ��������ֵ
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(i, j));     // ����λ��: <i,j>
				++label;  // û���ظ����ţ���ʼ�µı�ǩ
				while (!neighborPixels.empty())
				{
					std::pair<int, int> curPixel = neighborPixels.top(); //�������һ����һ�������غ���������һ�е��Ǹ��ŵı�Ÿ�����
					int curX = curPixel.first;
					int curY = curPixel.second;
					lableImg.at<int>(curX, curY) = label;
					neighborPixels.pop();
					if (curX>1 && curY>1 && curX<(rows - 1) && curY<(cols - 1))//�߽��ж�,�������
					{
						if (lableImg.at<int>(curX, curY - 1) == 1)
						{//���
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (lableImg.at<int>(curX, curY + 1) == 1)
						{// �ұ�
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (lableImg.at<int>(curX - 1, curY) == 1)
						{// �ϱ�
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (lableImg.at<int>(curX + 1, curY) == 1)
						{// �±�
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
	outLabelImg = cv::Scalar::all(0); //����
	countnum = 0;
	int maxlinenum = 0; //���ڷ��ص������
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
			if (num == pixelValue)//ָ����ͨ����������
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
	short datalabe[] = { -1, 1 }; //��ǩ
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
	short datalabe[] = { -1, 1 }; //��ǩ
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

//gamma����
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
	//����
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
	transpose(imgSrc, imgSrc); //ͼƬ��ת90��
	flip(imgSrc, imgSrc, 1); //ͼƬ��ת90��	
	Mat outImg;
	HogFeatureOfOneImageForSVM(imgSrc, outImg);
	Mat sampleFeatureMat = Mat::zeros(1, descriptors.size(), CV_32FC1);
	sampleFeatureMat = Mat(descriptors);
	int response = 0;
	int saveResponse = 0;
	float maxrr = 100;
	float result;
	int numxml = sizeof(xmlModelname) / sizeof(xmlModelname[0]);  //xml����
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
			if (result <= maxrr)  //�жϾ�����С��
			{
				maxrr = result;
				response = i + j*numxml;
			}
		}
	}
	saveResponse = response;
	response = response % numxml;   //����������
	//free(ret);
	//ret = NULL;
	if (response == 52 && maxrr > -0.8 && (saveResponse<74 || saveResponse>222))  //��Ͳ��
	{
		response = SecondPredict(response);  //�����ж�
	}
	else if (maxrr > 0 && (saveResponse < 74 || saveResponse>222))
	{
		response = SecondPredict(response);  //�����ж�
	}

#if 0  //ͨ���������и��жϴ�С��ʶ����������һ��

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
		Mat borderinput = outImg.clone(); //��ȿ���
		resize(borderinput, borderinput, cv::Size(64, 128), 0, 0, CV_INTER_LINEAR);
		if (falllabel)
		{
			flip(borderinput, borderinput, 0); //ͼƬ��ת90��
			flip(borderinput, borderinput, 1); //ͼƬ��ת90��
		}
		int num = 1; //��ͨ��
		Mat labelimg;
		Seed_Filling(borderinput, labelimg, num);
		vector <int> sectest;//��Ч����
		vector<int>Secheight; //����λ��
		vector<int>realSecheight;//�߶�
		vector<int>maxlnum;//���������
		vector<bool>addhead;
		for (int i = 1; i <= num; i++)
		{
			int heightLabel = 0;
			int countnum = 0;
			int realheight = 0;
			int maxlinenum = 0;
			bool addlabel = false;
			maxlinenum = Label_connect(labelimg, i, countnum, heightLabel, realheight, addlabel);
			if (countnum < 8) //5���㣬����//����������ļ��
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
			if ((sectest.at(2) >dottest + dottest / 3) && (maxlnum.at(2) - maxlnum.at(1)>maxlnum.at(1) / 3))  //���������
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
		else if ((colnum >1) && (Secheight.at(1)<(borderinput.rows / 2))) //����¶���//���������
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
		if ((maxlnum.at(0)>30) && (maxlnum.at(0) - maxlnum.at(1)>maxlnum.at(1) / 2))              //�����һ��,maxlnum.at(0)>30һ�򳤶�
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
		CString strText = _T("�ļ���      ʶ����         Ԥ��ֵ      С��ͷ�ж�");//strTextΪҪд��txt�ļ�������
		//�����Unicode�ַ�������д�����ĵ�ʹ����Ҫ���������һ�䣬ASCII����.��Ҫ����ͷ�ļ�locale.h
		char* pOldLocale = setlocale(LC_CTYPE, "chs");
		CString TxtFold_file;
		TxtFold_file.Format(_T("D:\\Allpic\\%d.txt"), nu - 1);
		CStdioFile fOutput(TxtFold_file, CFile::modeCreate | CFile::modeWrite | CFile::typeText);
		CString strTextM;
		strTextM.Format(_T("%f"), maxrr);
		CString realname;
		if (2 == response || 38 == response)
		{
			realname = _T("����");
		}
		else if (3 == response || 39 == response)
		{
			realname = _T("����");

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

	//ͨ��С���ж϶��������
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

		// ��ʱ��������ͼƬ
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
		//imwrite(ss, imgSrc); //����ͼƬ
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
		//imwrite(ss, imgSrc); //����ͼƬ


		cvSaveImage(ss.c_str(), &plImageP);
	}
#endif
#if 1
	if (response > 42)// && maxrr <= 0.1)  //�ѷ���ͼ����ת180��
	{
		//transpose(ImgArr[0], ImgArr[0]); //ͼƬ��ת90��
		if (response != 71)
		{
			flip(ImgArr[0], ImgArr[0], 0); //ͼƬ��ת180��
			flip(ImgArr[0], ImgArr[0], 1); //ͼƬ��ת90��
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
	case 71:     //�ٴ�
	case 72:
		response = 43;
		break;
	case 73:    //�ٴ��
		response = 44;
		break;
	default:
		response = response;
		break;
	}
	return response;
}

