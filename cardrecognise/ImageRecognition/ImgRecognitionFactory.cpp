#include <afx.h>
#include <afxwin.h>
#include <map>
#include "ImageWindowTestTool.h"

#include "ImgRecognitionFactory.h"
#include "X1FRecognition.h"

#define CARDCLASSARRAYSIZE 39

static void ImageBlend(IplImage* pImgCurrent, IplImage* pImgResult,const int Num) //对图像进行叠加拼接
{
	if (pImgCurrent->height != pImgResult->height) return;
	if (pImgCurrent->width*Num > pImgResult->width) return;
	if (3 != pImgCurrent->nChannels || 3 != pImgResult->nChannels) return;

	for (int row = 0; row < pImgCurrent->height; row++)
	{
		const uchar* ptr_c = (uchar*)pImgCurrent->imageData + row*pImgCurrent->widthStep; //图像的单独部分
		uchar* ptr_r = (uchar*)pImgResult->imageData + row*pImgResult->widthStep + pImgCurrent->width * 3*(Num-1);
		for (int col = 0; col < pImgCurrent->width; col++)
		{
			const uchar* ptr_c1 = ptr_c + 1;
			const uchar*  ptr_c2 = ptr_c1 + 1;
			uchar* ptr_r1 = ptr_r + 1;
			uchar* ptr_r2 = ptr_r1 + 1;

			*ptr_r = *ptr_c;
			*ptr_r1 = *ptr_c1;
			*ptr_r2 = *ptr_c2;

			ptr_r += 3;
			ptr_c += 3;
		}
	}
}

static void GetColorPartImg(void* pImg, const int nNum, const int nWidth, const int nHeight, IplImage* pImgJointResult, std::vector<Mat>& imgArr,int nStartPos) //从裸数据中获取出彩色片段的图片
{
	int RGBlinesize = (nWidth * 24 + 31) / 32 * 4;
	DWORD RGBimagesize = RGBlinesize*nHeight;
	BYTE* pWrite = new BYTE[RGBimagesize]; //保存即将要写入的RGB数据
	memset(pWrite, 0, RGBimagesize); //填充为0
	IplImage* pImgWrite = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 3);

	int nJointNum = 0;
	int nYlinesize = (nWidth * 8 + 31) / 32 * 4;
	int nUVWidth = 2 * ((nWidth + 1) / 2);
	int nUVHeight = (nHeight + 1) / 2;
	DWORD YUVimagesize = nYlinesize*nHeight + nUVWidth*nUVHeight;
	for (int k = 0; k < nNum; k++)
	{
		BYTE* pRead = (BYTE*)pImg + k*YUVimagesize;
		BYTE* pUV = pRead + nYlinesize*nHeight;
		for (int i = 0; i < nHeight; i++)
		{
			for (int j = 0; j < nWidth; j++)
			{
				int iUV = i / 2; //除2，是因为4个相邻的Y共用一对UV
				int jUV = j / 2; //除2，是因为4个相邻的Y共用一对UV
				BYTE Y = *(pRead + i*nYlinesize + j);
				BYTE U = *(pUV + iUV*nUVWidth + 2 * jUV); //乘2，因为，UV是一个组合出现的
				BYTE V = *(pUV + iUV*nUVWidth + 2 * jUV + 1);
				int GrayR = Y + 1.4075*(V - 128);
				int GrayG = Y - 0.3455*(U - 128) - 0.7169*(V - 128);
				int GrayB = Y + 1.779*(U - 128);
				if (GrayR < 0) GrayR = 0;
				if (GrayR > 255) GrayR = 255;
				if (GrayG < 0) GrayG = 0;
				if (GrayG > 255) GrayG = 255;
				if (GrayB < 0) GrayB = 0;
				if (GrayB > 255) GrayB = 255;
				*(pWrite + i*RGBlinesize + 3 * j) = (BYTE)GrayR;
				*(pWrite + i*RGBlinesize + 3 * j + 1) = (BYTE)GrayG;
				*(pWrite + i*RGBlinesize + 3 * j + 2) = (BYTE)GrayB;
			}
		}
		pImgWrite->imageData = (char*)pWrite;
		imgArr[nStartPos++] = Mat(pImgWrite, true);

		if (NULL != pImgJointResult && (0 == k || nNum / 2 == k || nNum - 1 == k))
		{
			nJointNum++;
			ImageBlend(pImgWrite, pImgJointResult, nJointNum);
		}
	}

	cvReleaseImageHeader(&pImgWrite);
	delete[] pWrite;
}

static void GetJointImg(void* pImg, const int nWidth, const int nHeight, IplImage* pImgWrite, const bool bThresholdZip) //从裸数据中获取出拼接的结果图
{
	if (!bThresholdZip) memcpy(pImgWrite->imageData, pImg, pImgWrite->imageSize); //没有进行二值化压缩
	else
	{
		int nBitCount = 1;
		int linesize = (nWidth * nBitCount + 31) / 32 * 4;

		BYTE nLoop = 0;
		BYTE GREY = 0;
		int nCurrentBytePos = 0; //每一行的当前所在的字节位置
		for (int i = 0; i < nHeight; i++)
		{
			uchar* ptr_w = (uchar*)pImgWrite->imageData + i*pImgWrite->widthStep;
			BYTE* pRead = (BYTE*)pImg + i*linesize;
			nCurrentBytePos = 0; //每一行的当前所在的字节位置
			for (int j = 0; j < nWidth;)
			{
				BYTE VALUE = *(pRead + nCurrentBytePos);
				while (nLoop < 8)
				{
					GREY = (0x80 >> nLoop) & VALUE;
					if (GREY > 0) *(ptr_w + j) = 255;
					else *(ptr_w + j) = 0;
					nLoop++;
					j++;
					if (j >= nWidth) break;
				}

				nCurrentBytePos++;
				nLoop = 0;
			}
		}
	}
}

//static bool strCardClassArray[CARDCLASSARRAYSIZE] = { false };
//static void InitCardClassArray()
//{
//	CString strCardNameArr[CARDCLASSARRAYSIZE] = { _T(""), \
//		_T("yiwan"), _T("erwan"), _T("sanwan"), _T("siwan"), _T("wuwan"), _T("liuwan"), _T("qiwan"), _T("bawan"), _T("jiuwan"), \
//		_T("yitong"), _T("ertong"), _T("santong"), _T("sitong"), _T("wutong"), _T("liutong"), _T("qitong"), _T("batong"), _T("jiutong"), \
//		_T("yitiao"), _T("ertiao"), _T("santiao"), _T("sitiao"), _T("wutiao"), _T("liutiao"), _T("qitiao"), _T("batiao"), _T("jiutiao"), \
//		_T("dongfeng"), _T("nanfeng"), _T("xifeng"), _T("beifeng"), \
//		_T("hongzhong"), _T("facai"), _T("baiban")
//	};
//	std::map<CString, int> mapNameToIndex; //保存CString-index对应关系
//	for (int i = 1; i < CARDCLASSARRAYSIZE; i++)
//	{
//		if (_T("") != strCardNameArr[i]) mapNameToIndex.insert(std::map<CString, int>::value_type(strCardNameArr[i], i));
//	}
//
//	std::map<CString, int>::iterator ptr;
//	CStdioFile file;
//	if (file.Open(_T("CardConfig.log"), CFile::modeRead))
//	{
//		CString strCardName;
//		while (file.ReadString(strCardName))
//		{
//			ptr = mapNameToIndex.find(strCardName);
//			if (mapNameToIndex.end() != ptr)
//			{
//				strCardClassArray[ptr->second] = true;
//			}
//		}
//
//		file.Close();
//	}
//}

bool save2D = false;
#if 0
bool save2C = true;//false;
bool savebinery2C = true; //false
#else
bool save2C = false;
bool savebinery2C = false; 
#endif
bool write2txt = false;

float Gammapara = 0.38;
int unigipara = 210;
int unitepara = 1;
int openhandle = 0;

int initialParam(const int flag)
{
	std::ifstream fin("上山打老虎", std::ios::in);
	string DebugNameArr[5] = {"1","54TC","ggbB2C","GHBWRITE","HFHHHF"};
	const int num = sizeof(DebugNameArr) / sizeof(DebugNameArr[0]);
	char line[128] = { 0 };
	while (fin.getline(line, sizeof(line)))
	{
		stringstream word(line);
		word >> DebugNameArr[0];
		int xn = 0;
		for (int i = 0; i < num; i++)
		{
			if (DebugNameArr[0] == DebugNameArr[i])
			{
				xn = i;
			}
		}
		switch (xn)
		{
		case 1:
			save2C = true;
			save2D = false;
			break;
		case 2:
			savebinery2C = true;
			break;
		case 3:
			write2txt = true;
			break;
		case 4:
			//g_pRecon->
			break;
		default:
			if (save2C)
			save2D = true;
			break;
		}
	}
	fin.clear();
	fin.close();
#if 1 
    if (flag)
    {
        openhandle = 1;
    }
    std::ifstream finp("gammapara", std::ios::in);
    char linep[128] = { 0 };
    string DebugNameArrp[5] = { "", "gammapara", "unifi", "unite" };
   
    while (finp.getline(linep, sizeof(linep)))
    {
        stringstream word(linep);
        word >> DebugNameArrp[0];
        if (DebugNameArrp[0] == DebugNameArrp[1])
        {
            finp.getline(linep, sizeof(linep));
            stringstream word(linep);
            word >> Gammapara;
            if (Gammapara < 0 || Gammapara > 1)
            {
                Gammapara = 1;
            }

        }
        else if (DebugNameArrp[0] == DebugNameArrp[2])
        {
            finp.getline(linep, sizeof(linep));
            stringstream word(linep);
            word >> unigipara;
            if (unigipara > 254)
            {
                unigipara = 254;
            }
            else if (unigipara < 100)
            {
                unigipara = 100;
            }

        }
        else if (DebugNameArrp[0] == DebugNameArrp[3])
        {
            finp.getline(linep, sizeof(linep));
            stringstream word(linep);
            word >> unitepara;
            if (unitepara>0)
            {
                unitepara = 1;
            }
            else
            {
                unitepara = 0;
            }
        }
    }
    finp.clear();
    finp.close();
#endif

	return 0;
}

CStartAlgorithm::CStartAlgorithm()
{
	m_hMutex = ::CreateMutex(NULL, FALSE, NULL);
	m_pDFEDst = new BYTE[200*1024];
	m_pHOGDst = NULL;
	//InitCardClassArray();
	m_pRecon = NULL;
	m_pSecondClassifyRecon = NULL;
}

CStartAlgorithm::~CStartAlgorithm()
{
	::WaitForSingleObject(m_hMutex, INFINITE);

	delete m_pRecon;
	delete m_pSecondClassifyRecon;
	m_pRecon = NULL;
	m_pSecondClassifyRecon = NULL;

	delete m_pDFEDst;
	delete m_pHOGDst;
	m_pDFEDst = NULL;
	m_pHOGDst = NULL;

	::ReleaseMutex(m_hMutex);

	::CloseHandle(m_hMutex);
}

int selectbandflag = 3;  //版本选帧
//默认向上兼容所有识别
bool CStartAlgorithm::SelectAlgorithm(const WCHAR* AlgorithmName,const int NUM)
{
	::WaitForSingleObject(m_hMutex, INFINITE);
	wcscpy_s(m_strAlgorithmName, AlgorithmName);
    int flag = 0;
    if (0 == wcscmp(m_strAlgorithmName, _T("bigbig")))
    {
        flag = 1;
    }
	initialParam(flag);
	delete m_pRecon;
	delete m_pSecondClassifyRecon;
	m_pRecon = NULL;
	m_pSecondClassifyRecon = NULL;
	bool bFind = false;

	switch (NUM)
	{

	case 1:
		bFind = true;
		selectbandflag = 0; //
		m_pRecon = new X1FRecognition;
		break;
	case 2:
		bFind = true;
		selectbandflag = 1;
		m_pRecon = new X1FRecognition;
        break;
	default:
		bFind = true;
		selectbandflag = 2;
		m_pRecon = new X1FRecognition;
		break;
	}
	
	::ReleaseMutex(m_hMutex);
	return bFind;
}

int CStartAlgorithm::start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const WCHAR* strPath)
{
	::WaitForSingleObject(m_hMutex, INFINITE);

	if (NULL == m_pRecon)
	{
		::ReleaseMutex(m_hMutex);
		return -1;
	}
	m_pRecon->SetParam(nCardNum, strSide, nThreshold);

	std::vector<Mat> imgArr;
	char* pszPath = ::WideconvertMulti(strPath);
	imgArr.push_back(imread(pszPath,0)); //载入灰度图
	delete[] pszPath;

	int nCardIndex = m_pRecon->StartRecon(imgArr);

	::ReleaseMutex(m_hMutex);
	return nCardIndex;
}

int CStartAlgorithm::start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo)
{
	::WaitForSingleObject(m_hMutex, INFINITE);

	if (NULL != pDstImageInfo) pDstImageInfo->pOutDst = NULL; //初始化
	if (NULL == m_pRecon || NULL == m_pDFEDst)
	{
		::ReleaseMutex(m_hMutex);
		return -1;
	}
	if (nNum <= 0 || nWidth <= 0 || nHeight <= 0)
	{
		::ReleaseMutex(m_hMutex);
		return -1;
	}

	bool bThresholdZip = false; //是否有进行二值化压缩
	std::vector<Mat> imgArr(nNum);
	if (0 /*0 == wcscmp(m_strAlgorithmName, _T("HOG"))*/) //彩色图片，不再用这种算法了
	{
		int RGBlinesize = (nWidth * 24 + 31) / 32 * 4;
		DWORD RGBimagesize = RGBlinesize*nHeight;
		const int nImgPartNum = 3; //要显示的拼接片段的数量
		if (NULL == m_pHOGDst) m_pHOGDst = new BYTE[RGBimagesize * nImgPartNum]; //要从中选出3个片段显示
		IplImage* pImgJointResult = cvCreateImage(cvSize(nWidth * nImgPartNum, nHeight), IPL_DEPTH_8U, 3);
		GetColorPartImg(pImg, nNum, nWidth, nHeight, pImgJointResult, imgArr,0);
		if (NULL != pDstImageInfo)
		{
			memcpy(m_pHOGDst, pImgJointResult->imageData, pImgJointResult->imageSize);
			pDstImageInfo->pOutDst = m_pHOGDst;
			pDstImageInfo->nOutWidth = pImgJointResult->width;
			pDstImageInfo->nOutHeight = pImgJointResult->height;
			pDstImageInfo->nOutChannels = pImgJointResult->nChannels;
			pDstImageInfo->nImgSize = pImgJointResult->imageSize;
		}
		cvReleaseImage(&pImgJointResult);
	}
	else //灰色图片和彩色片段
	{
		IplImage* pImgWrite = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
		DWORD imagesize = pImgWrite->imageSize;
		if (0 != *((BYTE*)pImg)) bThresholdZip = true; //有进行二值化压缩
		pImg = (BYTE*)pImg + 1;
		GetJointImg(pImg, nWidth, nHeight, pImgWrite, bThresholdZip);
		imgArr[0] = Mat(pImgWrite, true); //拼接结果图
#if 0
		if (NULL != pDstImageInfo)
		{
			memcpy(m_pDFEDst, pImgWrite->imageData, pImgWrite->imageSize);
			pDstImageInfo->pOutDst = m_pDFEDst;
			pDstImageInfo->nOutWidth = pImgWrite->width;
			pDstImageInfo->nOutHeight = pImgWrite->height;
			pDstImageInfo->nOutChannels = pImgWrite->nChannels;
			pDstImageInfo->nImgSize = pImgWrite->imageSize;
		}
#endif
		cvReleaseImage(&pImgWrite);

		int nColorNum = nNum - 1; //彩色片段的数量
		if (nColorNum > 0) //把彩色片段取出来
		{
			BYTE *pColorImg = (BYTE*)pImg + imagesize;
			int nColorWidth = *(int*)pColorImg;
			int nColorHeight = *((int*)pColorImg + 1);
			pColorImg = pColorImg + 8;
			if (nColorWidth > 0 && nColorHeight > 0) GetColorPartImg(pColorImg, nColorNum, nColorWidth, nColorHeight, NULL, imgArr,1);
		}
	}
	m_pRecon->SetParam(nCardNum, strSide, nThreshold,bThresholdZip);

	int nCardIndex = m_pRecon->StartRecon(imgArr);
	//if (NULL != m_pSecondClassifyRecon && strCardClassArray[nCardIndex]) nCardIndex = m_pSecondClassifyRecon->StartRecon(imgArr);
	//if (0 == wcscmp(m_strAlgorithmName, _T("HOG2")))//灰色图片和彩色片段
	//{
		IplImage* imgoutt = &IplImage(imgArr[0]);
		if (NULL != pDstImageInfo)
		{
			memcpy(m_pDFEDst, imgoutt->imageData, imgoutt->imageSize);
			pDstImageInfo->pOutDst = m_pDFEDst;
			pDstImageInfo->nOutWidth = imgoutt->width;
			pDstImageInfo->nOutHeight = imgoutt->height;
			pDstImageInfo->nOutChannels = imgoutt->nChannels;
			pDstImageInfo->nImgSize = imgoutt->imageSize;
		}
	//}

	::ReleaseMutex(m_hMutex);
	return nCardIndex;
}

int CStartAlgorithm::start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const std::vector<WCHAR*>& strPathArr)
{
	::WaitForSingleObject(m_hMutex, INFINITE);

	if (NULL == m_pRecon)
	{
		::ReleaseMutex(m_hMutex);
		return -1;
	}
	m_pRecon->SetParam(nCardNum, strSide, nThreshold);

	std::vector<Mat> imgArr;
	for (int i = 0; i < strPathArr.size();i++)
	{
		char* pszPath = ::WideconvertMulti(strPathArr[i]);
		imgArr.push_back(imread(pszPath));
		delete[] pszPath;
	}

	int nCardIndex = m_pRecon->StartRecon(imgArr);

	::ReleaseMutex(m_hMutex);
	return nCardIndex;
}
