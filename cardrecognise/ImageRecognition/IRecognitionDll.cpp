#define RECOGNITION_DLL _declspec(dllexport)

#include <afx.h>
#include <afxwin.h>
#include "ImgRecognitionFactory.h"
#include "IRecognitionDll.h"

#pragma comment(lib, "IlmImf.lib")
#pragma comment(lib, "libjasper.lib")
#pragma comment(lib, "libjpeg.lib")
#pragma comment(lib, "libpng.lib")
#pragma comment(lib, "libtiff.lib")
#pragma comment(lib, "zlib.lib")
#pragma comment(lib, "opencv_calib3d2410.lib")
#pragma comment(lib, "opencv_contrib2410.lib")
#pragma comment(lib, "opencv_core2410.lib")
#pragma comment(lib, "opencv_features2d2410.lib")
#pragma comment(lib, "opencv_flann2410.lib")
#pragma comment(lib, "opencv_gpu2410.lib")
#pragma comment(lib, "opencv_highgui2410.lib")
#pragma comment(lib, "opencv_imgproc2410.lib")
#pragma comment(lib, "opencv_legacy2410.lib")
#pragma comment(lib, "opencv_ml2410.lib")
#pragma comment(lib, "opencv_nonfree2410.lib")
#pragma comment(lib, "opencv_objdetect2410.lib")
#pragma comment(lib, "opencv_ocl2410.lib")
#pragma comment(lib, "opencv_photo2410.lib")
#pragma comment(lib, "opencv_stitching2410.lib")
#pragma comment(lib, "opencv_superres2410.lib")
#pragma comment(lib, "opencv_ts2410.lib")
#pragma comment(lib, "opencv_video2410.lib")
#pragma comment(lib, "opencv_videostab2410.lib")
#pragma comment(lib, "User32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "vfw32.lib")
#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "AdvAPI32.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "oleaut32.lib")

static CStartAlgorithm* g_pAlg = NULL;
static WCHAR g_strAlgorithmName[128] = {0};
static HANDLE g_hMutex = NULL;
BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, LPVOID lpReserved)
{
	if (DLL_PROCESS_ATTACH == dwReason)
	{
		if (NULL == g_hMutex) g_hMutex = ::CreateMutex(NULL, FALSE, NULL);
	}
	else if (DLL_PROCESS_DETACH == dwReason)
	{
		if (NULL != g_hMutex)
		{
			::CloseHandle(g_hMutex);
			g_hMutex = NULL;
		}
	}

	return TRUE;
}

bool initRecogn(const WCHAR* AlgorithmName,const int NUM)
{
	::WaitForSingleObject(g_hMutex, INFINITE);

	bool bSuccess = false;
	if (NULL == g_pAlg) g_pAlg = new CStartAlgorithm;
	if (NULL != g_pAlg && 0 != wcscmp(AlgorithmName, g_strAlgorithmName)) //切换算法
	{
		wcscpy_s(g_strAlgorithmName, AlgorithmName);
		if (g_pAlg->SelectAlgorithm(AlgorithmName,NUM)) bSuccess = true;
		else
		{
			//AfxMessageBox(_T("未定义的算法！"));
		}
	}

	::ReleaseMutex(g_hMutex);
	 return bSuccess;
}

void releaseRecogn()
{
	::WaitForSingleObject(g_hMutex, INFINITE);

	delete g_pAlg;
	g_pAlg = NULL;

	::ReleaseMutex(g_hMutex);
}

int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum
	, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo)
{
	::WaitForSingleObject(g_hMutex, INFINITE);

	if (NULL != pDstImageInfo) pDstImageInfo->pOutDst = NULL;
	int nCardIndex = -1;
	if (NULL == pImg || NULL == pDstImageInfo)
	{
		::ReleaseMutex(g_hMutex);

		//AfxMessageBox(_T("startRecogn传入的指针为空！"));
		return nCardIndex;
	}
	if (nNum <= 0 || nWidth <= 0 || nHeight <= 0)
	{
		::ReleaseMutex(g_hMutex);

		WCHAR strErr[256];
		wsprintf(strErr, _T("传入的参数有误！nNum:%d,nWidth:%d,nHeight:%d"), nNum, nWidth, nHeight);
		//AfxMessageBox(strErr);
		return nCardIndex;
	}

	if (NULL != g_pAlg) nCardIndex = g_pAlg->start(nCardNum, strSide, nThreshold, pImg, nNum, nWidth, nHeight, pDstImageInfo);

	::ReleaseMutex(g_hMutex);
	return nCardIndex;
}


CInterFace::CInterFace(const WCHAR* AlgorithmName, const int NUM)
{
	g_pAlg = new CStartAlgorithm;
	if (NULL != g_pAlg)
	{
		if (!g_pAlg->SelectAlgorithm(AlgorithmName,NUM))
		{
			//AfxMessageBox(_T("未定义的算法！"));
		}
	}
}

CInterFace::~CInterFace()
{
	::WaitForSingleObject(g_hMutex, INFINITE);

	delete g_pAlg;
	g_pAlg = NULL;

	::ReleaseMutex(g_hMutex);
}

int CInterFace::startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const WCHAR* strPath)
{
	::WaitForSingleObject(g_hMutex, INFINITE);

	int nCardIndex = -1;
	if (NULL != g_pAlg) nCardIndex = g_pAlg->start(nCardNum, strSide, nThreshold, strPath);

	::ReleaseMutex(g_hMutex);
	return nCardIndex;
}

int CInterFace::startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo)
{
	::WaitForSingleObject(g_hMutex, INFINITE);

	if (NULL != pDstImageInfo) pDstImageInfo->pOutDst = NULL;
	int nCardIndex = -1;
	if (NULL == pImg || NULL == pDstImageInfo)
	{
		::ReleaseMutex(g_hMutex);

		//AfxMessageBox(_T("startRecogn传入的指针为空！"));
		return nCardIndex;
	}
	if (nNum <= 0 || nWidth <= 0 || nHeight <= 0)
	{
		::ReleaseMutex(g_hMutex);

		WCHAR strErr[256];
		wsprintf(strErr, _T("传入的参数有误！nNum:%d,nWidth:%d,nHeight:%d"), nNum, nWidth, nHeight);
		//AfxMessageBox(strErr);
		return nCardIndex;
	}

	if (NULL != g_pAlg) nCardIndex = g_pAlg->start(nCardNum, strSide, nThreshold, pImg, nNum, nWidth, nHeight, pDstImageInfo);

	::ReleaseMutex(g_hMutex);
	return nCardIndex;
}

int CInterFace::startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const std::vector<WCHAR*>& strPathArr)
{
	::WaitForSingleObject(g_hMutex, INFINITE);

	int nCardIndex = -1;
	if (NULL != g_pAlg) nCardIndex = g_pAlg->start(nCardNum, strSide, nThreshold, strPathArr);

	::ReleaseMutex(g_hMutex);
	return nCardIndex;
}