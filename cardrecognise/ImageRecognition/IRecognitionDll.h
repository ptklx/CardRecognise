#ifndef _IRECOGNITIONDLL_H
#define _IRECOGNITIONDLL_H

#ifndef RECOGNITION_DLL
#define RECOGNITION_DLL _declspec(dllimport)
#endif

#include <vector>
#include "ImgRecoginitionOutParam.h"

extern "C"
{
	RECOGNITION_DLL bool initRecogn(const WCHAR* AlgorithmName, const int NUM );
	RECOGNITION_DLL void releaseRecogn();
	RECOGNITION_DLL int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum
		, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo);
}

class RECOGNITION_DLL CInterFace
{
public:
	CInterFace(const WCHAR* AlgorithmName, const int NUM ); //AlgorithmName：使用的识别算法的名称
	~CInterFace();
	int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const WCHAR* strPath); //nCardNum：进行识别的是第几张牌，strSide：这张待识别的牌的东、西、南、北方位，nThreshold:二值化阈值，strPath：单张牌图片所在的路径
	int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo); //nCardNum：同上，strSide：同上,nThreshold：同上，pImg：图像数据的内存指针，nWidth：图宽，nHeight：图高
	int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const std::vector<WCHAR*>& strPathArr); //nCardNum：同上，strSide：同上,nThreshold：同上,strPathArr：单张牌对应的所有图片片段的路径
};


#endif