#ifndef _IMGRECOGNITIONFACTORY_H
#define _IMGRECOGNITIONFACTORY_H

#include <vector>
#include "ImgRecoginitionOutParam.h"
#include "ImgRecognition.h"

class CStartAlgorithm
{
public:
	CStartAlgorithm();
	~CStartAlgorithm();
	int start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const WCHAR* strPath); //nCardNum：进行识别的是第几张牌，strSide：这张待识别的牌的东、西、南、北方位，nThreshold:二值化阈值，strPath：单张牌图片所在的路径
	int start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo); //nCardNum：同上，strSide：同上,nThreshold：同上，pImg：图像数据的内存指针，nWidth：图宽，nHeight：图高
	int start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const std::vector<WCHAR*>& strPathArr); //nCardNum：同上，strSide：同上,nThreshold：同上,strPathArr：单张牌对应的所有图片片段的路径
	bool SelectAlgorithm(const WCHAR* AlgorithmName,const int NUM = 0);
private:
	HANDLE m_hMutex;
	IRecognition* m_pRecon;
	IRecognition* m_pSecondClassifyRecon;
	WCHAR m_strAlgorithmName[128];
	BYTE* m_pDFEDst;
	BYTE* m_pHOGDst;
};


#endif