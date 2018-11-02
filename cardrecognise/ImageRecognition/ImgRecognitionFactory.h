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
	int start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const WCHAR* strPath); //nCardNum������ʶ����ǵڼ����ƣ�strSide�����Ŵ�ʶ����ƵĶ��������ϡ�����λ��nThreshold:��ֵ����ֵ��strPath��������ͼƬ���ڵ�·��
	int start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo); //nCardNum��ͬ�ϣ�strSide��ͬ��,nThreshold��ͬ�ϣ�pImg��ͼ�����ݵ��ڴ�ָ�룬nWidth��ͼ��nHeight��ͼ��
	int start(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const std::vector<WCHAR*>& strPathArr); //nCardNum��ͬ�ϣ�strSide��ͬ��,nThreshold��ͬ��,strPathArr�������ƶ�Ӧ������ͼƬƬ�ε�·��
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