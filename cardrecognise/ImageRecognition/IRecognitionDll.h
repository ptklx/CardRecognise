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
	CInterFace(const WCHAR* AlgorithmName, const int NUM ); //AlgorithmName��ʹ�õ�ʶ���㷨������
	~CInterFace();
	int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const WCHAR* strPath); //nCardNum������ʶ����ǵڼ����ƣ�strSide�����Ŵ�ʶ����ƵĶ��������ϡ�����λ��nThreshold:��ֵ����ֵ��strPath��������ͼƬ���ڵ�·��
	int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, void* pImg, const int nNum, const int nWidth, const int nHeight, OutImageInfo* pDstImageInfo); //nCardNum��ͬ�ϣ�strSide��ͬ��,nThreshold��ͬ�ϣ�pImg��ͼ�����ݵ��ڴ�ָ�룬nWidth��ͼ��nHeight��ͼ��
	int startRecogn(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const std::vector<WCHAR*>& strPathArr); //nCardNum��ͬ�ϣ�strSide��ͬ��,nThreshold��ͬ��,strPathArr�������ƶ�Ӧ������ͼƬƬ�ε�·��
};


#endif