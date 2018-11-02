#include <afx.h>
#include "ImgRecognition.h"

IRecognition::IRecognition()
{
	m_nCardNum = 0;
	wmemset(m_strSide, 0, sizeof(m_strSide));
	m_nThreshold = 50;
	m_bThresholdZip = false;
}

IRecognition::~IRecognition()
{
}

void IRecognition::SetParam(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const bool bThresholdZip)
{
	m_nCardNum = nCardNum;
	wcscpy_s(m_strSide, strSide);
	m_nThreshold = nThreshold;
	m_bThresholdZip = bThresholdZip;
}



