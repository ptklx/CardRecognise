#ifndef _IMGRECOGNITIONOUTPARAM_H
#define _IMGRECOGNITIONOUTPARAM_H


struct OutImageInfo
{
public:
	void* pOutDst;
	int nOutWidth;
	int nOutHeight;
	int nOutChannels;
	unsigned long nImgSize;
};


#endif