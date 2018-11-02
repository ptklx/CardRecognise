#ifndef _IMGRECOGNITION_H
#define _IMGRECOGNITION_H
//#include <cstddef>         //常用常量
//#include <cstdio> 　　　　 //定义输入／输出函数
//#include <iostream> 　　　//数据流输入／输出
//#include <cwchar> 　　　　 //宽字符处理及输入／输出
#include <afx.h>
//#include "opencv2/opencv.hpp"

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
//#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/photo/photo.hpp"
//#include "opencv2/video/video.hpp"
//#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
//#include "ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

/**********
#pragma comment( lib, "lib/opencv_core2410.lib" )
#pragma comment( lib, "lib/opencv_imgproc2410.lib" )
#pragma comment( lib, "lib/opencv_objdetect2410.lib" )
#pragma comment( lib, "lib/opencv_highgui2410.lib" )
#pragma comment( lib, "lib/opencv_ml2410.lib" )


#pragma comment( lib, "lib/opencv_core2410d.lib" )
#pragma comment( lib, "lib/opencv_imgproc2410d.lib" )
#pragma comment( lib, "lib/opencv_objdetect2410d.lib" )
#pragma comment( lib, "lib/opencv_highgui2410d.lib" )
#pragma comment( lib, "lib/opencv_ml2410d.lib" )

***********/

#include <vector>

using namespace cv;

class  IRecognition //识别算法的虚基类
{
public:
	IRecognition();
	virtual ~IRecognition();
	virtual int StartRecon(std::vector<Mat>& ImgArr) = 0;

	void SetParam(const int nCardNum, const WCHAR* strSide, const unsigned char nThreshold, const bool bThresholdZip = false);
protected:
	int m_nCardNum;
	WCHAR m_strSide[128];
	unsigned char m_nThreshold;
	bool m_bThresholdZip; //是否有进行二值化压缩
};

#endif