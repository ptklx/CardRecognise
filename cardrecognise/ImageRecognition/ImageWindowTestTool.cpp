#include <afx.h>
#include <afxwin.h>
#include "ImageWindowTestTool.h"

//char* WideconvertMulti(CString src) //cstring转换为char*
//{
//	//USES_CONVERSION;
//	//char* pstr = T2A(strDataSend);//W2A(str)
//
//	int cstrLen = src.GetLength(); //按字符计算的字符数
//	int pstrLen = WideCharToMultiByte(CP_ACP, 0, src, cstrLen, NULL, 0, 0, 0);//pstrlen为字节数
//	char * pstr = new char[pstrLen + 1];
//	WideCharToMultiByte(CP_ACP, 0, src, cstrLen, pstr, pstrLen, 0, 0);
//	pstr[pstrLen] = '\0';
//
//	return pstr;
//}
//
//
//
//CString MulticonvertWide(char* src) //char*转换为cstring
//{
//	//USES_CONVERSION;
//	//CString temp = A2T(src); //A2W(src)
//
//	int charLen = strlen(src); //计算char*数组大小，以字节为单位，一个汉字占两个字节
//	int wideLen = MultiByteToWideChar(CP_ACP, 0, src, charLen, NULL, 0); //计算多字节字符的大小，按字符计算
//	TCHAR *buf = new TCHAR[wideLen + 1]; //为宽字节字符数组申请空间，数组大小为按字节计算的多字节字符大小
//	MultiByteToWideChar(CP_ACP, 0, src, charLen, buf, wideLen); //多字节编码转换成宽字节编码
//	buf[wideLen] = '\0'; //添加字符串结尾
//
//	CString temp = buf; //将TCHAR数组转换为CString temp.Append(buf);
//	delete[] buf;
//
//	return temp;
//}

char* WideconvertMulti(const WCHAR* WStr) //WCHAR*转换为char*
{
	/*memset(buf, 0, bufInLen);
	WideCharToMultiByte(CP_ACP, 0, src, -1,
		chr, length, NULL, NULL);
*/
	//wchar_t *WStr = L"string to convert";
	size_t len = wcslen(WStr) + 1;
	size_t converted = 0;
	char *CStr;
	CStr = (char*)malloc(len*sizeof(char));
	wcstombs_s(&converted, CStr, len, WStr, _TRUNCATE);
	return CStr;
}

WCHAR* MulticonvertWide(const char* CStr) //char*转换为WCHAR*
{
	
	size_t len = strlen(CStr) + 1;
	size_t converted = 0;
	wchar_t *WStr;
	WStr = (wchar_t*)malloc(len*sizeof(wchar_t));
	mbstowcs_s(&converted, WStr, len, CStr, _TRUNCATE);
	return WStr;


}

//写实时检测信息到"实时检测日志.txt"
void  WriteDebug(char* info )
{
	string setname = "debug.txt";
	string Save_Trainning_Data_Path = setname;
	FILE* fpout;
	int err = fopen_s(&fpout, Save_Trainning_Data_Path.c_str(), "a+");
	fputs("\r\n", fpout); //换行
	fputs(info, fpout);
	fputs("\r\n", fpout); 
	fclose(fpout);
	
}
