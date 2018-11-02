#include <afx.h>
#include <afxwin.h>
#include "ImageWindowTestTool.h"

//char* WideconvertMulti(CString src) //cstringת��Ϊchar*
//{
//	//USES_CONVERSION;
//	//char* pstr = T2A(strDataSend);//W2A(str)
//
//	int cstrLen = src.GetLength(); //���ַ�������ַ���
//	int pstrLen = WideCharToMultiByte(CP_ACP, 0, src, cstrLen, NULL, 0, 0, 0);//pstrlenΪ�ֽ���
//	char * pstr = new char[pstrLen + 1];
//	WideCharToMultiByte(CP_ACP, 0, src, cstrLen, pstr, pstrLen, 0, 0);
//	pstr[pstrLen] = '\0';
//
//	return pstr;
//}
//
//
//
//CString MulticonvertWide(char* src) //char*ת��Ϊcstring
//{
//	//USES_CONVERSION;
//	//CString temp = A2T(src); //A2W(src)
//
//	int charLen = strlen(src); //����char*�����С�����ֽ�Ϊ��λ��һ������ռ�����ֽ�
//	int wideLen = MultiByteToWideChar(CP_ACP, 0, src, charLen, NULL, 0); //������ֽ��ַ��Ĵ�С�����ַ�����
//	TCHAR *buf = new TCHAR[wideLen + 1]; //Ϊ���ֽ��ַ���������ռ䣬�����СΪ���ֽڼ���Ķ��ֽ��ַ���С
//	MultiByteToWideChar(CP_ACP, 0, src, charLen, buf, wideLen); //���ֽڱ���ת���ɿ��ֽڱ���
//	buf[wideLen] = '\0'; //����ַ�����β
//
//	CString temp = buf; //��TCHAR����ת��ΪCString temp.Append(buf);
//	delete[] buf;
//
//	return temp;
//}

char* WideconvertMulti(const WCHAR* WStr) //WCHAR*ת��Ϊchar*
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

WCHAR* MulticonvertWide(const char* CStr) //char*ת��ΪWCHAR*
{
	
	size_t len = strlen(CStr) + 1;
	size_t converted = 0;
	wchar_t *WStr;
	WStr = (wchar_t*)malloc(len*sizeof(wchar_t));
	mbstowcs_s(&converted, WStr, len, CStr, _TRUNCATE);
	return WStr;


}

//дʵʱ�����Ϣ��"ʵʱ�����־.txt"
void  WriteDebug(char* info )
{
	string setname = "debug.txt";
	string Save_Trainning_Data_Path = setname;
	FILE* fpout;
	int err = fopen_s(&fpout, Save_Trainning_Data_Path.c_str(), "a+");
	fputs("\r\n", fpout); //����
	fputs(info, fpout);
	fputs("\r\n", fpout); 
	fclose(fpout);
	
}
