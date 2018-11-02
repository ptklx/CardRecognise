#ifndef _IMAGEWINDOWTESTTOOL_H
#define _IMAGEWINDOWTESTTOOL_H
#include <string>
using namespace std;
//extern char* WideconvertMulti(CString src); //cstring转换为char*
//extern CString MulticonvertWide(char* src); //char*转换为cstring
extern char* WideconvertMulti(const WCHAR* src); //WCHAR*转换为char*
extern WCHAR* MulticonvertWide(const char* src); //char*转换为WCHAR*
extern void  WriteDebug(char* info);
#endif