#ifndef _IMAGEWINDOWTESTTOOL_H
#define _IMAGEWINDOWTESTTOOL_H
#include <string>
using namespace std;
//extern char* WideconvertMulti(CString src); //cstringת��Ϊchar*
//extern CString MulticonvertWide(char* src); //char*ת��Ϊcstring
extern char* WideconvertMulti(const WCHAR* src); //WCHAR*ת��Ϊchar*
extern WCHAR* MulticonvertWide(const char* src); //char*ת��ΪWCHAR*
extern void  WriteDebug(char* info);
#endif