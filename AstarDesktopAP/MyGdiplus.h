#pragma once
#ifndef MYGDIPLUS_H_
#define MYGDIPLUS_H_

// : >> Gdi+
#include <ObjIdl.h>
#include <gdiplus.h>
#pragma comment(lib, "Gdiplus.lib")
using namespace Gdiplus;

void Gdi_Init();
void Gdi_Draw(HDC);
void Gdi_End();

#endif

