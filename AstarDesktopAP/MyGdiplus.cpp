#include "MyGdiplus.h"

ULONG_PTR g_GdiToken;

void Gdi_Init()
{
	GdiplusStartupInput gpsi;
	GdiplusStartup(&g_GdiToken, &gpsi, NULL);
}

void Gdi_Draw(HDC hdc)
{
	Graphics graphics(hdc);

	// : >> 배경 그리기
	Image img(L"images/maple_background.bmp");
	int w = img.GetWidth();
	int h = img.GetHeight();
	graphics.DrawImage(&img, 0, 0, w, h);
	// <<
}

void Gdi_End()
{
	GdiplusShutdown(g_GdiToken);
}