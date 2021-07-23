#pragma once

#ifndef FIGUREDRAW_H_
#define FIGUREDRAW_H_

#include <iostream>
#include <windowsx.h>
#include <windows.h>
#include <WinUser.h>

void DrawGrid(HDC, POINT, POINT, UINT);									// 핸들, 시작 위치, 끝 위치, 선의 개수
void DrawGrid(HDC, POINT, UINT, UINT);									// 핸들, 중심 위치, 간격, 선의 개수
void DrawEllipse(HDC hdc, POINT c1, LONG r);							// 핸들, 중심 위치, 반지름
void DrawSunFlower(HDC hdc, POINT c1, LONG r1, LONG r2);				// 핸들, 기준 원 중심 위치, 기준 원 반지름, 바깥 원 반지름
void DrawSunFlowerN(HDC hdc, POINT c1, LONG r1, LONG n);				// 핸들, 기준 원 중심 위치, 기준 원 반지름, 바깥 원 반지름
void DrawRectangle(HDC hdc, POINT c1, LONG w, LONG h);					// 핸들, 중심점 위치, 가로, 세로
void DrawRectangle(HDC hdc, POINT c1, LONG w, LONG h, float tilt);		// 핸들, 중심점 위치, 가로, 세로, 회전한 각도
void DrawRectangle(HDC hdc, POINT *c1);									// 핸들, 중심점 위치, 가로, 세로, 회전한 각도
void DrawInputText(HDC hdc, RECT r1, LPCTSTR str,						// 핸들, 사각현, 문장, 캐럿 여부
	UINT Flags = DT_TOP | DT_LEFT | DT_SINGLELINE, bool Caret = false);
void DrawInputText(HDC hdc, POINT c, LONG w, LONG h,					
	LPCTSTR str, UINT Flags = DT_TOP | DT_LEFT | DT_SINGLELINE, bool Carett = false);	// 핸들, 중심점, 가로, 세로, 문장, 캐럿 여부
void DrawStar(HDC hdc, POINT c, LONG r);								// 핸들, 중심점, 반지름
void DrawStarN(HDC hdc, POINT c, LONG r, LONG n);						// 핸들, 중심점, 반지름, 바깥 꼭지점 개수
void arrowKeysColor(HDC hdc, POINT c, LONG d,							// 핸들, 중심점, 방향키 크기, 색깔, 방향키 값
	COLORREF crcolor, LONG keyVal);										// 핸들, 중심점, 방향키 크기, 색깔, 방향키 값
void getRectPoints(POINT c1, LONG w, LONG h, float tilt, POINT *pts);	// 핸들, 중심점 위치, 가로, 세로, 회전한 각도
template <typename T>
T pointDotProduct(POINT a, POINT b);									// 포인트 자료형 내적 값 반환

// input = 폴리곤1, 폴리곤2, 반환 받을 가장 짧은 거리, 반환 받을 가장 짧은 거리 방향
// output = 겹침 여부
// obj1, obj2 폴리곤 2개를 대입 시, 두 도형이 겹치는지 여부를 반환한다. 가장 짧은 반응 거리 및 방향을 매개변수로 넣은 변수로 얻을 수 있다.
bool satAlgorithm(POINT *object1Pt, POINT *object2Pt, float &shortest_distance, std::pair<float, float> &shortest_normal_vector);

bool doCirclesOverlap(float x1, float y1, float r1, float x2, float y2, float r2);		// 두 원의 중심점 및 반지름
float getfDistance(POINT p1, POINT p2);													// 포인트형 점 두개의 L2N
float getL2N(float a, float b);

#endif