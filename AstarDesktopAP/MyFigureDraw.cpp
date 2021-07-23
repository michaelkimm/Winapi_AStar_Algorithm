

#include <iostream>
#include <vector>
#include <tchar.h>

#include "MyFigureDraw.h"

#ifndef PI
#define PI 3.141592
#endif // !PI
void DrawGrid(HDC hdc, POINT p1, POINT p2, UINT n)	// 시작 위치, 끝 위치, 선의 개수
{
	LONG dx = (p2.x - p1.x) / n; // 가로 칸 개수
	LONG dy = (p2.y - p1.y) / n; // 세로 칸 개수

	// 가로 선 그리기
	for (LONG j = p1.y; j <= p2.y; j += dy)
	{
		MoveToEx(hdc, p1.x, j, NULL);
		LineTo(hdc, p2.x, j);
	}
	// 세로 선 그리기
	for (LONG i = p1.x; i <= p2.x; i += dx)
	{
		MoveToEx(hdc, i, p1.y, NULL);
		LineTo(hdc, i, p2.y);
	}
}
void DrawGrid(HDC hdc, POINT c1, UINT dx, UINT n)		// 중심 위치, 간격, 선의 개수
{
	LONG d = (n / 2) * dx;
	POINT p1{ c1.x - d, c1.y - d };
	POINT p2{ c1.x + d, c1.y + d };
	DrawGrid(hdc, p1, p2, n);							// 중심 위치와 간격을 알면, 시작 위치와 끝 위치를 알 수 있어요. 이를 이용해서 DrawGrid 위 함수를 재사용 했습니다.
}

void DrawEllipse(HDC hdc, POINT c1, LONG r)
{
	Ellipse(hdc, c1.x - r, c1.y - r, c1.x + r, c1.y + r);
}

void DrawSunFlower(HDC hdc, POINT c1, LONG r1, LONG r2)		// 외곽원 반지름으로 그리기 해바라기 그리기
{
	// n_out 구하기
	LONG n_out = (LONG)(round(180 / (asin((double)r2 / (r1 + r2)) * (180 / PI))));	// 기준 원 곁에 원이 몇개가 생기나? 기하학적 관계 이용했습니다. sin역함수가 사용되네요

	// 작은 원 그리기
	for (LONG theta = 0; theta < 360; theta += (360 / n_out))						// 기준 원 곁에 있는 원들의 중심이 r1+r2반지름을 가지는 원임에서 착안했습니다. 원의 모든 점은 sin cos으로 표현됩니다.
	{
		LONG x2{ (LONG)(c1.x + (double)(r1 + r2) * cos(theta * PI / 180)) };
		LONG y2{ (LONG)(c1.y + (double)(r1 + r2) * sin(theta * PI / 180)) };
		DrawEllipse(hdc, POINT{ x2,y2 }, r2);
	}
}

void DrawSunFlowerN(HDC hdc, POINT c1, LONG r1, LONG n)		// 외곽원 갯수로 그리기 해바라기 그리기
{
	// r2 구하기
	double s = sin(360 / n / 2 * PI / 180);
	LONG r2 = (LONG)(round((r1 * (sin(s) / (1 - sin(s))))));						// DrawEllipseFlower에서 n_out구하는 공식을 변형하면 되요

	// 작은 원 그리기
	for (LONG theta = 0; theta < 360; theta += (360 / n))						// 기준 원 곁에 있는 원들의 중심이 r1+r2반지름을 가지는 원임에서 착안했습니다. 원의 모든 점은 sin cos으로 표현됩니다.
	{
		LONG x2{ (LONG)(c1.x + (double)(r1 + r2) * cos(theta * PI / 180)) };
		LONG y2{ (LONG)(c1.y + (double)(r1 + r2) * sin(theta * PI / 180)) };
		DrawEllipse(hdc, POINT{ x2,y2 }, r2);
	}
}

void DrawRectangle(HDC hdc, POINT c1, LONG w, LONG h)		// 핸들, 중심점 위치, 가로, 세로
{
	Rectangle(hdc, c1.x - w / 2, c1.y - h / 2, c1.x + w / 2, c1.y + h / 2);
}

void DrawRectangle(HDC hdc, POINT c1, LONG w, LONG h, float tilt)		// 핸들, 중심점 위치, 가로, 세로, 회전한 각도
{
	// p1234는 원점에서의 사각형 위치 pts는 translation & rotation 변환 후 네점 위치 저장 공간
	POINT p1234[4] = { -w / 2, -h / 2, w / 2, -h / 2, w / 2, h / 2, -w / 2, h / 2 };
	POINT pts[4] = { NULL, };
	// 변환 행렬 적용
	getRectPoints(c1, w, h, tilt, pts);
	// 그리기
	DrawRectangle(hdc, pts);
}
void DrawRectangle(HDC hdc, POINT *c1)									// 핸들, 중심점 위치, 가로, 세로, 회전한 각도
{
	int i;
	for (i = 0; i < 3; i++)
	{
		MoveToEx(hdc, c1[i].x, c1[i].y, NULL);
		LineTo(hdc, c1[i + 1].x, c1[i + 1].y);
	}
	MoveToEx(hdc, c1[i].x, c1[i].y, NULL);
	LineTo(hdc, c1[0].x, c1[0].y);
}
void DrawInputText(HDC hdc, RECT r1, LPCTSTR str, UINT Flags, bool Caret)
{
	// 사각형 그리기
	POINT c1{ (r1.left + r1.right) / 2, (r1.top + r1.bottom) / 2 };
	LONG w{ r1.right - r1.left };
	LONG h{ r1.bottom - r1.top };
	DrawRectangle(hdc, c1, w, h);

	// 텍스트 그리기
	LONG gap = 2;
	RECT r2{ r1.left + gap, r1.top + gap, r1.right - gap, r1.bottom - gap };	// 캐럿 크기 생각해서 작게 사각형 생성
		// 텍스트 캐럿 설정
	if (Caret == true)
	{
		SIZE size;
		GetTextExtentPoint(hdc, str, _tcslen(str), &size);
		SetCaretPos(r2.left + size.cx, r1.top);
	}
	// 텍스트 그리기
	DrawText(hdc, str, _tcslen(str), &r2, Flags);
}

void DrawInputText(HDC hdc, POINT c, LONG w, LONG h, LPCTSTR str, UINT Flags, bool Caret)
{
	// 사각형 그리기
	DrawRectangle(hdc, c, w, h);

	// 텍스트 그리기
	LONG gap = 2;
	RECT r2{ c.x - w / 2 + gap, c.y - h / 2 + gap, c.x + w / 2 - gap, c.y + h / 2 - gap };	// 캐럿 크기 생각해서 작게 사각형 생성
		// 텍스트 캐럿 설정
	if (Caret == true)
	{
		SIZE size;
		GetTextExtentPoint(hdc, str, _tcslen(str), &size);
		SetCaretPos(c.x - w / 2 + size.cx, c.y - h / 2);
	}
	// 텍스트 그리기
	DrawText(hdc, str, _tcslen(str), &r2, Flags);
}

void DrawStar(HDC hdc, POINT c, LONG r)
{
	double double_d;	// 중심에서 점까지 거리
	POINT point[10];	// 바깥 꼭지점 및 안 꼭지점 집합
	LONG pointIdx = 0;	// 꼭지점 인덱스
	int theta = 90 - 2 * (360 / (2 * 5));
	double theta2rad = PI / 180;

	double short_d = r * sin(theta * theta2rad) / cos(theta * theta2rad);	// 중심에서 안쪽 꼭지점까지 거리
	for (theta = 0; theta < 360; theta += (360 / 10))
	{
		// 각도에 따라 달라지는 d 설정
		if (theta / (360 / 10) % 2 == 1)
			double_d = short_d;
		else
			double_d = r;

		// 바깥 꼭지점 및 안 꼭지점 설정
		double x = cos(theta * theta2rad) * double_d + c.x;		// 평행이동 행렬 & 회전 행렬을 생각하셔도 되고, 벡터로 생각하셔도 됩니다.
		double y = sin(theta * theta2rad) * double_d + c.y;
		point[pointIdx++] = { (LONG)x, (LONG)y };
	}

	// 별 그리기
	Polygon(hdc, point, 10);
}


void DrawStarN(HDC hdc, POINT c, LONG r, LONG n)
{
	double double_d;						// 중심에서 점까지 거리
	POINT * point = new POINT[2 * n];		// 바깥 꼭지점 및 안 꼭지점 집합
	LONG pointIdx = 0;						// 꼭지점 인덱스
	int theta = 90 - 2 * (360 / (2 * n));	// 꼭지점 사이 각도
	double theta2rad = PI / 180;

	// 각도가 커지면서 꼭지점 위치를 하나씩 point 배열에 대입
	double short_d = r * sin(theta * theta2rad) / cos(theta * theta2rad);	// 중심에서 안쪽 꼭지점까지 거리
	for (theta = 0; theta < 360; theta += (360 / (2 * n)))
	{
		// 각도에 따라 달라지는 d 설정
		if (theta / (360 / (2 * n)) % 2 == 1)
			double_d = short_d;
		else
			double_d = r;

		// 바깥 꼭지점 및 안 꼭지점 설정
		double x = cos(theta * theta2rad) * double_d + c.x;		// 평행이동 행렬 & 회전 행렬을 생각하셔도 되고, 벡터로 생각하셔도 됩니다.
		double y = sin(theta * theta2rad) * double_d + c.y;
		point[pointIdx++] = { (LONG)x, (LONG)y };
	}

	// 별 그리기
	Polygon(hdc, point, 2 * n);

	delete[] point;
}

void arrowKeysColor(HDC hdc, POINT c, LONG d, COLORREF crcolor, LONG keyVal)
{
	POINT left_c = { c.x - d, c.y };
	POINT top_c = { c.x, c.y - d };
	POINT right_c = { c.x + d, c.y };

	// 방향키 4개 그리기
	DrawInputText(hdc, top_c, d, d, _T("Up"), DT_CENTER | DT_VCENTER | DT_SINGLELINE);
	DrawInputText(hdc, c, d, d, _T("Down"), DT_CENTER | DT_VCENTER | DT_SINGLELINE);
	DrawInputText(hdc, left_c, d, d, _T("Left"), DT_CENTER | DT_VCENTER | DT_SINGLELINE);
	DrawInputText(hdc, right_c, d, d, _T("Right"), DT_CENTER | DT_VCENTER | DT_SINGLELINE);


	// keyVal != 0 이면
		// 그 위에 빨강 그리기
	if (keyVal != 0)
	{
		HBRUSH hBrush = CreateSolidBrush(RGB(0, 0, 255));
		HBRUSH oldBrush = (HBRUSH)SelectObject(hdc, hBrush);
		switch (keyVal)
		{
		case VK_LEFT:
			DrawRectangle(hdc, left_c, d, d);
			break;
		case VK_RIGHT:
			DrawRectangle(hdc, right_c, d, d);
			break;
		case VK_UP:
			DrawRectangle(hdc, top_c, d, d);
			break;
		case VK_DOWN:
			DrawRectangle(hdc, c, d, d);
			break;
		default:
		{
		}
		}
		SelectObject(hdc, oldBrush);
		DeleteObject(hBrush);
	}
}

void getRectPoints(POINT c1, LONG w, LONG h, float tilt, POINT *pts)		// 핸들, 중심점 위치, 가로, 세로, 회전한 각도
{
	// p1234는 원점에서의 사각형 위치 pts는 translation & rotation 변환 후 네점 위치 저장 공간
	POINT p1234[4] = { -w / 2, -h / 2, w / 2, -h / 2, w / 2, h / 2, -w / 2, h / 2 };


	// 변환 행렬 적용
	for (int i = 0; i < 4; i++)
	{
		POINT pt{ NULL, };
		pts[i].x = (long)(p1234[i].x * cos((double)tilt) - p1234[i].y * sin((double)tilt) + c1.x);
		pts[i].y = (long)(p1234[i].x * sin((double)tilt) + p1234[i].y * cos((double)tilt) + c1.y);
	}
}

template <typename T>
T pointDotProduct(POINT a, POINT b)
{
	return a.x * b.x + a.y * b.y;
}

bool satAlgorithm(POINT *object1Pt, POINT *object2Pt, float &shortest_distance, std::pair<float, float> &shortest_normal_vector)
{
	// 두 벡터로 겹치는지 여부 판단
			// 노멀 벡터 구하기

	// obj1 각 변의 법선 벡터
	std::vector<std::pair<float, float>> norms;
	float fdistance = 0.f;			// 법선 단위 벡터 만드는 스칼라 값
	for (int i = 0; i < 3; i++)
	{
		fdistance = getfDistance(object1Pt[i], object1Pt[i + 1]);
		norms.emplace_back(std::pair<float, float>{ -(object1Pt[i + 1].y - object1Pt[i].y) / fdistance, (object1Pt[i + 1].x - object1Pt[i].x) / fdistance });
	}
	fdistance = getfDistance(object1Pt[0], object1Pt[3]);
	norms.emplace_back(std::pair<float, float>{ -(object1Pt[0].y - object1Pt[3].y) / fdistance, (object1Pt[0].x - object1Pt[3].x) / fdistance });

	// obj2 각 변의 법선 벡터
	for (int i = 0; i < 3; i++)
	{
		fdistance = getfDistance(object2Pt[i], object2Pt[i + 1]);
		norms.emplace_back(std::pair<float, float>{ -(object2Pt[i + 1].y - object2Pt[i].y) / fdistance, (object2Pt[i + 1].x - object2Pt[i].x) / fdistance });
	}
	fdistance = getfDistance(object2Pt[0], object2Pt[3]);
	norms.emplace_back(std::pair<float, float>{ -(object2Pt[0].y - object2Pt[3].y) / fdistance, (object2Pt[0].x - object2Pt[3].x) / fdistance });

	// 각 노멀 벡터에 정사영한 값 구하기
		// 겹치는지 판단
		// 각 도형의 최대 최소 값 구하기
		// (최대 - 최소), (최소 - 최대) 중 절대값 작은 것이 최소 거리
		// 그 벡터도 저장

	auto pairFloat2DotProduct = [](POINT p1, std::pair<float, float> v1)	// 내적 인라인 함수
	{
		return p1.x * v1.first + p1.y * v1.second;
	};

	// 각 법선 벡터에 대해 obj들의 점 정사영
	float min_abs = 3.4E+37f;
	for (std::pair<float, float> &norm : norms)
	{
		// 해당 법선 벡터에 대해 obj1 정사영
		float proj1 = pairFloat2DotProduct(object1Pt[0], norm);
		float obj1_proj_max = proj1, obj1_proj_min = proj1;
		for (int i = 1; i < 4; i++)
		{
			proj1 = pairFloat2DotProduct(object1Pt[i], norm);
			if (proj1 >= obj1_proj_max) obj1_proj_max = proj1;				// 최대 최소 판정 및 업데이트
			if (proj1 <= obj1_proj_min) obj1_proj_min = proj1;
		}

		// 해당 법선 벡터에 대해 obj2 정사영
		proj1 = pairFloat2DotProduct(object2Pt[0], norm);
		float obj2_proj_max = proj1, obj2_proj_min = proj1;
		for (int i = 1; i < 4; i++)
		{
			proj1 = pairFloat2DotProduct(object2Pt[i], norm);
			if (proj1 >= obj2_proj_max) obj2_proj_max = proj1;				// 최대 최소 판정 및 업데이트
			if (proj1 <= obj2_proj_min) obj2_proj_min = proj1;
		}

		if (obj1_proj_min <= obj2_proj_max && obj1_proj_max >= obj2_proj_min)
		{
			// 겹치다
			// 최소 절대값 구하기
			fabs(obj2_proj_max - obj1_proj_min) <= fabs(obj1_proj_max - obj2_proj_min) ?
				min_abs = fabs(obj2_proj_max - obj1_proj_min) : min_abs = fabs(obj1_proj_max - obj2_proj_min);		// 현재 최소 절대값 업데이트

			if (min_abs <= shortest_distance)	// 이전 최소 절대값과 비교 후 최소값 및 법선 벡터 업데이트
			{
				shortest_distance = min_abs;
				shortest_normal_vector = norm;
			}
		}
		else
			return false;	// 안겹침
	}
	// 겹침
	return true;
}

bool doCirclesOverlap(float x1, float y1, float r1, float x2, float y2, float r2)
{
	return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) <= (r1 + r2)*(r1 + r2);
}

float getfDistance(POINT p1, POINT p2)
{
	return sqrtf((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

float getL2N(float a, float b)
{
	return sqrtf(a*a + b * b);
}