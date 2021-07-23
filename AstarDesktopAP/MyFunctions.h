#pragma once

#include "framework.h"
#include "Hole.h"
#include <vector>

class Machine;


// POINT pt가 polygon 내부에 있는지 확인하는 함수입니다.
bool IsInside(POINT pt, std::vector<POINT> &polygon);					// 일부 선 위의 점도 내부라고 판정
int MyIsPointInPoly(POINT& pose_, std::vector<POINT>& polygon_hole_);	// 선 위의 점은 내부라고 판정x

// POINT pt가 polygon 선분 위에 있는지 확인하는 함수입니다.
int MyIsPointOnPoly(POINT& pose_, std::vector<POINT> &polygon_hole_, bool think_as_poly = true);

// pt점이 선분p1-p2와 충돌하는지 확인하는 함수입니다.
int LinePointPt(POINT p1, POINT p2, POINT pt);
int LinePoint(float x1, float y1, float x2, float y2, float px, float py);

// 선분p1-p2가 선분p3-p4와 충돌하는지 확인하는 함수입니다.
int LineLinePt(POINT p1, POINT p2, POINT p3, POINT p4);
int LineLine(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);

// : >> 면적 구하기 위한 함수들입니다
// 2차원 벡터의 외적 값입니다
int ccw(const POINT & pa, const POINT & pb, const POINT & pc);
// 면적 구하기 함수입니다.
double GetArea(const std::vector<POINT>& polygon);


// 원 충돌함수
int DidCollide(Machine*, Machine*);

float PtDistance(POINT& p1, POINT& p2);
float PtDot(int x1, int y1, float vx, float vy);

bool CirclePoly(int cx, int cy, int r, std::vector<POINT>& polygon);
bool CircleLine(int cx, int cy, int r, int x1, int y1, int x2, int y2);
bool PointInCircle(int px, int py, int cx, int cy, int r);