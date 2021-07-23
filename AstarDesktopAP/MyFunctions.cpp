#include "MyFunctions.h"
#include <math.h>

#include <iostream>
using namespace std;

// POINT pt가 polygon 내부에 있는지 확인하는 함수입니다.
bool IsInside(POINT pt, std::vector<POINT> &polygon)
{
	int crosses = 0;
	int j;
	float cross_pt_x;
	for (int i = 0; i < polygon.size(); i++)
	{
		j = (i + 1) % polygon.size();
		// pt가 polygon의 해당 선분의 y좌표 사이에 있을 때
		if (!((polygon[i].y >= pt.y) != (polygon[j].y >= pt.y)))
			continue;
		// pt.x가 해당 선분과 교차하는 점의 x좌표를 cross_pt라 하자
		cross_pt_x = (pt.y - polygon[i].y) * (polygon[j].x - polygon[i].x) / 1.0f / (polygon[j].y - polygon[i].y) + polygon[i].x;
		// cross_pt.x > pt.x면 교차
		if (cross_pt_x > pt.x)
			crosses++;
	}
	return crosses % 2 > 0;
}

int MyIsPointInPoly(POINT& pose_, std::vector<POINT>& polygon_hole_)
{
	
	int collision = false;
	int next = 0;
	for (int current = 0; current < polygon_hole_.size(); current++)
	{
		next = current + 1;
		if (next == polygon_hole_.size()) next = 0;

		POINT vc = polygon_hole_[current];
		POINT vn = polygon_hole_[next];

		if (((vc.y >= pose_.y && vn.y < pose_.y) || (vc.y < pose_.y && vn.y >= pose_.y)) &&
			(pose_.x < (vn.x - vc.x)*(pose_.y - vc.y) / (vn.y - vc.y) + vc.x))
		{
			collision = !collision;
		}
	}
	return collision;
}

// POINT pt가 polygon 선분 위에 있는지 확인하는 함수입니다.
int MyIsPointOnPoly(POINT& pose_, std::vector<POINT> &polygon_hole_, bool think_as_poly)
{
	int collision = false;
	int next = 0;
	for (int current = 0; current < polygon_hole_.size(); current++)
	{
		next = current + 1;
		if (next == polygon_hole_.size())
		{
			if (think_as_poly)
				next = 0;
			else
				continue;
		}

		if (LinePoint(polygon_hole_[current].x, polygon_hole_[current].y,
			polygon_hole_[next].x, polygon_hole_[next].y, pose_.x, pose_.y))
		{
			collision = !collision;
			return collision;
		}
	}
	return collision;
}


// pt점이 선분p1-p2와 충돌하는지 확인하는 함수입니다.
int LinePointPt(POINT p1, POINT p2, POINT pt) {

	return LinePoint(p1.x, p1.y, p2.x, p2.y, pt.x, pt.y);
}

int LinePoint(float x1, float y1, float x2, float y2, float px, float py) {

	// get distance from the point to the two ends of the line
	float d1 = sqrtf((px - x1)*(px - x1) + (py - y1)*(py - y1));
	float d2 = sqrtf((px - x2)*(px - x2) + (py - y2)*(py - y2));

	// get the length of the line
	float lineLen = sqrtf((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));

	// since floats are so minutely accurate, add
	// a little buffer zone that will give collision
	float buffer = 0.1f;    // higher # = less accurate

	// if the two distances are equal to the line's 
	// length, the point is on the line!
	// note we use the buffer here to give a range, 
	// rather than one #
	if (d1 + d2 >= lineLen - buffer && d1 + d2 <= lineLen + buffer) {
		return true;
	}
	return false;
}


// 선분p1-p2가 선분p3-p4와 충돌하는지 확인하는 함수입니다,
int LineLinePt(POINT p1, POINT p2, POINT p3, POINT p4)
{
	return LineLine(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);
}

int LineLine(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

	// calculate the distance to intersection point
	float uA = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));
	float uB = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));

	// if uA and uB are between 0-1, lines are colliding
	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
		return true;
	}
	return false;
}


// : >> 면적 구하기 위한 함수들입니다
// 2차원 벡터의 외적 값입니다
int ccw(const POINT & pa, const POINT & pb, const POINT & pc)
{
	return (pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x);
}
// 면적 구하기 함수입니다.
double GetArea(const std::vector<POINT>& polygon)
{
	double sum = 0;

	if (polygon.size() < 3)
		return sum;
	
	for (int i = 0; i < polygon.size() - 2; i++)
	{
		sum += ccw(polygon[0], polygon[i + 1], polygon[i + 2]);
	}

	return fabs(sum) / 2;
}


// 원 충돌함수
int DidCollide(Machine *machine_a, Machine *machine_b)
{
	// 머신a가 머신b와 충돌했으면 true 반환, 안했으면 1반환.
	

	return 0;
}

float PtDistance(POINT & p1, POINT & p2)
{
	return sqrtf((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

float PtDot(int x1, int y1, float vx, float vy)
{
	return x1 * vx + y1 * vy;
}

bool CirclePoly(int cx, int cy, int r, std::vector<POINT>& polygon)
{
	int next;
	for (int current = 0; current < polygon.size(); current++)
	{
		next = (current + 1) % polygon.size();
		
		if (CircleLine(cx, cy, r, polygon[current].x, polygon[current].y, polygon[next].x, polygon[next].y))
			return true;
	}
	return false;
}

bool CircleLine(int cx, int cy, int r, int x1, int y1, int x2, int y2)
{
	bool inside1 = PointInCircle(x1, y1, cx, cy, r);
	bool inside2 = PointInCircle(x2, y2, cx, cy, r);

	// 점이 원 안에 있으면 true
	if (inside1 || inside2)
		return true;

	float distX = x2 - x1;
	float distY = y2 - y1;
	float len = sqrtf((distX * distX) + (distY * distY));

	float dot = (((cx - x1) * (x2 - x1)) + ((cy - y1) * (y2 - y1))) / pow(len, 2);

	float closestX = x1 + (dot * (x2 - x1));
	float closestY = y1 + (dot * (y2 - y1));

	// 내적 점이 선분 위에 없으면 false
	bool onSegment = LinePoint(x1, y1, x2, y2, closestX, closestY);
	if (!onSegment)
		return false;

	// 내적 점과 중심 사이 거리
	distX = closestX - cx;
	distY = closestY - cy;

	// 가장 가까운 내적 점과 중심점 사이 거리
	float pt_distance = sqrtf((distX * distX) + (distY * distY));
	if (pt_distance <= r)
		return true;

	return false;
}

bool PointInCircle(int px, int py, int cx, int cy, int r)
{
	return ((px - cx) * (px - cx) + (py - cy) * (py - cy)) <= (r * r);
}
