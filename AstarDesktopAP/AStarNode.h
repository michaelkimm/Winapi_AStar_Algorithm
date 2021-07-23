#pragma once

#ifndef ASTARNODE_H_
#define ASTARNODE_H_

#include <Windows.h>
#include <vector>
#include <stdio.h>
using namespace std;

class AStarNode
{
public:
	POINT pose_;
	int g_ = -1;
	int h_ = -1;
	int f_ = -1;

	bool obstackle_ = false;
	bool is_path_ = false;

	int width_;
	int height_;


public:
	AStarNode() {}
	AStarNode(POINT _pose, int _g, int _h);
	AStarNode(int _i, int _j, int _g, int _h);
	~AStarNode() {}

	POINT GetPose() const { return pose_; }

	AStarNode& operator=(const AStarNode& b);

	bool operator>(AStarNode b) const { return f_ > b.f_; }

	void Draw(HDC);
};

template <typename T>
class AStarAlgorithm
{
private:
	const int INF_INT_ = 2147480000;

	// A* 알고리즘 특성
	int look_ahead_size_ = 3;

	int straight_w_ = 10;
	int diag_w_ = 14;
	
	int dir_[8][2] = {
		{ -1, -1 },
		{ -1, 0 },
		{ -1, 1 },
		{ 0, -1 },
		{ 0, 1 },
		{ 1, -1 },
		{ 1, 0 },
		{ 1, 1 }
	};

public:
	AStarAlgorithm() {}
	~AStarAlgorithm() {}

	// A* 알고리즘으로 path 만들고, path에 저장
	int FindPath(vector<vector<T>>& _a_star_map, vector<POINT>& _path, const int sx, const int sy, const int ex, const int ey,
					int **_map, int _map_width, int _map_height);
	int FindPath(vector<vector<T>>& _a_star_map, vector<POINT>& _path, const POINT _start, const POINT _end, 
					int **_map, int _map_width, int _map_height);
	
	// 후보 노드에서 목적지 노드까지의 거리를 휴리스틱으로 계산. 여기서 휴리스틱은 직선 & 대각선 길이로 판단
	int GetHeuristicsDistance(const POINT& p1, const POINT& p2);

};



#include <queue>
#include <iostream>

#include "MyFigureDraw.h"
#include "MyGdiplus.h"

using namespace std;

AStarNode::AStarNode(POINT _pose, int _g, int _h)
	: pose_(_pose), g_(_g), h_(_h)
{
	f_ = _g + _h;
}
AStarNode::AStarNode(int _i, int _j, int _g, int _h)
	: pose_(POINT{ _i, _j }), g_(_g), h_(_h)
{
	f_ = _g + _h;
}

void AStarNode::Draw(HDC _hdc)
{
	POINT c1{ pose_.x * width_ + width_ / 2, pose_.y * height_ + height_ / 2 };
	DrawRectangle(_hdc, c1, width_, height_);					// 핸들, 중심점 위치, 가로, 세로
	//if (f_ != -1)
	//{
	//	RECT r1{ c1.x - width_ / 4, c1.y - height_ / 4, c1.x + width_ / 4, c1.y + height_ / 4 };

	//	// DrawInputText(_hdc, r1, _T(""));
	//}
}

AStarNode& AStarNode::operator=(const AStarNode& b)
{
	pose_ = b.pose_;
	g_ = b.g_;
	h_ = b.h_;
	f_ = b.f_;

	obstackle_ = b.obstackle_;
	width_ = b.width_;
	height_ = b.height_;

	return *this;
}

// : >> AStarAlgorithm

template <typename T>
int AStarAlgorithm<T>::FindPath(vector<vector<T>>& _a_star_map, vector<POINT>& _path, const int sx, const int sy, const int ex, const int ey, int **_map, int _map_width, int _map_height)
{
	return FindPath(POINT{ sx, sy }, POINT{ ex, ey });
}

template <typename T>
int AStarAlgorithm<T>::FindPath(vector<vector<T>>& _a_star_map, vector<POINT>& _path, const POINT _start, const POINT _end, int **_map, int _map_width, int _map_height)
{
	// : >> 맵 관련 함수

	// 장애물이 표시된 맵(int**형 포인터를 저장할 예정. 계산 편의를 위해 int*로 저장할 것)
	int* map_ = (int*)_map;
	int map_width_ = _map_width;
	int map_height_ = _map_height;

	// 방문 여부 저장하는 맵
	vector<vector<bool>> visited_map(map_width_, vector<bool>(map_height_, false));

	// 해당 정점으로 오기 전에 정점 위치 저장 맵
	vector<vector<POINT>> prev_walked(map_width_, vector<POINT>(map_height_, { -1, -1 }));
	
	// 도착점에 가장 가까울 것으로 예측되는 노드. 도착점에 가지 못하는 경우 쓰인다.
	T nearest_node;

	// 경로 찾았나?
	int found_path = 0;

	// <<

	// : >> 환경 변수 초기화
	// 맵 & h값 초기화
	for (int i = 0; i < map_height_; i++)
	{
		for (int j = 0; j < map_width_; j++)
		{
			// 모든 맵 초기화;
			_a_star_map[i][j].pose_ = POINT{ j, i };
			_a_star_map[i][j].g_ = INF_INT_;
			_a_star_map[i][j].f_ = INF_INT_;
			_a_star_map[i][j].h_ = INF_INT_;

			_a_star_map[i][j].is_path_ = false;
		}
	}
	// <<

	// 우선 순위 큐
	priority_queue<T, vector<T>, greater<T>> extract_min_heap;


	// : >> 시작 정점 초기화
	_a_star_map[_start.y][_start.x] = T{ _start.y, _start.x, 0, 0 };
	T cur_node{ _start, 0, GetHeuristicsDistance(_start, _end) };
	nearest_node = cur_node;
	// <<

	// 첫 노드 삽입
	extract_min_heap.push(cur_node);

	// 왓던 길 표시
	prev_walked[_start.y][_start.x] = cur_node.pose_;

	// <<

	// : >> 목표 노드에 도착 안했으면
	while (!extract_min_heap.empty())
	{
		// 현재 노드 추출 / 후보 경로중 가장 f값 작은 노드
		cur_node = extract_min_heap.top();
		extract_min_heap.pop();

		// 방문 표시
		visited_map[cur_node.pose_.y][cur_node.pose_.x] = true;

		// 목적지 도달 판단
		if ((cur_node.pose_.x == _end.x && cur_node.pose_.y == _end.y))
		{
			found_path = 1;
			break;
		}

		// : >> 현재 노드 주변 값 업데이트
		for (int i = 0; i < (pow(look_ahead_size_, 2) - 1); i++)
		{
			// 판단할 현재 노드 주변 위치 POINT
			POINT node_idx{ cur_node.pose_.x + dir_[i][0], cur_node.pose_.y + dir_[i][1] };

			// 맵 안인가?
			if (node_idx.x < 0 || node_idx.y < 0 || node_idx.x >= map_width_ || node_idx.y >= map_height_ )
				continue;

			// 장애물인가?
			if (map_[node_idx.y * map_height_ + node_idx.x] == 1)
				continue;

			// 방문하지 않은 정점이면서 정점 값을 업데이트할 수 있는 경우
			if (visited_map[node_idx.y][node_idx.x] == false)
			{
				T tmp_pose{ node_idx, _a_star_map[cur_node.pose_.y][cur_node.pose_.x].g_ + GetHeuristicsDistance(cur_node.pose_, node_idx),
									GetHeuristicsDistance(node_idx, _end) };

				// 후보 경로 타일로 갔을 때, 총 지나갈 길(f_)가 적은 것을 택한다.
				int haha = _a_star_map[tmp_pose.pose_.y][tmp_pose.pose_.x].f_;
				if (tmp_pose.g_ < _a_star_map[tmp_pose.pose_.y][tmp_pose.pose_.x].g_)
				{
					// 도착 못할 상황 대비하여 도착점과 가장 가까운 점 저장
					if (tmp_pose.h_ < nearest_node.h_)
						nearest_node = tmp_pose;

					// 노드 특성 업데이트
					_a_star_map[node_idx.y][node_idx.x] = tmp_pose;

					// bfs 방식 꺼내기 위해 후보 경로 저장
					extract_min_heap.push(tmp_pose);

					// 이전 거리 저장. BFS와 연관이 있는 것 같은데, 더 알아봐야겠다.
					prev_walked[node_idx.y][node_idx.x] = POINT{ cur_node.pose_.x , cur_node.pose_.y };
				}

			}

		}
	}
	// <<

	// 도착지에 도달 못한 경우
	if (!found_path)
	{
		cout << "도달 실패!!\n";
		cur_node = nearest_node;
	}

	// 이미 있던 길 비우기
	_path.clear();

	// :  >> 백 트랙킹
	POINT bt_idx = cur_node.pose_;
	while (!(bt_idx.x == prev_walked[bt_idx.y][bt_idx.x].x && bt_idx.y == prev_walked[bt_idx.y][bt_idx.x].y))	// 시작점이 아닐 경우 실행
	{
		_a_star_map[bt_idx.y][bt_idx.x].is_path_ = true;
		_path.emplace_back(bt_idx);
		bt_idx = prev_walked[bt_idx.y][bt_idx.x];
	}

	// 첫 점도 삽입
	_a_star_map[bt_idx.y][bt_idx.x].is_path_ = true;
	_path.emplace_back(bt_idx);
	// <<

	// 뒤집기
	std::reverse(_path.begin(), _path.end());

	return found_path;
}


template <typename T>
int AStarAlgorithm<T>::GetHeuristicsDistance(const POINT& p1, const POINT& p2)
{
	int dx = abs(p2.x - p1.x);
	int dy = abs(p2.y - p1.y);
	int bigger = dx > dy ? dx : dy;
	int smaller = dx > dy ? dy : dx;

	return  smaller * diag_w_ + straight_w_ * (bigger - smaller);
}




#endif












