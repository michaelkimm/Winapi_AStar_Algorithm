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

	// A* �˰��� Ư��
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

	// A* �˰������� path �����, path�� ����
	int FindPath(vector<vector<T>>& _a_star_map, vector<POINT>& _path, const int sx, const int sy, const int ex, const int ey,
					int **_map, int _map_width, int _map_height);
	int FindPath(vector<vector<T>>& _a_star_map, vector<POINT>& _path, const POINT _start, const POINT _end, 
					int **_map, int _map_width, int _map_height);
	
	// �ĺ� ��忡�� ������ �������� �Ÿ��� �޸���ƽ���� ���. ���⼭ �޸���ƽ�� ���� & �밢�� ���̷� �Ǵ�
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
	DrawRectangle(_hdc, c1, width_, height_);					// �ڵ�, �߽��� ��ġ, ����, ����
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
	// : >> �� ���� �Լ�

	// ��ֹ��� ǥ�õ� ��(int**�� �����͸� ������ ����. ��� ���Ǹ� ���� int*�� ������ ��)
	int* map_ = (int*)_map;
	int map_width_ = _map_width;
	int map_height_ = _map_height;

	// �湮 ���� �����ϴ� ��
	vector<vector<bool>> visited_map(map_width_, vector<bool>(map_height_, false));

	// �ش� �������� ���� ���� ���� ��ġ ���� ��
	vector<vector<POINT>> prev_walked(map_width_, vector<POINT>(map_height_, { -1, -1 }));
	
	// �������� ���� ����� ������ �����Ǵ� ���. �������� ���� ���ϴ� ��� ���δ�.
	T nearest_node;

	// ��� ã�ҳ�?
	int found_path = 0;

	// <<

	// : >> ȯ�� ���� �ʱ�ȭ
	// �� & h�� �ʱ�ȭ
	for (int i = 0; i < map_height_; i++)
	{
		for (int j = 0; j < map_width_; j++)
		{
			// ��� �� �ʱ�ȭ;
			_a_star_map[i][j].pose_ = POINT{ j, i };
			_a_star_map[i][j].g_ = INF_INT_;
			_a_star_map[i][j].f_ = INF_INT_;
			_a_star_map[i][j].h_ = INF_INT_;

			_a_star_map[i][j].is_path_ = false;
		}
	}
	// <<

	// �켱 ���� ť
	priority_queue<T, vector<T>, greater<T>> extract_min_heap;


	// : >> ���� ���� �ʱ�ȭ
	_a_star_map[_start.y][_start.x] = T{ _start.y, _start.x, 0, 0 };
	T cur_node{ _start, 0, GetHeuristicsDistance(_start, _end) };
	nearest_node = cur_node;
	// <<

	// ù ��� ����
	extract_min_heap.push(cur_node);

	// �Ӵ� �� ǥ��
	prev_walked[_start.y][_start.x] = cur_node.pose_;

	// <<

	// : >> ��ǥ ��忡 ���� ��������
	while (!extract_min_heap.empty())
	{
		// ���� ��� ���� / �ĺ� ����� ���� f�� ���� ���
		cur_node = extract_min_heap.top();
		extract_min_heap.pop();

		// �湮 ǥ��
		visited_map[cur_node.pose_.y][cur_node.pose_.x] = true;

		// ������ ���� �Ǵ�
		if ((cur_node.pose_.x == _end.x && cur_node.pose_.y == _end.y))
		{
			found_path = 1;
			break;
		}

		// : >> ���� ��� �ֺ� �� ������Ʈ
		for (int i = 0; i < (pow(look_ahead_size_, 2) - 1); i++)
		{
			// �Ǵ��� ���� ��� �ֺ� ��ġ POINT
			POINT node_idx{ cur_node.pose_.x + dir_[i][0], cur_node.pose_.y + dir_[i][1] };

			// �� ���ΰ�?
			if (node_idx.x < 0 || node_idx.y < 0 || node_idx.x >= map_width_ || node_idx.y >= map_height_ )
				continue;

			// ��ֹ��ΰ�?
			if (map_[node_idx.y * map_height_ + node_idx.x] == 1)
				continue;

			// �湮���� ���� �����̸鼭 ���� ���� ������Ʈ�� �� �ִ� ���
			if (visited_map[node_idx.y][node_idx.x] == false)
			{
				T tmp_pose{ node_idx, _a_star_map[cur_node.pose_.y][cur_node.pose_.x].g_ + GetHeuristicsDistance(cur_node.pose_, node_idx),
									GetHeuristicsDistance(node_idx, _end) };

				// �ĺ� ��� Ÿ�Ϸ� ���� ��, �� ������ ��(f_)�� ���� ���� ���Ѵ�.
				int haha = _a_star_map[tmp_pose.pose_.y][tmp_pose.pose_.x].f_;
				if (tmp_pose.g_ < _a_star_map[tmp_pose.pose_.y][tmp_pose.pose_.x].g_)
				{
					// ���� ���� ��Ȳ ����Ͽ� �������� ���� ����� �� ����
					if (tmp_pose.h_ < nearest_node.h_)
						nearest_node = tmp_pose;

					// ��� Ư�� ������Ʈ
					_a_star_map[node_idx.y][node_idx.x] = tmp_pose;

					// bfs ��� ������ ���� �ĺ� ��� ����
					extract_min_heap.push(tmp_pose);

					// ���� �Ÿ� ����. BFS�� ������ �ִ� �� ������, �� �˾ƺ��߰ڴ�.
					prev_walked[node_idx.y][node_idx.x] = POINT{ cur_node.pose_.x , cur_node.pose_.y };
				}

			}

		}
	}
	// <<

	// �������� ���� ���� ���
	if (!found_path)
	{
		cout << "���� ����!!\n";
		cur_node = nearest_node;
	}

	// �̹� �ִ� �� ����
	_path.clear();

	// :  >> �� Ʈ��ŷ
	POINT bt_idx = cur_node.pose_;
	while (!(bt_idx.x == prev_walked[bt_idx.y][bt_idx.x].x && bt_idx.y == prev_walked[bt_idx.y][bt_idx.x].y))	// �������� �ƴ� ��� ����
	{
		_a_star_map[bt_idx.y][bt_idx.x].is_path_ = true;
		_path.emplace_back(bt_idx);
		bt_idx = prev_walked[bt_idx.y][bt_idx.x];
	}

	// ù ���� ����
	_a_star_map[bt_idx.y][bt_idx.x].is_path_ = true;
	_path.emplace_back(bt_idx);
	// <<

	// ������
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












