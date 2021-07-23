#include "AStarNode.h"
#include <vector>
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
	DrawRectangle(_hdc, POINT{ pose_.x + width_ / 2, pose_.y + height_ / 2 }, width_, height_);					// �ڵ�, �߽��� ��ġ, ����, ����
}

void AStarTile::Draw(HDC _hdc)
{
	DrawRectangle(_hdc, POINT{ pose_.x + width_ / 2, pose_.y + height_ / 2}, width_, height_);					// �ڵ�, �߽��� ��ġ, ����, ����
}


//AStarNode& AStarNode::operator=(const AStarNode& b)
//{
//	pose_ = b.pose_;
//	g_ = b.g_;
//	h_ = b.h_;
//	f_ = b.f_;
//
//	return *this;
//}

// : >> AStarAlgorithm

// ������ & �ı���
template <typename T>
AStarAlgorithm<T>::AStarAlgorithm(int **_map, int _map_width, int _map_height) : map_((int*)_map), map_width_(_map_width), map_height_(_map_height)
{

}

// �ʱ�ȭ �Լ�(���� ���� �ʱ�ȭ)
template <typename T>
void AStarAlgorithm<T>::SetMap(int **_map, int _map_width, int _map_height)
{
	map_ = (int*)_map;
	map_width_ = _map_width;
	map_height_ = _map_height;
}

template <typename T>
vector<vector<T>> AStarAlgorithm<T>::FindPath(int sx, int sy, int ex, int ey)
{
	return FindPath(POINT{ sx, sy }, POINT{ ex, ey });
}

template <typename T>
vector<vector<T>> AStarAlgorithm<T>::FindPath(POINT _start, POINT _end)
{
	// : >> �� ���� �Լ�

	// AStarNode�� ������ 2���� ���� ��
	vector<vector<T>> a_star_map(map_width_, vector<T>(map_height_, T{ -1, -1, -1, -1 }));

	// �湮 ���� �����ϴ� ��
	vector<vector<bool>> visited_map(map_width_, vector<bool>(map_height_, false));

	// �ش� �������� ���� ���� ���� ��ġ ���� ��
	vector<vector<POINT>> prev_walked(map_width_, vector<POINT>(map_height_, { -1, -1 }));

	// <<

	// : >> ȯ�� ���� �ʱ�ȭ
	// �� & h�� �ʱ�ȭ
	for (int i = 0; i < map_height_; i++)
	{
		for (int j = 0; j < map_width_; j++)
		{
			// ��� �� �ʱ�ȭ;
			visited_map[i][j] = false;

			a_star_map[i][j].pose_ = POINT{ i, j };

			a_star_map[i][j].g_ = INF_INT_;
			a_star_map[i][j].f_ = INF_INT_;

			prev_walked[i][j].x = -1;
			prev_walked[i][j].y = -1;
		}
	}
	// <<

	// �켱 ���� ť
	priority_queue<T, vector<T>, greater<T>> extract_min_heap;


	// : >> ���� ���� �ʱ�ȭ
	a_star_map[_start.x][_start.y] = T{ _start.x, _start.y, 0, 0 };
	T node{ _start, 0, GetHeuristicsDistance(_start, _end) };

	// ù ��� ����
	extract_min_heap.push(node);

	// �Ӵ� �� ǥ��
	prev_walked[_start.x][_start.y] = node.pose_;

	// <<

	T cur_node{ 0, 0, 0, 0 };

	// : >> ��ǥ ��忡 ���� ��������
	while (1)
	{
		// ���� ��� ���� / �ĺ� ����� ���� f�� ���� ���
		cur_node = extract_min_heap.top();
		extract_min_heap.pop();

		// �湮 ǥ��
		visited_map[cur_node.pose_.x][cur_node.pose_.y] = true;

		// ������ ���� �Ǵ�
		if ((cur_node.pose_.x == _end.x && cur_node.pose_.y == _end.y))
			break;

		// : >> ���� ��� �ֺ� �� ������Ʈ
		for (int i = 0; i < (pow(look_ahead_size_, 2) - 1); i++)
		{
			POINT node_idx{ cur_node.pose_.x + dir_[i][0], cur_node.pose_.y + dir_[i][1] };

			// �� ���ΰ�?
			if (node_idx.x < 0 || node_idx.y < 0 || node_idx.x >= (pow(look_ahead_size_, 2) - 1) || node_idx.y >= (pow(look_ahead_size_, 2) - 1))
				continue;

			// ��ֹ��ΰ�?
			if (map_[node_idx.x * map_height_ + node_idx.y] == 1)
			{
				a_star_map[node_idx.x][node_idx.y].obstackle_ = true;
				continue;
			}

			// �湮���� ���� �����̸鼭 ���� ���� ������Ʈ�� �� �ִ� ���
			if (visited_map[node_idx.x][node_idx.y] == false)
			{
				T tmp_pose{ node_idx, a_star_map[cur_node.pose_.x][cur_node.pose_.y].g_ + GetHeuristicsDistance(cur_node.pose_, node_idx),
									GetHeuristicsDistance(node_idx, _end) };

				// �ĺ� ��� Ÿ�Ϸ� ���� ��, �� ������ ��(f_)�� ���� ���� ���Ѵ�.
				int haha = a_star_map[tmp_pose.pose_.x][tmp_pose.pose_.y].f_;
				if (tmp_pose.f_ < a_star_map[tmp_pose.pose_.x][tmp_pose.pose_.y].f_)
				{
					// ��� Ư�� ������Ʈ
					a_star_map[node_idx.x][node_idx.y] = tmp_pose;

					// bfs ��� ������ ���� �ĺ� ��� ����
					extract_min_heap.push(tmp_pose);

					// ���� �Ÿ� ����. BFS�� ������ �ִ� �� ������, �� �˾ƺ��߰ڴ�.
					prev_walked[node_idx.x][node_idx.y] = POINT{ cur_node.pose_.x, cur_node.pose_.y };
				}

			}

		}
	}
	// <<

	// �� Ʈ��ŷ
	POINT bt_idx = cur_node.pose_;
	while (!(bt_idx.x == prev_walked[bt_idx.x][bt_idx.y].x && bt_idx.y == prev_walked[bt_idx.x][bt_idx.y].y))
	{
		path_.emplace_back(bt_idx);
		bt_idx = prev_walked[bt_idx.x][bt_idx.y];
	}
	// ù ���� ����
	path_.emplace_back(bt_idx);

	// ������
	std::reverse(path_.begin(), path_.end());

	return a_star_map;
}

template <typename T>
vector<POINT> AStarAlgorithm<T>::GetPath() const
{
	return path_;
}

template <typename T>
int AStarAlgorithm<T>::GetHeuristicsDistance(const POINT& p1, const POINT& p2)
{
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;
	int bigger = dx > dy ? dx : dy;
	int smaller = dx > dy ? dy : dx;

	return  smaller * diag_w_ + straight_w_ * (bigger - smaller);
}

