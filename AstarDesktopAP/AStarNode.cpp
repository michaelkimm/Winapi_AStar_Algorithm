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
	DrawRectangle(_hdc, POINT{ pose_.x + width_ / 2, pose_.y + height_ / 2 }, width_, height_);					// 핸들, 중심점 위치, 가로, 세로
}

void AStarTile::Draw(HDC _hdc)
{
	DrawRectangle(_hdc, POINT{ pose_.x + width_ / 2, pose_.y + height_ / 2}, width_, height_);					// 핸들, 중심점 위치, 가로, 세로
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

// 생성자 & 파괴자
template <typename T>
AStarAlgorithm<T>::AStarAlgorithm(int **_map, int _map_width, int _map_height) : map_((int*)_map), map_width_(_map_width), map_height_(_map_height)
{

}

// 초기화 함수(맵을 보고 초기화)
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
	// : >> 맵 관련 함수

	// AStarNode로 구성된 2차원 벡터 맵
	vector<vector<T>> a_star_map(map_width_, vector<T>(map_height_, T{ -1, -1, -1, -1 }));

	// 방문 여부 저장하는 맵
	vector<vector<bool>> visited_map(map_width_, vector<bool>(map_height_, false));

	// 해당 정점으로 오기 전에 정점 위치 저장 맵
	vector<vector<POINT>> prev_walked(map_width_, vector<POINT>(map_height_, { -1, -1 }));

	// <<

	// : >> 환경 변수 초기화
	// 맵 & h값 초기화
	for (int i = 0; i < map_height_; i++)
	{
		for (int j = 0; j < map_width_; j++)
		{
			// 모든 맵 초기화;
			visited_map[i][j] = false;

			a_star_map[i][j].pose_ = POINT{ i, j };

			a_star_map[i][j].g_ = INF_INT_;
			a_star_map[i][j].f_ = INF_INT_;

			prev_walked[i][j].x = -1;
			prev_walked[i][j].y = -1;
		}
	}
	// <<

	// 우선 순위 큐
	priority_queue<T, vector<T>, greater<T>> extract_min_heap;


	// : >> 시작 정점 초기화
	a_star_map[_start.x][_start.y] = T{ _start.x, _start.y, 0, 0 };
	T node{ _start, 0, GetHeuristicsDistance(_start, _end) };

	// 첫 노드 삽입
	extract_min_heap.push(node);

	// 왓던 길 표시
	prev_walked[_start.x][_start.y] = node.pose_;

	// <<

	T cur_node{ 0, 0, 0, 0 };

	// : >> 목표 노드에 도착 안했으면
	while (1)
	{
		// 현재 노드 추출 / 후보 경로중 가장 f값 작은 노드
		cur_node = extract_min_heap.top();
		extract_min_heap.pop();

		// 방문 표시
		visited_map[cur_node.pose_.x][cur_node.pose_.y] = true;

		// 목적지 도달 판단
		if ((cur_node.pose_.x == _end.x && cur_node.pose_.y == _end.y))
			break;

		// : >> 현재 노드 주변 값 업데이트
		for (int i = 0; i < (pow(look_ahead_size_, 2) - 1); i++)
		{
			POINT node_idx{ cur_node.pose_.x + dir_[i][0], cur_node.pose_.y + dir_[i][1] };

			// 맵 안인가?
			if (node_idx.x < 0 || node_idx.y < 0 || node_idx.x >= (pow(look_ahead_size_, 2) - 1) || node_idx.y >= (pow(look_ahead_size_, 2) - 1))
				continue;

			// 장애물인가?
			if (map_[node_idx.x * map_height_ + node_idx.y] == 1)
			{
				a_star_map[node_idx.x][node_idx.y].obstackle_ = true;
				continue;
			}

			// 방문하지 않은 정점이면서 정점 값을 업데이트할 수 있는 경우
			if (visited_map[node_idx.x][node_idx.y] == false)
			{
				T tmp_pose{ node_idx, a_star_map[cur_node.pose_.x][cur_node.pose_.y].g_ + GetHeuristicsDistance(cur_node.pose_, node_idx),
									GetHeuristicsDistance(node_idx, _end) };

				// 후보 경로 타일로 갔을 때, 총 지나갈 길(f_)가 적은 것을 택한다.
				int haha = a_star_map[tmp_pose.pose_.x][tmp_pose.pose_.y].f_;
				if (tmp_pose.f_ < a_star_map[tmp_pose.pose_.x][tmp_pose.pose_.y].f_)
				{
					// 노드 특성 업데이트
					a_star_map[node_idx.x][node_idx.y] = tmp_pose;

					// bfs 방식 꺼내기 위해 후보 경로 저장
					extract_min_heap.push(tmp_pose);

					// 이전 거리 저장. BFS와 연관이 있는 것 같은데, 더 알아봐야겠다.
					prev_walked[node_idx.x][node_idx.y] = POINT{ cur_node.pose_.x, cur_node.pose_.y };
				}

			}

		}
	}
	// <<

	// 백 트랙킹
	POINT bt_idx = cur_node.pose_;
	while (!(bt_idx.x == prev_walked[bt_idx.x][bt_idx.y].x && bt_idx.y == prev_walked[bt_idx.x][bt_idx.y].y))
	{
		path_.emplace_back(bt_idx);
		bt_idx = prev_walked[bt_idx.x][bt_idx.y];
	}
	// 첫 점도 삽입
	path_.emplace_back(bt_idx);

	// 뒤집기
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

