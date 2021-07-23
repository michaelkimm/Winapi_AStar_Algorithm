// AstarDesktopAP.cpp : 애플리케이션에 대한 진입점을 정의합니다.
//

#include "framework.h"
#include "AstarDesktopAP.h"

#define MAX_LOADSTRING 100

// : >> 디버깅용 콘솔창
#include <stdio.h>
#include <iostream>

#ifdef UNICODE
#pragma comment(linker, "/entry:wWinMainCRTStartup /subsystem:console")
#else
#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")
#endif

using namespace std;

// <<


#include "AStarNode.h"

#include "MyGdiplus.h"
#include "MyFigureDraw.h"

#ifndef MAP_SIZE
#define MAP_SIZE 30
#endif

#ifndef INF_INT_
#define INF_INT_ 2147480000
#endif



static RECT rect_view;
static POINT mouse_pt;
static int view_width;
static int view_height;
static int dx;
static int dy;

// : >> A*

// A* 알고리즘 객체
AStarAlgorithm<AStarNode> a_star;

// A* 계산한 맵
vector<vector<AStarNode>> AStarMap;

// 경로 저장 공간
vector<POINT> path;

int map[MAP_SIZE][MAP_SIZE] = { 0, };
vector<POINT> map_box;

POINT start_node{ 0, 0 };
POINT end_node{ 0, 0 };

void DrawTile(HDC _hdc, int _dx, int _dy);
// <<

// : >> 더블 버퍼링
HBITMAP hDoubleBufferImage = 0;
// <<

// 전역 변수:
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.

// 이 코드 모듈에 포함된 함수의 선언을 전달합니다:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // TODO: 여기에 코드를 입력합니다.

    // 전역 문자열을 초기화합니다.
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_ASTARDESKTOPAP, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // 애플리케이션 초기화를 수행합니다:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_ASTARDESKTOPAP));

    MSG msg;

    // 기본 메시지 루프입니다:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int) msg.wParam;
}



//
//  함수: MyRegisterClass()
//
//  용도: 창 클래스를 등록합니다.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_ASTARDESKTOPAP));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_ASTARDESKTOPAP);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   함수: InitInstance(HINSTANCE, int)
//
//   용도: 인스턴스 핸들을 저장하고 주 창을 만듭니다.
//
//   주석:
//
//        이 함수를 통해 인스턴스 핸들을 전역 변수에 저장하고
//        주 프로그램 창을 만든 다음 표시합니다.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장합니다.

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  함수: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  용도: 주 창의 메시지를 처리합니다.
//
//  WM_COMMAND  - 애플리케이션 메뉴를 처리합니다.
//  WM_PAINT    - 주 창을 그립니다.
//  WM_DESTROY  - 종료 메시지를 게시하고 반환합니다.
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

    switch (message)
    {
	case WM_CREATE:
		{
			GetClientRect(hWnd, &rect_view);
			// : >> 환경 변수 초기화

			// 맵 & h값 초기화
			AStarMap = vector<vector<AStarNode>>{ MAP_SIZE, vector<AStarNode>{ MAP_SIZE, {-1, -1, -1, -1 }} };
			for (int i = 0; i < MAP_SIZE; i++)
			{
				for (int j = 0; j < MAP_SIZE; j++)
				{
					// 모든 맵 초기화;
					AStarMap[i][j].pose_ = POINT{ j, i };

					AStarMap[i][j].g_ = INF_INT_;
					AStarMap[i][j].f_ = INF_INT_;
				}
			}
			// <<
		}
		break;
	case WM_SIZE:
		{
			GetClientRect(hWnd, &rect_view);
			// DrawGrid에 맞는 맵을 알아야한다.
			view_width = rect_view.right;		// 화면 가로 사이즈
			view_height = rect_view.bottom;		// 화면 세로 사이즈
			dx = view_width / MAP_SIZE;			// 가로 칸 사이즈
			dy = view_height / MAP_SIZE;		// 세로 칸 사이즈

			// 클릭한 곳이 어떤 타일 안에 있는지 알아야함.
		}
		break;
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // 메뉴 선택을 구문 분석합니다:
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
	case WM_LBUTTONDOWN:
		{
			mouse_pt = POINT{ GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };

			// w(wall)을 누르면 장애물 생성
			if (GetKeyState(0x57) & 0x8000)
			{
				cout << "w키 + 마우스 왼쪽 down!\n";
				map[mouse_pt.y / dy][mouse_pt.x / dx] = 1;
			}

			// d(delete)을 누르면 장애물 제거
			else if (GetKeyState(0x44) & 0x8000)
			{
				cout << "d키 + 마우스 왼쪽 down!\n";
				map[mouse_pt.y / dy][mouse_pt.x / dx] = 0;
			}

			InvalidateRect(hWnd, NULL, false);
		}
		break;
	case WM_LBUTTONUP:
		{
			mouse_pt = POINT{ GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam) };

			// s(start)을 누르면 출발점 설정
			if (GetKeyState(0x53) & 0x8000)
			{
				cout << "s키 + 마우스 왼쪽 up : (" << mouse_pt.x << ", " << mouse_pt.y << ")\n";
				start_node = POINT{ mouse_pt.x / dx, mouse_pt.y / dy };
			}

			// e(end)을 누르면 도착지 설정
			else if (GetKeyState(0x45) & 0x8000)
			{
				cout << "e키 + 마우스 왼쪽 up!\n";

				end_node = POINT{ mouse_pt.x / dx, mouse_pt.y / dy };

				// 맵 설정, A* 계산한 맵, A* 경로 저장
				a_star.FindPath(AStarMap, path, start_node, end_node, (int**)map, MAP_SIZE, MAP_SIZE);
				cout << "A* 계산한 맵 & 경로 저장 완료!\n";

				// 길 프린트
				cout << "path: ";
				for (vector<POINT>::iterator i = path.begin(); i != path.end(); i++)
				{
					cout << "(" << (*i).x << ", " << (*i).y << ") -> ";
				}
				cout << endl;
			}

			InvalidateRect(hWnd, NULL, false);
		}
		break;
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
			
			// 타일 그리기
			DrawTile(hdc, dx, dy);

            EndPaint(hWnd, &ps);
        }
        break;
    case WM_DESTROY:
		DeleteObject(hDoubleBufferImage);
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// 정보 대화 상자의 메시지 처리기입니다.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}



void DrawTile(HDC _hdc, int _dx, int _dy)
{
	HBRUSH hBrush;
	HBRUSH hOldBrush;
	COLORREF color;

	HDC hMemDC = CreateCompatibleDC(_hdc);
	HBITMAP hOldBitmap;

	if (hDoubleBufferImage == 0)
		hDoubleBufferImage = CreateCompatibleBitmap(_hdc, rect_view.right, rect_view.bottom);
	hOldBitmap = (HBITMAP)SelectObject(hMemDC, hDoubleBufferImage);


	{
		for (int i = 0; i < MAP_SIZE; i++)
		{
			for (int j = 0; j < MAP_SIZE; j++)
			{
				AStarMap[i][j].width_ = _dx;
				AStarMap[i][j].height_ = _dy;

				// 타일이 <장애물 or 출발점 or 도착점> 경우, 페인트
				if (map[i][j] == 1)
				{
					color = RGB(116, 116, 116);	// 회색
				}
				else if (start_node.y == i && start_node.x == j)
				{
					color = RGB(72, 156, 255);	// 연한 파랑색
					cout << "start: <<" << j << ", " << i << ">>\n";
				}
				else if (end_node.y == i && end_node.x == j)
				{
					color = RGB(241, 95, 95);	// 연한 빨간색
				}
				else if (AStarMap[i][j].is_path_ == true)
				{
					color = RGB(183, 240, 177);	// 연한 초록색
				}
				else
				{
					color = RGB(255, 255, 255);	// 하얀색
				}
				hBrush = CreateSolidBrush(color);
				hOldBrush = (HBRUSH)SelectObject(hMemDC, hBrush);

				AStarMap[i][j].Draw(hMemDC);

				SelectObject(hMemDC, hOldBrush);
				DeleteObject(hBrush);
			}
		}
	}

	

	BitBlt(_hdc, 0, 0, rect_view.right, rect_view.bottom, hMemDC, 0, 0, SRCCOPY);
	SelectObject(hMemDC, hOldBitmap);
	DeleteDC(hMemDC);
}