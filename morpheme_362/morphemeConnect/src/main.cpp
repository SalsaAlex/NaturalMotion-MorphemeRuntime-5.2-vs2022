#include <windows.h>
#include <stdlib>

#define WX_HEADER ""

#include WX_HEADER

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	return wxEntry(hInstance, hPrevInstance, lpCmdLine, nShowCmd);
}