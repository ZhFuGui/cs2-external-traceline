#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

BOOL WINAPI DllMain(HINSTANCE, DWORD, LPVOID) {
    // No I/O, allocation, locking or worker creation under the loader lock.
    return TRUE;
}
