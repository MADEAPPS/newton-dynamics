// stdafx.cpp : source file that includes just the standard includes
// dPluginCommons.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information

#include "dPluginStdafx.h"

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file

#ifdef _MSC_VER
BOOL APIENTRY DllMain( HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
{
	switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
			#ifdef _DEBUG
				_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
			#endif
			break;

		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
	}


	return TRUE;
}
#endif


void* operator new (size_t size) 
{ 
	return dContainersAlloc::Alloc (size);
}

void operator delete (void* ptr) 
{ 
	dContainersAlloc::Free (ptr);
}


void* dPluginAlloc::operator new (size_t size)
{
	return ::new char[size];
}

void dPluginAlloc::operator delete (void* ptr)
{
	::delete[] (char*) ptr;
}

