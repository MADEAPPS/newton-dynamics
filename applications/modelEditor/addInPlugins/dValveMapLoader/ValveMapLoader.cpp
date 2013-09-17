// ValveMapLoader.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "ValveMapLoader.h"
#include "ValveMapImport.h"


dPluginRecord** GetPluginArray()
{
	static dPluginRecord* array[] = 
	{
		ValveMapImport::GetPlugin(),
		NULL
	};

	return array;
}


