/////////////////////////////////////////////////////////////////////////////
// Name:        dll glue .h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#include "dfbxStdafx.h"
#include "dfbxGlue.h"
#include "dfbxImport.h"
#include "dfbxExport.h"

dPluginRecord** GetPluginArray()
{
	static dPluginRecord* array[] = 
	{
		dfbxImport::GetPluginFBX(),
		dfbxExport::GetPluginFBX(),

		dfbxImport::GetPluginOBJ(),
		dfbxExport::GetPluginOBJ(),

		dfbxImport::GetPluginDAE(),
		dfbxExport::GetPluginDAE(),

		NULL
	};

	return array;
}


