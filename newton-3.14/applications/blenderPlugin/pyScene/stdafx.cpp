/////////////////////////////////////////////////////////////////////////////
// Name:        stdafx.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
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

// stdafx.cpp : source file that includes just the standard includes
// pyScene.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"
#include <assert.h>

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file

#ifndef _DEBUG
extern "C" {
	int _CrtDbgReportW( 
		int reportType,
		const wchar_t *filename,
		int linenumber,
		const wchar_t *moduleName,
		const wchar_t *format)
	{
		assert (0);
		return 0;
	}
}
#endif