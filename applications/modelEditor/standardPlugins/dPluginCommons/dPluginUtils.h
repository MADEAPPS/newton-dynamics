/////////////////////////////////////////////////////////////////////////////
// Name:        dPluginStdafx.h
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



#ifndef __D_PLUGIN_UTILS_H__
#define __D_PLUGIN_UTILS_H__


// for some reason specifying a relative does not seem to work in Linus
// and i have to specify a absolute path
// #define ASSETS_PATH "."
void GetMediaDirectory (char* const mediaDirOut);
void GetAplicationDirectory (char* const aplicationDirOut);
void GetWorkingFileName (const char* const name, char* const outPathNameOut);


// TODO: reference additional headers your program requires here

#endif