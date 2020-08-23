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



// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the ValveMapLoader_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// NewtonPlugin_API functions as being imported from a DLL, whereas this DLL sees symbols

// defined with this macro as being exported.
#ifndef _D_FBX_EXPORT_IMPORT_GLUE_H_
#define _D_FBX_EXPORT_IMPORT_GLUE_H_

#ifdef _FBX_PLUGIN_EXPORTS
#define fbxPlugin_API __declspec(dllexport)
#else
#define fbxPlugin_API __declspec(dllimport)
#endif



class dPluginRecord;
extern "C" 	fbxPlugin_API dPluginRecord** GetPluginArray();


#endif