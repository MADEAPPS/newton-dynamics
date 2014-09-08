/////////////////////////////////////////////////////////////////////////////
// Name:        dMeshNGD.h
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

#ifndef _D_FREEZE_SCALE_H_
#define _D_FREEZE_SCALE_H_


class dFreezeSceneScale: public dPluginTool
{
	public:
	dFreezeSceneScale();
	~dFreezeSceneScale();
	static dFreezeSceneScale* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetDescription () {return "Freeze Scene Scale";}
	virtual const char* GetSignature () {return "Freeze Scene Scale";}
	virtual bool Execute (dPluginInterface* const interface);
};


class dFreezeGeometryScale: public dPluginTool
{
	public:
	dFreezeGeometryScale();
	~dFreezeGeometryScale();
	static dFreezeGeometryScale* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetDescription () {return "Freeze Geometry Scale";}
	virtual const char* GetSignature () {return "Freeze Geometry Scale";}
	virtual bool Execute (dPluginInterface* const interface);
};


class dFreezeRootRotation: public dPluginTool
{
	public:
	dFreezeRootRotation();
	~dFreezeRootRotation();
	static dFreezeRootRotation* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetDescription () {return "Freeze Root Rotation";}
	virtual const char* GetSignature () {return "Freeze Root Rotation";}
	virtual bool Execute (dPluginInterface* const interface);
};



#endif