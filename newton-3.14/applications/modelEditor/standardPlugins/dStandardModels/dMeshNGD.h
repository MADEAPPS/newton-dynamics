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

#ifndef _D_MESH_NGD_H_
#define _D_MESH_NGD_H_


class dMeshNGD: public dPluginMesh
{
	public:
	dMeshNGD();
	~dMeshNGD();
	static dMeshNGD* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetDescription () {return "load a NGD mesh asset";}
	virtual const char* GetSignature () {return "NGD mesh";}
	virtual dPluginScene* Create (dPluginInterface* const interface);
	virtual void Destroy (dPluginInterface* const interface, dPluginScene* const asset);
};

#endif