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


#include "stdafx.h"

#include "dMeshNGD.h"
#include "dRigidBody.h"
#include "dBoxCollision.h"
#include "dStandardModel.h"
#include "dSphereCollision.h"
#include "dMeshBoxPrimitive.h"
#include "dStandardModelsDllGlue.h"

dPluginRecord** GetPluginArray()
{
	static dPluginRecord* array[] = 
	{
		// do not register this plug in since it is a native file format, but leave it for reference to write other loader type 
		//dMeshNGD::GetPlugin(),
		dRigidBody::GetPlugin(),
		dBoxCollision::GetPlugin(),
		dStandardModel::GetPlugin(),
		dSphereCollision::GetPlugin(),
		dMeshBoxPrimitive::GetPlugin(),
		NULL
	};

	return array;
}


