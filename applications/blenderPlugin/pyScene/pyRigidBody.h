/////////////////////////////////////////////////////////////////////////////
// Name:        pyRigidBody.h
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

#pragma once
#include "pyTypes.h" 
#include "pyBaseNodeInfo.h"


class pyScene;

class pyRigidBody: public pyBaseNodeInfo<dRigidbodyNodeInfo>
{
	enum ShapesType
	{	
		m_box = 0,
		m_sphere, 
		m_cylinder, 
		m_cone, 
		m_collisionTree, 
		m_convexHull, 
	};
public:
	pyRigidBody(pyScene* scene, void* node);
	~pyRigidBody(void);

	void SetName (const char* name); 
	void SetShape (int type);
	void SetMass (double mass);
};
