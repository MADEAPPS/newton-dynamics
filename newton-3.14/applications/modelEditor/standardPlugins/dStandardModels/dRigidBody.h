/////////////////////////////////////////////////////////////////////////////
// Name:        dRigidBody.h
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

#ifndef _D_RIGID_BODY_H_
#define _D_RIGID_BODY_H_


class dRigidBody: public dBodyPlugin
{
	public:
	dRigidBody();
	~dRigidBody();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetDescription () {return "create rigid body";}

	virtual const char* GetSignature () {return "rigid body";}
	virtual bool Create (dPluginInterface* const interface);

	static dRigidBody* GetPlugin();


	private:
};

#endif