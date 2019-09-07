/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// RenderPrimitive.h: interface for the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __SKY_BOX_H_
#define __SKY_BOX_H_

#include "toolbox_stdafx.h"
#include "../DemoEntity.h"

class DemoEntityManager;

class SkyBox: public DemoEntity
{
	public:
	SkyBox(GLuint shader);
	~SkyBox();

	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const;

	private:
	void DrawMesh () const;

	GLuint m_displayList;
	GLuint m_shader;
	GLuint m_textures[6];
};

#endif 

