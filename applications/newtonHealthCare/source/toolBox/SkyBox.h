/* Copyright (c) <2009> <Newton Game Dynamics>
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

#ifndef __SKY_BOX__
#define __SKY_BOX__

#include <toolbox_stdafx.h>
#include "../DemoEntity.h"


class SkyBox: public DemoEntity
{
	public:
	SkyBox();
	~SkyBox();

	virtual void Render(dFloat timeStep) const;

	private:
	GLuint m_textures[6];
	dVector m_size;
};

#endif 

