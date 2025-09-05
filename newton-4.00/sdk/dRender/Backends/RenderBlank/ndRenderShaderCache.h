/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
#ifndef __RENDER_SHADER_CACHE_H__
#define __RENDER_SHADER_CACHE_H__

#include "ndRenderStdafx.h"
#include "ndRenderContext.h"

class ndRenderShaderCache
{
	public:
	ndRenderShaderCache();
	~ndRenderShaderCache();
	
	private:
	friend class ndRenderContext;
};


#endif
