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

#include "ndSandboxStdafx.h"
#include "ndDemoEntityManager.h"
#include "ndDebugDisplayRenderPass.h"

ndDebugDisplayRenderPass::ndDebugDisplayRenderPass(ndDemoEntityManager* const owner)
	:ndRenderPass(*owner->GetRenderer())
	,m_owner(owner)
{
	m_active = false;
}

ndDebugDisplayRenderPass::~ndDebugDisplayRenderPass()
{
}

void ndDebugDisplayRenderPass::SetDisplayMode(ndInt32 mode)
{
	if (!mode)
	{
		m_active = false;
		return;
	}

	m_active = true;
}

void ndDebugDisplayRenderPass::RenderScene(ndFloat32)
{

}
