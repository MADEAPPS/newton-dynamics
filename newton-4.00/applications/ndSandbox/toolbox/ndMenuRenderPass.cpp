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
#include "ndMenuRenderPass.h"
#include "ndDemoEntityManager.h"

ndMenuRenderPass::ndMenuRenderPass(ndDemoEntityManager* const owner)
	:ndGuiRenderPass(*owner->GetRenderer())
	,m_owner(owner)
{
}

ndMenuRenderPass::~ndMenuRenderPass()
{
}

void ndMenuRenderPass::RenderScene(ndFloat32)
{
	StateBegin();
	m_owner->RenderStats();
	StateEnd();
}
