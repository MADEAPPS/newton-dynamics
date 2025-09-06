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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderPassGui.h"

ndRenderPassGui::ndRenderPassGui(ndRender* const owner)
	:ndRenderPass(owner)
{
}

ndRenderPassGui::~ndRenderPassGui()
{
}

void ndRenderPassGui::StateBegin()
{
	ndRenderContext* const context = *m_owner->m_context;
	context->SetGuiRenderStates();
}

void ndRenderPassGui::StateEnd()
{
	ndRenderContext* const context = *m_owner->m_context;
	context->EndGuiRenderStates();
}
