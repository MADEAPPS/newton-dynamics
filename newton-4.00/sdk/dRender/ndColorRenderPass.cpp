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
#include "ndColorRenderPass.h"
#include "ndRenderSceneNode.h"

ndColorRenderPass::ndColorRenderPass(ndRender* const owner)
	:ndRenderPass(owner)
{
}

ndColorRenderPass::~ndColorRenderPass()
{
}

void ndColorRenderPass::RenderScene(ndFloat32 timestep)
{
#if 0
	ImGuiIO& io = ImGui::GetIO();
	ndInt32 display_w = (ndInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	ndInt32 display_h = (ndInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);

	//glViewport(0, 0, (GLint)width, (GLint)height);
	//glGetIntegerv(GL_VIEWPORT, m_viewport);

	//glBindFramebufferEXT(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, display_w, display_h);

	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
	//glClear(GL_COLOR_BUFFER_BIT);

	//ImVec4 clearColor = ImColor(114, 144, 154);
	//glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glScissor(0, 0, display_w, display_h);
	//glEnable(GL_SCISSOR_TEST);	
	glDisable(GL_SCISSOR_TEST);

	// Culling. 
	glCullFace (GL_BACK);
	glFrontFace (GL_CCW);
	glEnable (GL_CULL_FACE);
	
	//	glEnable(GL_DITHER);
	// z buffer test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);
	
	// render all entities
	const ndMatrix globalMatrix (ndGetIdentityMatrix());

	//if (!m_manager->m_hideVisualMeshes) 
	//{
	//	for (ndDemoEntityManager::ndNode* node = m_manager->GetFirst(); node; node = node->GetNext())
	//	{
	//		ndDemoEntity* const entity = *node->GetInfo();
	//		entity->Render(timestep, m_manager, globalMatrix);
	//	}
	//
	//	while (m_manager->m_transparentHeap.GetCount())
	//	{
	//		const ndDemoEntityManager::TransparentMesh& transparentMesh = m_manager->m_transparentHeap[0];
	//		transparentMesh.m_mesh->RenderTransparency(m_manager, transparentMesh.m_matrix);
	//		m_manager->m_transparentHeap.Pop();
	//	}
	//}
	//
	//// render sky lost reduce overdraw and render state change
	//if (m_manager->m_sky)
	//{
	//	m_manager->m_sky->Render(timestep, m_manager, globalMatrix);
	//}

	//if (m_manager->m_renderDemoGUI)
	//{
	//	m_manager->m_renderDemoGUI->RenderUI();
	//}
	//
	////if (m_showMeshSkeleton)
	////{
	////	for (ndNode* node = ndList<ndDemoEntity*>::GetFirst(); node; node = node->GetNext())
	////	{
	////		ndDemoEntity* const entity = node->GetInfo();
	////		entity->RenderSkeleton(this, globalMatrix);
	////	}
	////}
	////
	////if (m_showAABB) 
	////{
	////	RenderBodiesAABB(this);
	////}
	////
	////if (m_showScene)
	////{
	////	RenderWorldScene(this);
	////}
	////
	//////if (m_showRaycastHit) {
	//////	RenderRayCastHit(m_world);
	//////}
	////
	////if (m_showBodyFrame)
	////{
	////	RenderBodyFrame(this);
	////}
	//
	//if (m_manager->m_collisionDisplayMode)
	//{
	//	m_debugDisplay.RenderDebugShapes(m_manager, m_manager->m_collisionDisplayMode);
	//}
	//
	//if (m_manager->m_showCenterOfMass)
	//{
	//	m_debugDisplay.RenderCenterOfMass(m_manager);
	//}
	//
	//if (m_manager->m_showJointDebugInfo)
	//{
	//	m_debugDisplay.RenderJointsDebugInfo(m_manager);
	//}
	//
	//if (m_manager->m_showModelsDebugInfo)
	//{
	//	m_debugDisplay.RenderModelsDebugInfo(m_manager);
	//}
	//
	//if (m_manager->m_showContactPoints)
	//{
	//	m_debugDisplay.RenderContactPoints(m_manager);
	//}
	//
	//if (m_manager->m_showNormalForces) 
	//{
	//	m_debugDisplay.RenderContactPoints(m_manager);
	//	m_debugDisplay.RenderNormalForces (m_manager);
	//}

	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glDisable(GL_CULL_FACE);
	//glDisable(GL_DEPTH_TEST);
	//glEnable(GL_SCISSOR_TEST);
#endif

	m_owner->m_context->SetCollorPassRenderStates();

	const ndMatrix globalMatrix(ndGetIdentityMatrix());
	ndList<ndSharedPtr<ndRenderSceneNode>>& scene = m_owner->m_scene;
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = scene.GetFirst(); node; node = node->GetNext())
	{
		ndRenderSceneNode* const sceneNode = *node->GetInfo();
		sceneNode->Render(sceneNode->m_owner, timestep, globalMatrix);
	}
}
