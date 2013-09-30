/////////////////////////////////////////////////////////////////////////////
// Name:        dPluginScene.cpp
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


#include "dPluginStdafx.h"
#include "dPluginScene.h"
#include "dPluginInterface.h"


dPluginScene::dPluginScene(NewtonWorld* const newton)
	:dScene (newton)
	,m_lru (100)
{
	dScene::dTreeNode* const rootNode = GetRootNode();
	dNodeInfo* const rootInfo = GetInfoFromNode(rootNode);
	rootInfo->SetEditorFlags (dPluginInterface::m_expanded);

}

dPluginScene::dPluginScene(const dPluginScene& scene)
	:dScene (scene)
	,m_lru (scene.m_lru)
{
}


dPluginScene::~dPluginScene(void)
{
}


int dPluginScene::IncLRU()
{
	m_lru ++;
	return m_lru;
}



void dPluginScene::RenderFlatShaded (dSceneRender* const render)
{

//	virtual void EnableZbuffer();
//	virtual void DisableZbuffer();
//	virtual void EnableBackFace();
//	virtual void DisableBackFace();
//	virtual void EnableBlend();
//	virtual void DisableBlend();
//	virtual void EnableLighting();
//	virtual void DisableLighting();
//	virtual void EnableTexture();
//	virtual void DisableTexture();
//	virtual void SetColor(const dVector& color);

//	glEnable (GL_DEPTH_TEST);
	render->EnableZbuffer();

//	glDepthFunc(GL_LEQUAL);

//	glDisable(GL_CULL_FACE);
	render->EnableBackFace();

//	glDisable (GL_LIGHTING);
	//render->EnableLighting();
	render->DisableLighting();

//	glDisable(GL_TEXTURE_2D);
	render->DisableTexture();

//	glDisable(GL_BLEND);
	render->DisableBlend();

	dScene::dTreeNode* const rootNode = GetRootNode();
	for (void* link = GetFirstChildLink(rootNode); link; link = GetNextChildLink(rootNode, link)) {
		dScene::dTreeNode* const node = GetNodeFromLink (link);
		dNodeInfo* const info = GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			//info->DrawFlatShaded(render, this, node);
			RenderFlatShadedSceneNode (render, node);
		}
	}
}

void dPluginScene::RenderWireframeSelection (dSceneRender* const render, dPluginInterface* const interface)
{
	render->EnableZbuffer();
	render->EnableBackFace();
	render->DisableLighting();
	render->DisableTexture();
	render->DisableBlend();

	render->SetColor(dVector (1.0f, 1.0f, 1.0f));

	dScene::dTreeNode* const rootNode = GetRootNode();
	for (void* link = GetFirstChildLink(rootNode); link; link = GetNextChildLink(rootNode, link)) {
		dScene::dTreeNode* const sceneNode = GetNodeFromLink (link);
		dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*)GetInfoFromNode(sceneNode);
		if (sceneInfo->IsType(dSceneNodeInfo::GetRttiType())) {
			//RenderSelectedSceneNodes (render, sceneNode, interface);
			RenderSelectedSceneNodes (render, link, interface);
		}
	}
}


void dPluginScene::RenderWireframe (dSceneRender* const render)
{
	render->EnableZbuffer();
	render->EnableBackFace();
	render->DisableLighting();
	render->DisableTexture();
	render->DisableBlend();

	dScene::dTreeNode* const rootNode = GetRootNode();
	for (void* link = GetFirstChildLink(rootNode); link; link = GetNextChildLink(rootNode, link)) {
		dScene::dTreeNode* const sceneNode = GetNodeFromLink (link);
		dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*)GetInfoFromNode(sceneNode);
		if (sceneInfo->IsType(dSceneNodeInfo::GetRttiType())) {
			RenderWireframeSceneNode (render, sceneNode);
		}
	}
}


void dPluginScene::RenderSolidWireframe (dSceneRender* const render)
{
	render->SetColor(dVector (0.5f, 0.5f, 0.5f));
	render->EnableZBias(1.0f);
	RenderFlatShaded (render);
	render->DisableZBias();

	render->SetColor(dVector (0.0f, 0.0f, 0.0f));
	RenderWireframe (render);
}


void dPluginScene::RenderFlatShadedSceneNode (dSceneRender* const render, dScene::dTreeNode* const sceneNode)
{
	dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*)GetInfoFromNode(sceneNode);	

	render->PushMatrix (sceneInfo->GetTransform());

	for (void* link = GetFirstChildLink(sceneNode); link; link = GetNextChildLink(sceneNode, link)) {
		dScene::dTreeNode* const node = GetNodeFromLink (link);
		dGeometryNodeInfo* const geometryInfo = (dGeometryNodeInfo*)GetInfoFromNode(node);
		if (geometryInfo->IsType(dGeometryNodeInfo::GetRttiType())) {
			geometryInfo->DrawFlatShaded(render, this, node);
		}
	}

	for (void* link = GetFirstChildLink(sceneNode); link; link = GetNextChildLink(sceneNode, link)) {
		dScene::dTreeNode* const node = GetNodeFromLink (link);
		dSceneNodeInfo* const info = (dSceneNodeInfo*)GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			RenderFlatShadedSceneNode (render, node);
		}
	}

	render->PopMatrix ();
}


void dPluginScene::RenderWireframeSceneNode (dSceneRender* const render, dScene::dTreeNode* const sceneNode)
{
	dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*)GetInfoFromNode(sceneNode);	

	render->PushMatrix (sceneInfo->GetTransform());

	for (void* link = GetFirstChildLink(sceneNode); link; link = GetNextChildLink(sceneNode, link)) {
		dScene::dTreeNode* const node = GetNodeFromLink (link);
		dGeometryNodeInfo* const geometryInfo = (dGeometryNodeInfo*)GetInfoFromNode(node);
		if (geometryInfo->IsType(dGeometryNodeInfo::GetRttiType())) {
			geometryInfo->DrawWireFrame(render, this, node);
		}
	}

	for (void* link = GetFirstChildLink(sceneNode); link; link = GetNextChildLink(sceneNode, link)) {
		dScene::dTreeNode* const node = GetNodeFromLink (link);
		dSceneNodeInfo* const info = (dSceneNodeInfo*)GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			RenderWireframeSceneNode (render, node);
		}
	}

	render->PopMatrix ();
}


//void dPluginScene::RenderSelectedSceneNodes (dSceneRender* const render, dScene::dTreeNode* const sceneNode, dPluginInterface* const interface)
void dPluginScene::RenderSelectedSceneNodes (dSceneRender* const render, void* const incidentSceneNodeLink, dPluginInterface* const interface)
{
	dScene::dTreeNode* const sceneNode = GetNodeFromLink (incidentSceneNodeLink);
	dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*)GetInfoFromNode(sceneNode);	

	render->PushMatrix (sceneInfo->GetTransform());
	if (interface->IsNodeSelected (incidentSceneNodeLink)) {
		for (void* link = GetFirstChildLink(sceneNode); link; link = GetNextChildLink(sceneNode, link)) {
			dScene::dTreeNode* const node = GetNodeFromLink (link);
			dGeometryNodeInfo* const geometryInfo = (dGeometryNodeInfo*)GetInfoFromNode(node);
			if (geometryInfo->IsType(dGeometryNodeInfo::GetRttiType())) {
				geometryInfo->DrawWireFrame(render, this, node);
			}
		}
	}

	for (void* link = GetFirstChildLink(sceneNode); link; link = GetNextChildLink(sceneNode, link)) {
		dScene::dTreeNode* const node = GetNodeFromLink (link);
		dSceneNodeInfo* const info = (dSceneNodeInfo*)GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			//RenderSelectedSceneNodes (render, node, interface);
			RenderSelectedSceneNodes (render, link, interface);
		}
	}

	render->PopMatrix ();
}


void dPluginScene::UpdateAllOOBB ()
{
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const sceneNode = iter.GetNode();
		dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*) GetInfoFromNode(sceneNode);
		if (sceneInfo->IsType(dSceneNodeInfo::GetRttiType())){
			sceneInfo->UpdateOOBB (this, sceneNode);
		}
	}
}


void dPluginScene::Serialize (const char* const fileName)
{
	dAssert (0);
	dScene::Serialize (fileName);
}

bool dPluginScene::Deserialize (const char* const fileName)
{
	bool state = dScene::Deserialize (fileName);
	return state;
}