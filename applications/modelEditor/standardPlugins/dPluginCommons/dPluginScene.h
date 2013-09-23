/////////////////////////////////////////////////////////////////////////////
// Name:        dPluginScene.h
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

#ifndef _D_PLUGIN_SCENE_H_
#define _D_PLUGIN_SCENE_H_

#include "dPluginUtils.h"

class dPluginInterface;

class dPluginSceneRegisterClass
{
	public:
	dPluginSceneRegisterClass(const char* const className, const dNodeInfo* const singletonClass)
	{
		dNodeInfo::ReplaceSingletonClass (className, singletonClass);
	}
};

#define D_EDITOR_SCENE_REGISTER_CLASS(className) \
	D_IMPLEMENT_CLASS_NODE(className) \
	static dPluginSceneRegisterClass __##className(className::BaseClassName(), &className::GetSingleton())


class dPluginScene: public dScene, public dPluginAlloc
{
	public:
	DPLUGIN_API dPluginScene(NewtonWorld* const newton);
	virtual DPLUGIN_API ~dPluginScene(void);

	virtual DPLUGIN_API void RenderWireframe (dSceneRender* const render);
	virtual DPLUGIN_API void RenderFlatShaded (dSceneRender* const render);
	virtual DPLUGIN_API void RenderSolidWireframe (dSceneRender* const render);
	virtual DPLUGIN_API void RenderWireframeSelection (dSceneRender* const render, dPluginInterface* const interface);
	virtual DPLUGIN_API void UpdateAllOOBB ();
	virtual void DPLUGIN_API MergeScene (dPluginInterface* const interface, dPluginScene* const asset) const;

	private:
	virtual void RenderWireframeSceneNode (dSceneRender* const render, dScene::dTreeNode* const sceneNode);
	virtual void RenderFlatShadedSceneNode (dSceneRender* const render, dScene::dTreeNode* const sceneNode);
	//virtual void RenderSelectedSceneNodes (dSceneRender* const render, dScene::dTreeNode* const sceneNode, dPluginInterface* const interface);
	virtual void RenderSelectedSceneNodes (dSceneRender* const render, void* const incidentSceneNodeLink, dPluginInterface* const interface);

	int IncLRU();
	int m_lru;
};

#endif