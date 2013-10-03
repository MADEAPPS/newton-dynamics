/////////////////////////////////////////////////////////////////////////////
// Name:        dScene.cpp
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
#include "dPluginMesh.h"
#include "dPluginInterface.h"
#include "dUndoCurrentScene.h"

dUndoCurrentScene::dUndoCurrentScene(dPluginInterface* const interface, dPluginScene* const deltaScene) 
	:dUndoRedo()
	,m_deltaScene(deltaScene)
	,m_interface(interface)
{
	m_deltaScene->AddRef();
}

dUndoCurrentScene::~dUndoCurrentScene() 
{
	dAssert(m_deltaScene);
	m_deltaScene->Release();
}

dUndoRedo* dUndoCurrentScene::CreateRedoState() const
{
	return new dUndoCurrentScene (m_interface, m_deltaScene);
}

void dUndoCurrentScene::RestoreState(dUndodeRedoMode mode)
{
	dPluginScene* const scene = m_interface->GetScene();
	switch (mode)
	{
		case m_undo:
		{
			scene->dScene::UnmergeScene(m_deltaScene);
			m_interface->RefreshExplorerEvent (true);
			break;
		}

		default:
		{
			scene->dScene::MergeScene(m_deltaScene);
			m_interface->RefreshExplorerEvent (false);
			break;
		}
	}
}
