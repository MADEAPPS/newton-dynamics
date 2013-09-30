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

#ifndef _D_UNDO_CURRENT_SCENE_H_
#define _D_UNDO_CURRENT_SCENE_H_

#include "dPluginStdafx.h"
#include "dUndoRedo.h"
#include "dPluginInterface.h"

class dPluginMesh;
class dPluginScene;


class dUndoCurrentScene: public dUndoRedo
{
	public:
/*
	class dSelectionTree: public dHierarchy<dSelectionTree>
	{
		public:
		dSelectionTree (dNodeInfo* const info)
			:dHierarchy<dSelectionTree>(info->GetName())
			,m_info(info)
		{
			m_info->AddRef();
		}

		~dSelectionTree ()
		{
			m_info->Release();
		}

		dNodeInfo* m_info;
	};
*/

	dUndoCurrentScene(dPluginInterface* const interface, dPluginScene* const deltaScene);
	virtual ~dUndoCurrentScene();

	protected:
	virtual void RestoreState(dUndodeRedoMode mode);
	virtual dUndoRedo* CreateRedoState() const;
	
//	int nodeIndex;
//	dTree<int, void*> m_exploreStatus;
//	dSelectionTree* m_selection;
	dPluginScene* m_deltaScene;
	dPluginInterface* m_interface;
};

#endif