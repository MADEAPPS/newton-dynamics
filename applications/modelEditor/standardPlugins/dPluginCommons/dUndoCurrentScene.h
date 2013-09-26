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
	dUndoCurrentScene(dPluginInterface* const interface);
	virtual ~dUndoCurrentScene();

	protected:
	virtual void RestoreState();
	virtual dUndoRedo* CreateRedoState() const;
	

	int nodeIndex;
	dPluginInterface* m_interface;
	dTree<int, void*> m_selection;
	dTree<int, void*> m_exploreStatus;
	dPluginScene* m_backup;
};

#endif