/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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

class dPluginMesh;
class dPluginScene;
class dPluginInterface;
enum dUndodeRedoMode;

class dUndoCurrentScene: public dUndoRedo
{
	public:
	DPLUGIN_API dUndoCurrentScene(dPluginInterface* const interface, dPluginScene* const deltaScene);
	DPLUGIN_API virtual ~dUndoCurrentScene();

	protected:
	virtual void RestoreState (dUndodeRedoMode mode);
	virtual dUndoRedo* CreateRedoState() const;
	
	dPluginScene* m_deltaScene;
	dPluginInterface* m_interface;
};

#endif