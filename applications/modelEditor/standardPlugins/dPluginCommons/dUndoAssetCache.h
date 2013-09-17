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

#ifndef _D_UNDO_ASSET_CACHE_H_
#define _D_UNDO_ASSET_CACHE_H_


#include "dPluginStdafx.h"
#include "dUndoCurrentAsset.h"



class dUndoAssetCache: public dUndoCurrentAsset
{
	public:
	dUndoAssetCache (dPluginInterface* const interface);
	~dUndoAssetCache();

	void RestoreState();
	dUndoRedo* CreateRedoState() const;

//	EditorExplorer* m_explorer;
	dList<dPluginInterface::AssetPluginAssociation> m_backupAssets;
	dList<dPluginInterface::AssetPluginAssociation>::dListNode* m_currentScene;
};





#endif