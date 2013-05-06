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

#ifndef _EDITOR_ASSET_EXPLORER_H_
#define _EDITOR_ASSET_EXPLORER_H_

class NewtonModelEditor;

class EditorAssetExplorer: public FXTreeList 
{
	class ChangeInfoName;

	public:
	EditorAssetExplorer();
	EditorAssetExplorer(FXComposite* const parent, NewtonModelEditor* const mainFrame);
	~EditorAssetExplorer();

	void create();

	void Populate (dPluginScene* const asset);
	long onMouseBottonEvent(FXObject* sender, FXSelector id, void* eventPtr);
	

	void HandleSelectionEvent (const dList<dScene::dTreeNode*>& traceToRoot);

	private:
	void ChangeItemName (FXTreeItem* const item);
	void HandleExpandEvent (FXTreeItem* const item);
	void HandleSelectionEvent (FXTreeItem* const item);
	void ExpandItem (dPluginScene* const scene, FXTreeItem* const item);
	void PopulateTextureChache(dPluginScene* const scene, FXTreeItem* const rootItem);
	void PopulateMaterialChache(dPluginScene* const scene, FXTreeItem* const rootItem);
	void PopulateGeometryChache(dPluginScene* const scene, FXTreeItem* const rootItem);
	void PopulateSceneNode(dPluginScene* const scene, FXTreeItem* const modelItem);
	
	FXTreeItem* m_rootItem;
	NewtonModelEditor* m_mainFrame;
	

	FXIcon* m_rootIcon;
	FXIcon* m_sceneIcon;
	FXIcon* m_meshIcon;
	FXIcon* m_textureIcon;
	FXIcon* m_materialIcon;
	FXIcon* m_geometryIcon;
	FXIcon* m_textureCacheIcon;
	FXIcon* m_materialCacheIcon;
	FXIcon* m_geometryCacheIcon;

	FXDECLARE(EditorAssetExplorer)
};


#endif