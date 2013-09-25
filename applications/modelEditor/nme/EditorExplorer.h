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

#ifndef _EDITOR_EXPLORER_H_
#define _EDITOR_EXPLORER_H_


class dPluginScene;
class NewtonModelEditor;
//class EditorAssetBrowser;
//class EditorAssetExplorer;

class EditorExplorer: public wxTreeCtrl
{
	public:
	EditorExplorer (NewtonModelEditor* const mainFrame);
	~EditorExplorer(void);

//	void Populate (const dPluginScene* const scene);
//	void PopulateModel(const dPluginScene* const scene, FXTreeItem* const modelItem);


	void SetBrowserSelection ();
	void RefreshAllViewer ();
	
	void AddAsset (dPluginScene* const asset, dPluginMesh* const plugin);
	void PopulateCurrentAsset ();

	dPluginScene* GetCurrentAsset() const; 


	void HandleSelectionEvent (const dList<dScene::dTreeNode*>& traceToRoot) const;

//	FXTabBook* m_tabBook;
//	FXTreeList* m_sceneExplorer;
	NewtonModelEditor* m_mainFrame;
//	EditorAssetBrowser* m_assetBrowser;
//	EditorAssetExplorer* m_assetExplorer;
};


#endif