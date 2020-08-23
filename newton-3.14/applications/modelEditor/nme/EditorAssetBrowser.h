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

#ifndef _EDITOR_ASSET_BROWSER_H_
#define _EDITOR_ASSET_BROWSER_H_

class NewtonModelEditor;

class EditorAssetBrowser: public FXComboBox
{
	public:
	class AssetList: public dTree<void*, dPluginScene*>
	{
		public:
		AssetList()
		{
		}
	};


	EditorAssetBrowser();
	EditorAssetBrowser (FXComposite* const parent, NewtonModelEditor* const mainFrame);
	virtual ~EditorAssetBrowser(void);
	
	void Clear();
	dPluginInterface::dAssetList::dListNode* GetCurrentAssetPluginNode () const;
	void AddAsset (dPluginInterface::dAssetList::dListNode* const assetPluginNode);
	void AddAssetAndPopulate (dPluginInterface::dAssetList::dListNode* const assetPluginNode);
	
	private:
	void InitExploreExpandedState(AssetList::dTreeNode* const assetNode, dScene::dTreeNode* const parentNode);

	NewtonModelEditor* m_mainFrame;
	AssetList m_assetList;
};


#endif