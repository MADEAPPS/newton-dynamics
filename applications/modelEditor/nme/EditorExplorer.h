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

class EditorExplorer: public wxTreeCtrl
{
	public:
	class TraverseExplorer;
	class ExplorerData;
	class ChangeNames;
	class UndoRedoChangeName;

	EditorExplorer (NewtonModelEditor* const mainFrame);
	~EditorExplorer(void);

	void Clear();
	void ReconstructScene(const dPluginScene* const scene);

	private:
	DECLARE_EVENT_TABLE()

	void OnKeyboardItem (wxKeyEvent& event);
	void OnExpandItem (wxTreeEvent& event);
	void OnCollapseItem (wxTreeEvent& event);
	void OnDeleteItem (wxTreeEvent& event);
	void OnSelectItem (wxTreeEvent& event);
	void OnEndEditItemName (wxTreeEvent& event);
	void OnBeginEditItemName (wxTreeEvent& event);

	NewtonModelEditor* m_mainFrame;

};


#endif