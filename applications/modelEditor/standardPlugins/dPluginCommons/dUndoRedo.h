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

#ifndef _UNDO_REDO_H_
#define _UNDO_REDO_H_

#include "dPluginUtils.h"

class dUndoRedo: public dPluginAlloc 
{
	public:
	enum dUndodeRedoMode
	{
		m_undo,
		m_redo,
	};
	DPLUGIN_API dUndoRedo();
	virtual DPLUGIN_API ~dUndoRedo();

	virtual dUndoRedo* CreateRedoState() const = 0;
	virtual void RestoreState(dUndodeRedoMode mode) = 0;
};

class dUndoRedoManager: public dPluginAlloc 
{
	public:
	class dStack: public dList<dUndoRedo*>
	{
		public:
		DPLUGIN_API dStack ();
		DPLUGIN_API ~dStack ();

		DPLUGIN_API void Clear();
		DPLUGIN_API void Push(dUndoRedo* const state);
		DPLUGIN_API dUndoRedo* Pop();
	};

	DPLUGIN_API dUndoRedoManager();
	DPLUGIN_API ~dUndoRedoManager();
	
	DPLUGIN_API void Clear();
	DPLUGIN_API void Push(dUndoRedo* const state);

	DPLUGIN_API void Undo();
	DPLUGIN_API void Redo();

	private:
	dStack m_undodStack;
	dStack m_redodStack;
};

#endif