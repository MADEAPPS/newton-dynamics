/////////////////////////////////////////////////////////////////////////////
// Name:        dMeshNGD.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
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

#ifndef _D_UNDO_REDO_SAVE_SELECTED_MESH_H_
#define _D_UNDO_REDO_SAVE_SELECTED_MESH_H_


class InfoMeshPair
{
	public:
	InfoMeshPair (dMeshNodeInfo* const info)
		:m_meshInfo (info)
		,m_newtonMesh (NewtonMeshCreateFromMesh(info->GetMesh())) 
	{
		m_meshInfo->AddRef();
	}

	InfoMeshPair (const InfoMeshPair& copy)
		:m_meshInfo (copy.m_meshInfo)
		,m_newtonMesh(NewtonMeshCreateFromMesh(copy.m_newtonMesh))
	{
		m_meshInfo->AddRef();
	}

	~InfoMeshPair ()
	{
		NewtonMeshDestroy (m_newtonMesh);
		m_meshInfo->Release();
	}

	dMeshNodeInfo* m_meshInfo;
	NewtonMesh* m_newtonMesh;
};

class dUndoRedoSaveSelectedMesh: public dUndoRedo, dList<InfoMeshPair>
{
	public:
	dUndoRedoSaveSelectedMesh (dPluginInterface* const interface);
	~dUndoRedoSaveSelectedMesh();

	using dUndoRedo::operator new;
	using dUndoRedo::operator delete;

	protected:
	virtual void RestoreState(dUndodeRedoMode mode);
	virtual dUndoRedo* CreateRedoState() const;

	dPluginInterface* m_interface;
};


#endif