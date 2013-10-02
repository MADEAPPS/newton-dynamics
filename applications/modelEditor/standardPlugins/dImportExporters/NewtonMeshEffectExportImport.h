/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonMeshEffectExport.h
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

#ifndef _NEWTON_MESH_EFFECT_EXPORT_IMPORT_H_
#define _NEWTON_MESH_EFFECT_EXPORT_IMPORT_H_


class NewtonMeshEffectImport: public dImportPlugin
{
	public:
	NewtonMeshEffectImport();
	~NewtonMeshEffectImport();

	static NewtonMeshEffectImport* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return "*.nme";}
	virtual const char* GetFileDescription () {return "import NewtonMeshEffect file";}

	virtual const char* GetSignature () {return "Netwon Mesh Effect import";}
	virtual bool Import (const char* const fileName, dPluginInterface* const interface);

	static void DeserializeCallback (void* const serializeHandle, void* const buffer, int size);
};



class NewtonMeshEffectExport: public dExportPlugin
{
	public:
	NewtonMeshEffectExport();
	~NewtonMeshEffectExport();

	static NewtonMeshEffectExport* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return "*.nme";}
	virtual const char* GetFileDescription () {return "save selected mesh as NewtonMeshEffect file";}

	virtual const char* GetSignature () {return "Netwon Mesh Effect Export";}
	virtual void Export (const char* const fileName, dPluginInterface* const interface);

	static void SerializeCallback (void* const serializeHandle, const void* const buffer, int size);
};

#endif