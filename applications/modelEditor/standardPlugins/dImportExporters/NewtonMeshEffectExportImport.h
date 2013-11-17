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
	static dImportPlugin* GetPlugin();

	private:
	NewtonMeshEffectImport() :dImportPlugin() {}
	~NewtonMeshEffectImport(){}
	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return "*.nme";}
	virtual const char* GetFileDescription () {return "nme import";}
	virtual const char* GetSignature () {return "Import Newton Mesh Effect file";}
	virtual bool Import (const char* const fileName, dPluginInterface* const interface);
	
	static void DeserializeCallback (void* const serializeHandle, void* const buffer, int size);
};



class NewtonMeshEffectExport: public dExportPlugin
{
	public:
	static dExportPlugin* GetPlugin();

	private:
	NewtonMeshEffectExport() :dExportPlugin() {}
	~NewtonMeshEffectExport(){}
	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return "*.nme";}
	virtual const char* GetFileDescription () {return "nme export";}
	virtual const char* GetSignature () {return "Export Newton Mesh Effect file";}
	virtual void Export (const char* const fileName, dPluginInterface* const interface);

	static void SerializeCallback (void* const serializeHandle, const void* const buffer, int size);
};

#endif