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

#ifndef _OFF_EXPORT_IMPORT_H_
#define _OFF_EXPORT_IMPORT_H_


class OffImport: public dImportPlugin
{
	public:
	static dImportPlugin* GetPlugin();

	public:
	OffImport():dImportPlugin() {}
	~OffImport() {}

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return "*.off";}
	virtual const char* GetFileDescription () {return "off mesh import";}

	virtual const char* GetSignature () {return "Import Object File Format file";}
	virtual bool Import (const char* const fileName, dPluginInterface* const interface);
};



class OffExport: public dExportPlugin
{
	public:
	static dExportPlugin* GetPlugin();

	public:
	OffExport():dExportPlugin(){}
	~OffExport(){}

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return "*.off";}
	virtual const char* GetFileDescription () {return "save selected mesh as OFF file";}

	virtual const char* GetSignature () {return "Off mesh export";}
	virtual void Export (const char* const fileName, dPluginInterface* const interface);
};

#endif