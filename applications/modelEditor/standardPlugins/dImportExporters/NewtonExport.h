/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonExport.h
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

#ifndef _NEWTON_EXPORT_H_
#define _NEWTON_EXPORT_H_


class NewtonExport: public dExportPlugin
{
	public:
	NewtonExport();
	~NewtonExport();

	static NewtonExport* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return ".bin";}
	virtual const char* GetFileDescription () {return "Newton serialized binary file";}

	virtual const char* GetSignature () {return "Newton Binary Export";}
	virtual void Export (const char* const fileName, dPluginInterface* const interface);

		
};

#endif