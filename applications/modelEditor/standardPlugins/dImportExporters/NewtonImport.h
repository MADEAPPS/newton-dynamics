/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonImport.h
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

#ifndef _NEWTON_IMPORT_H_
#define _NEWTON_IMPORT_H_


class NewtonImport: public dImportPlugin
{
	public:
	NewtonImport();
	~NewtonImport();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return ".bin";}
	virtual const char* GetFileDescription () {return "Newton serialized binary file";}

	virtual const char* GetSignature () {return "Newton Binary Import";}
	virtual bool Import (const char* const fileName, dPluginInterface* const interface);

	static NewtonImport* GetPlugin();

	private:
	void BuildSceneFromWorld(NewtonWorld* const world);


	static void DeserializeFile (void* const serializeHandle, void* const buffer, int size);
	static void BodyDeserialization (NewtonBody* const body, NewtonDeserializeCallback serializecallback, void* const serializeHandle);

};

#endif