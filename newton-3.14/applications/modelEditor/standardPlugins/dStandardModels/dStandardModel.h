/////////////////////////////////////////////////////////////////////////////
// Name:        dStandardModel.h
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

#ifndef _D_STANDARD_MODEL_H_
#define _D_STANDARD_MODEL_H_


class dStandardModel: public dModelPlugin
{
	public:
	dStandardModel();
	~dStandardModel();

	virtual const char* GetMenuName () { return "Create Standard Model";}
	virtual const char* GetDescription () {return "Create a simple model";}

	
	virtual const char* GetSignature () {return "ModelPlugin";}
	virtual bool Create (dPluginInterface* const interface);

	static dStandardModel* GetPlugin();


	private:
};

#endif