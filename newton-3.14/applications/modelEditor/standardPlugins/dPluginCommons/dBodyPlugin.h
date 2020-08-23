/////////////////////////////////////////////////////////////////////////////
// Name:        dBodyPlugin.h
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

#ifndef __D_BODY_PLUGIN_H__
#define __D_BODY_PLUGIN_H__

#include "dPluginRecord.h"

class dPluginInterface;

class dBodyPlugin: public dPluginRecord
{
	public:
	dBodyPlugin(void) {};
	virtual ~dBodyPlugin(void) {};

	virtual dPluginType GetType () { return m_body;}

	virtual const char* GetMenuName () {return NULL;}
	virtual const char* GetDescription () {return NULL;}

	virtual bool Create (dPluginInterface* const interface) = 0;
};



#endif