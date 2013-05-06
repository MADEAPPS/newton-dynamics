/////////////////////////////////////////////////////////////////////////////
// Name:        pyBaseNodeInfo.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
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

#pragma once
#include "stdafx.h"
#include "pyScene.h"

template<class Info>
class pyBaseNodeInfo
{
	public:
	pyBaseNodeInfo(pyScene* scene, void* node);
	~pyBaseNodeInfo(void);


	protected:
	Info* GetInfo();


	pyScene* m_scene;
	void* m_node;
};


template<class Info>
pyBaseNodeInfo<Info>::pyBaseNodeInfo(pyScene* scene, void* node)
{
	m_scene = scene;
	m_node = node;
}

template<class Info>
pyBaseNodeInfo<Info>::~pyBaseNodeInfo()
{
}


template<class Info>
Info* pyBaseNodeInfo<Info>::GetInfo()
{
	return (Info*) m_scene->GetScene()->GetInfoFromNode((dScene::dTreeNode*) m_node);
}

