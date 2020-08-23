/////////////////////////////////////////////////////////////////////////////
// Name:        pyObject.h
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

#include "StdAfx.h"
#include "pyObject.h"
#include "pyScene.h"

pyObject::pyObject(pyScene* scene, void* sceneNode)
	:pyBaseNodeInfo<dSceneNodeInfo>(scene, sceneNode)
{
}

pyObject::~pyObject(void)
{
}


void pyObject::SetName (const char* name)
{
	dSceneNodeInfo* info = GetInfo();
	info->SetName(name);
}


dSceneNodeInfo* pyObject::GetParentInfo()
{
	dScene::dTreeNode* node = (dScene::dTreeNode*) m_node;
	for (void* ptr = m_scene->GetScene()->GetFirstParent(node); ptr; ptr = m_scene->GetScene()->GetNextParent(node, ptr)) {
		dScene::dTreeNode* parentNode = m_scene->GetScene()->GetNodeFromLink(ptr);
		dNodeInfo* info = m_scene->GetScene()->GetInfoFromNode(parentNode);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			return (dSceneNodeInfo*) info;
		}
	}

	return NULL;
}

pyVertex pyObject::GetLocalPosition()
{
	pyVertex posit;

	dSceneNodeInfo* info = GetInfo();
	dSceneNodeInfo* parent = GetParentInfo();

	dMatrix matrix (info->GetTransform());
	if (parent) {
		matrix = matrix * parent->GetTransform().Inverse4x4();
	}
	posit.x = matrix.m_posit.m_x;
	posit.y = matrix.m_posit.m_y;
	posit.z = matrix.m_posit.m_z;
	return posit;
}

pyVertex pyObject::GetLocalEulers()
{
	dSceneNodeInfo* info = GetInfo();
	dSceneNodeInfo* parent = GetParentInfo();

	dMatrix matrix (info->GetTransform());
	if (parent) {
		matrix = matrix * parent->GetTransform().Inverse4x4();
	}

	dVector scale; 
	dMatrix localMatrix;
	dMatrix stretchAxis;
	matrix.PolarDecomposition (localMatrix, scale, stretchAxis);

	pyVertex euler;
	dVector eulerAngle (localMatrix.GetXYZ_EulerAngles ());
	euler.x = eulerAngle.m_x;
	euler.y = eulerAngle.m_y;
	euler.z = eulerAngle.m_z;
	return euler;
}

pyVertex pyObject::GetLocalScale()
{
	dSceneNodeInfo* info = GetInfo();
	dSceneNodeInfo* parent = GetParentInfo();

	dMatrix matrix (info->GetTransform());
	if (parent) {
		matrix = matrix * parent->GetTransform().Inverse4x4();
	}

	dVector localScale; 
	dMatrix localMatrix;
	dMatrix stretchAxis;
	matrix.PolarDecomposition (localMatrix, localScale, stretchAxis);

	pyVertex scale;
	scale.x = localScale.m_x;
	scale.y = localScale.m_y;
	scale.z = localScale.m_z;
	return scale;
}

pyMatrix4x4 pyObject::GetMatrix4x4()
{
	dSceneNodeInfo* info = GetInfo();
	dSceneNodeInfo* parent = GetParentInfo();

	dMatrix matrix (info->GetTransform());
	if (parent) {
		matrix = matrix * parent->GetTransform().Inverse4x4();
	}


	pyMatrix4x4 matrix4x4;
	int index = 0;
	double* element = &matrix4x4.e00;
	for (int i = 0; i < 4; i ++) {
		for (int j = 0; j < 4; j ++) {
			element[index] = matrix[i][j];
			index ++;
		}
	}
	return matrix4x4;
}


void pyObject::SetMatrix(double x, double y, double z, double pitch, double yaw, double roll, double scaleX, double scaleY, double scaleZ)
{
	dSceneNodeInfo* info = GetInfo();
	info->SetPosition(dVector (dFloat(x), dFloat(y), dFloat(z), 1.0f));
	info->SetEulers(dVector (dFloat(pitch), dFloat(yaw), dFloat(roll), 1.0f));
	info->SetScale (dVector (dFloat(scaleX), dFloat(scaleY), dFloat(scaleZ), 1.0f));
}