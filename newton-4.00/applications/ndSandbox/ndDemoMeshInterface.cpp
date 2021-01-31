/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMeshInterface.h"


std::thread::id ndDemoMeshInterface::m_renderThread;
dSpinLock ndDemoMeshInterface::m_lock;
dList<ndDemoMeshInterface*> ndDemoMeshInterface::m_pendingRelease;

ndDemoSubMeshMaterial::ndDemoSubMeshMaterial()
	:m_ambient(0.8f, 0.8f, 0.8f, 1.0f)
	,m_diffuse(0.8f, 0.8f, 0.8f, 1.0f)
	,m_specular(1.0f, 1.0f, 1.0f, 1.0f)
	,m_opacity(1.0f)
	,m_shiness(100.0f)
	,m_textureHandle(0)
{
	m_textureName[0] = 0;
}

ndDemoSubMeshMaterial::~ndDemoSubMeshMaterial()
{
	if (m_textureHandle)
	{
		ReleaseTexture(m_textureHandle);
	}
}

ndDemoMeshInterface::ndDemoMeshInterface()
	:dClassAlloc()
	,dRefCounter<ndDemoMeshInterface>()
	,m_name()
	,m_isVisible(true)
{
}

ndDemoMeshInterface::~ndDemoMeshInterface()
{
}

dInt32 ndDemoMeshInterface::Release()
{
	std::thread::id threadId = std::this_thread::get_id();
	if (threadId == m_renderThread)
	{
		return dRefCounter<ndDemoMeshInterface>::Release();
	}
	else
	{
		dScopeSpinLock lock(m_lock);
		m_pendingRelease.Append(this);
		return 0;
	}
}

void ndDemoMeshInterface::ReleasePending()
{
	if (m_pendingRelease.GetCount())
	{
		dScopeSpinLock lock(m_lock);
		while (m_pendingRelease.GetCount())
		{
			m_pendingRelease.GetFirst()->GetInfo()->Release();
			m_pendingRelease.Remove(m_pendingRelease.GetFirst());
		}
	}
}

const dString& ndDemoMeshInterface::GetName () const
{
	return m_name;
}


bool ndDemoMeshInterface::GetVisible () const
{
	return m_isVisible;
}

void ndDemoMeshInterface::SetVisible (bool visibilityFlag)
{
	m_isVisible = visibilityFlag;
}

ndDemoSubMesh::ndDemoSubMesh ()
	:m_material()
	,m_indexCount(0)
	,m_segmentStart(0)
	,m_hasTranparency(false)
{
}

ndDemoSubMesh::~ndDemoSubMesh ()
{
}

void ndDemoSubMesh::SetOpacity(dFloat32 opacity)
{
	m_material.m_opacity = opacity;
	m_material.m_ambient.m_w = opacity;
	m_material.m_diffuse.m_w = opacity;
	m_material.m_specular.m_w = opacity;
	m_hasTranparency = (opacity <= 0.99f) ? true : false;
}

