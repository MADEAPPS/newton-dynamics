/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
#ifndef __ND_RENDER_SCENE_NODE_INSTANCE_H__
#define __ND_RENDER_SCENE_NODE_INSTANCE_H__

#include "ndRenderStdafx.h"
#include "ndRenderSceneNode.h"
#include "ndRenderPrimitiveMesh.h"

class ndRenderSceneNodeInstanceImplement;
class ndRenderSceneNodeInstance : public ndRenderSceneNode
{
	public:
	ndRenderSceneNodeInstance(const ndMatrix& matrix, const ndRenderPrimitiveMesh::ndDescriptor& descriptor);

	virtual ndRenderSceneNodeInstance* GetAsInstance() override;
	virtual const ndRenderSceneNodeInstance* GetAsInstance() const override;

	void Finalize();
	virtual void Render(const ndRender* const owner, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const override;

	ndRenderPrimitiveMesh::ndDescriptor m_descriptor;
	ndSharedPtr<ndRenderSceneNodeInstanceImplement> m_implement;
	bool m_isInitialized;
};

#endif