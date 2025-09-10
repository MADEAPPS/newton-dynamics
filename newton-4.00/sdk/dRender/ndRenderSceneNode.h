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
#ifndef __ND_RENDER_SCENE_NODE_H__
#define __ND_RENDER_SCENE_NODE_H__

#include "ndRenderStdafx.h"
#include "ndRenderPrimitive.h"

class ndRender;
class ndRenderPassShadowsImplement;

class ndTransform
{
	public:
	ndTransform();
	ndTransform(const ndMatrix& matrix);
	ndTransform(const ndQuaternion& rotation, const ndVector& position);
	
	ndVector m_position;
	ndQuaternion m_rotation;
};

class ndRenderSceneNode : public ndContainersFreeListAlloc<ndRenderSceneNode>
{
	public:
	ndRenderSceneNode(const ndMatrix& matrix);
	virtual ~ndRenderSceneNode();

	void SetPrimitiveMatrix(const ndMatrix& matrix);
	void SetPrimitive(const ndSharedPtr<ndRenderPrimitive>& primitive);

	ndRender* GetOwner() const;
	virtual ndMatrix GetMatrix() const;
	virtual void SetMatrix(const ndQuaternion& rotation, const ndVector& position);
	virtual void SetTransform(const ndQuaternion& rotation, const ndVector& position);

	void InterpolateTransforms(ndFloat32 param);

	ndTransform GetTransform() const;
	void SetTransform(const ndTransform& transform);

	virtual void AddChild(const ndSharedPtr<ndRenderSceneNode>& child);
	virtual void RemoveChild(const ndSharedPtr<ndRenderSceneNode> child);

	virtual void Render(const ndRender* const owner, ndFloat32 timeStep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const;

	public:
	ndMatrix m_matrix;			// interpolated matrix
	ndMatrix m_primitiveMatrix;
	ndTransform m_transform0;
	ndTransform m_transform1;

	protected:
	ndRender* m_owner;
	ndRenderSceneNode* m_parent;
	ndSharedPtr<ndRenderPrimitive> m_primitve;
	ndList<ndSharedPtr<ndRenderSceneNode>> m_children;
	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* m_sceneHandle;

	public:
	bool m_isVisible;
	friend class ndRender;
	friend class ndRenderPassColor;
	friend class ndRenderPassTransparency;
};

#endif