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
class ndRenderSceneCamera;
class ndRenderSceneNodeInstance;
class ndRenderPassShadowsImplement;

class ndTransform
{
	public:
	ndTransform();
	ndTransform(const ndMatrix& matrix);
	ndTransform(const ndQuaternion& rotation, const ndVector& position);
	
	ndMatrix GetMatrix() const;

	ndVector m_position;
	ndQuaternion m_rotation;
};

class ndRenderSceneNode : public ndContainersFreeListAlloc<ndRenderSceneNode>
{
	public:

	ndRenderSceneNode(const ndMatrix& matrix);
	ndRenderSceneNode(const ndRenderSceneNode& src);
	virtual ~ndRenderSceneNode();

	virtual ndRenderSceneNode* Clone() const;

	ndRenderSceneNode* GetRoot() const;
	ndRenderSceneNode* GetParent() const;
	const ndList<ndSharedPtr<ndRenderSceneNode>>& GetChildren() const;

	virtual void AddChild(const ndSharedPtr<ndRenderSceneNode>& child);
	virtual void RemoveChild(const ndSharedPtr<ndRenderSceneNode> child);

	virtual ndRenderSceneCamera* GetAsCamera();
	virtual const ndRenderSceneCamera* GetAsCamera() const;

	virtual ndRenderSceneNodeInstance* GetAsInstance();
	virtual const ndRenderSceneNodeInstance* GetAsInstance() const;

	ndRenderSceneCamera* FindCameraNode();
	const ndRenderSceneCamera* FindCameraNode() const;

	ndSharedPtr<ndRenderSceneNode> GetSharedPtr() const;
	ndRenderSceneNode* FindByName(const ndString& name) const;
	ndRenderSceneNode* FindByClosestMatch(const ndString& name) const;

	ndSharedPtr<ndRenderPrimitive>GetPrimitive() const;
	void SetPrimitive(const ndSharedPtr<ndRenderPrimitive>& primitive);

	ndRender* GetOwner() const;
	virtual ndMatrix GetMatrix() const;
	virtual void SetMatrix(const ndQuaternion& rotation, const ndVector& position);
	virtual void SetTransform(const ndQuaternion& rotation, const ndVector& position);

	void InterpolateTransforms(ndFloat32 param);
	ndMatrix CalculateGlobalMatrix(const ndRenderSceneNode* const root = nullptr) const;
	ndMatrix CalculateGlobalTransform(const ndRenderSceneNode* const root = nullptr) const;

	ndTransform GetTransform() const;
	void SetTransform(const ndTransform& transform);

	ndRenderSceneNode* IteratorNext();
	ndRenderSceneNode* IteratorFirst();
	const ndRenderSceneNode* IteratorNext() const;
	const ndRenderSceneNode* IteratorFirst() const;

	virtual void Render(const ndRender* const owner, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const;

	ndMatrix m_matrix;			// interpolated local matrix
	ndMatrix m_globalMatrix;	// world space matrix calculated each frame for rendering
	ndTransform m_transform0;
	ndTransform m_transform1;
	ndString m_name;

	protected:
	virtual void ApplyPrimitiveTransforms();
	virtual ndRenderSceneNode* CloneSkeleton() const;
	void ClonePrimitives(const ndRenderSceneNode& src);
	
	ndRender* m_owner;
	ndRenderSceneNode* m_parent;
	ndSharedPtr<ndRenderPrimitive> m_primitive;
	ndList<ndSharedPtr<ndRenderSceneNode>> m_children;
	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* m_selfChildNode;
	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* m_sceneHandle;

	public:
	bool m_isVisible;
	friend class ndRender;
	friend class ndRenderPassColor;
	friend class ndRenderPassTransparency;
	friend class ndRenderSceneNodeInstanceImplement;
};

#endif