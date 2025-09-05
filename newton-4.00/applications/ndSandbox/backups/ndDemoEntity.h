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

#ifndef __DEMO_ENTITY_H__
#define __DEMO_ENTITY_H__

#include "ndDemoEntityManager.h"
#include "ndPhysicsUtils.h"

class ndDemoEntity;
class ndAnimKeyframe;
class ndShaderCache;
class ndDemoMeshInterface;
class ndShadowMapRenderPass;

//class ndDemoEntity: public ndNodeHierarchy<ndDemoEntity>
class ndDemoEntity : public ndSharedNodeHierarchy<ndDemoEntity>
{
	public:
	ndDemoEntity(const ndMatrix& matrix);
	ndDemoEntity(const ndDemoEntity& copyFrom);
	ndDemoEntity(ndDemoEntityManager* const scene, ndMesh* const meshEffect);
	virtual ~ndDemoEntity(void);

	ndSharedPtr<ndDemoMeshInterface> GetMesh();
	ndSharedPtr<ndDemoMeshInterface> GetMesh() const;
	void SetMesh(ndSharedPtr<ndDemoMeshInterface> mesh, const ndMatrix& meshMatrix = ndGetIdentityMatrix());

	const ndMatrix& GetMeshMatrix() const;  
	void SetMeshMatrix(const ndMatrix& matrix);  

	ndDemoEntity* CreateClone () const;

	const ndMatrix& GetRenderMatrix () const;
	ndMatrix CalculateGlobalMatrix (const ndDemoEntity* const root = nullptr) const;

	ndMatrix GetNextMatrix () const;
	ndMatrix GetCurrentMatrix () const;
	ndAnimKeyframe GetCurrentTransform() const;
	virtual void SetMatrix(const ndQuaternion& rotation, const ndVector& position);
	virtual void SetNextMatrix (const ndQuaternion& rotation, const ndVector& position);
	virtual void ResetMatrix(const ndMatrix& matrix);
	virtual void InterpolateMatrix (ndFloat32 param);
	ndMatrix CalculateInterpolatedGlobalMatrix (const ndDemoEntity* const root = nullptr) const;

	void RenderBone(ndDemoEntityManager* const scene, const ndMatrix& nodeMatrix) const;

	ndShapeInstance* CreateCollision() const;
	ndShapeInstance* CreateCollisionFromChildren() const;
	ndShapeInstance* CreateCompoundFromMesh(bool lowDetail = false);

	void RenderSkeleton(ndDemoEntityManager* const scene, const ndMatrix& matrix) const;
	virtual void Render(ndFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;

	ndDemoEntity* FindBySubString(const char* const subString) const;
	ndSharedPtr<ndDemoEntity> Find(const ndSharedPtr<ndDemoEntity>& self, const char* const name) const;

	const ndString& GetName() const;
	void SetName(const ndString& name);

	bool CastShadow() const;
	void SetShadowMode(bool mode);

	void Hide();
	void UnHide();

	virtual void RenderShadowMap(ndShadowMapRenderPass* const shadowMap, const ndMatrix& matrix);

	protected:
	mutable ndMatrix m_matrix;			// interpolated matrix
	ndVector m_curPosition;				// position one physics simulation step in the future
	ndVector m_nextPosition;             // position at the current physics simulation step
	ndQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	ndQuaternion m_nextRotation;         // rotation at the current physics simulation step  

	ndMatrix m_meshMatrix;
	ndSharedPtr<ndDemoMeshInterface> m_mesh;
	ndList <ndSharedPtr<ndDemoEntity>>::ndNode* m_rootNode;
	ndString m_name;
	ndSpinLock m_lock;
	bool m_isDead;
	bool m_isVisible;
	bool m_castShadow;

	friend class ndPhysicsWorld;
	friend class ndDemoEntityNotify;
	friend class ndDemoEntityManager;
};

#endif
