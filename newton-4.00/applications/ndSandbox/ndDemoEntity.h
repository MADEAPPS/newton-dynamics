/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
class ndShaderPrograms;
class ndDemoMeshInterface;

class ndDemoEntityNotify: public ndBodyNotify
{
	public:
	D_CLASS_REFLECTION(ndDemoEntityNotify);
	ndDemoEntityNotify(const ndLoadSaveBase::ndLoadDescriptor& desc);
	ndDemoEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody = nullptr, ndFloat32 gravity = DEMO_GRAVITY);
	virtual ~ndDemoEntityNotify();

	void* GetUserData() const
	{
		return m_entity;
	}

	virtual void OnObjectPick() const;
	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);
	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);

	virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndDemoEntity* m_entity;
	ndBodyDynamic* m_parentBody;
	ndDemoEntityManager* m_manager;
};

class ndDemoEntity : public ndNodeHierarchy<ndDemoEntity>
{
	public:

	class UserData
	{
		public:
		UserData()
		{
		}

		virtual ~UserData()
		{
		}
		
		virtual void OnRender (ndFloat32 timestep) const = 0;
	};

	ndDemoEntity(const ndDemoEntity& copyFrom);
	ndDemoEntity(const ndMatrix& matrix, ndDemoEntity* const parent);
	virtual ~ndDemoEntity(void);

	ndDemoMeshInterface* GetMesh() const;
	void SetMesh (ndDemoMeshInterface* const m_mesh, const ndMatrix& meshMatrix);

	const ndMatrix& GetMeshMatrix() const;  
	void SetMeshMatrix(const ndMatrix& matrix);  

	UserData* GetUserData ();
	void SetUserData (UserData* const data);

	ndNodeBaseHierarchy* CreateClone () const;

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

	void RenderBone() const;

	ndShapeInstance* CreateCollisionFromChildren() const;
	ndShapeInstance* CreateCompoundFromMesh(bool lowDetail = false) const;

	virtual void Render(ndFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;

	protected:
	mutable ndMatrix m_matrix;			// interpolated matrix
	ndVector m_curPosition;				// position one physics simulation step in the future
	ndVector m_nextPosition;             // position at the current physics simulation step
	ndQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	ndQuaternion m_nextRotation;         // rotation at the current physics simulation step  

	ndMatrix m_meshMatrix;
	ndDemoMeshInterface* m_mesh;
	UserData* m_userData;
	ndList <ndDemoEntity*>::ndNode* m_rootNode;
	bool m_isVisible;

	friend class ndDemoEntityNotify;
	friend class ndDemoEntityManager;
};

#endif
