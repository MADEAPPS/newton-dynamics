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
class ndShaderPrograms;
class ndDemoMeshInterface;

class ndDemoEntityNotify: public ndBodyNotify
{
	public:
	ndDemoEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody = nullptr, dFloat32 gravity = DEMO_GRAVITY);
	virtual ~ndDemoEntityNotify();

	void* GetUserData() const
	{
		return m_entity;
	}

	virtual void OnObjectPick() const;
	virtual void OnTranform(dInt32 threadIndex, const dMatrix& matrix);
	virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep);

	ndDemoEntity* m_entity;
	ndBodyDynamic* m_parentBody;
	ndDemoEntityManager* m_manager;
};

class ndDemoEntity : public dNodeHierarchy<ndDemoEntity>
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
		
		virtual void OnRender (dFloat32 timestep) const = 0;
	};

	ndDemoEntity(const ndDemoEntity& copyFrom);
	ndDemoEntity(const dMatrix& matrix, ndDemoEntity* const parent);
	virtual ~ndDemoEntity(void);

	ndDemoMeshInterface* GetMesh() const;
	void SetMesh (ndDemoMeshInterface* const m_mesh, const dMatrix& meshMatrix);

	const dMatrix& GetMeshMatrix() const;  
	void SetMeshMatrix(const dMatrix& matrix);  

	UserData* GetUserData ();
	void SetUserData (UserData* const data);

	dNodeBaseHierarchy* CreateClone () const;

	const dMatrix& GetRenderMatrix () const;
	dMatrix CalculateGlobalMatrix (const ndDemoEntity* const root = nullptr) const;

	dMatrix GetNextMatrix () const;
	dMatrix GetCurrentMatrix () const;
	virtual void SetMatrix(const dQuaternion& rotation, const dVector& position);
	virtual void SetNextMatrix (const dQuaternion& rotation, const dVector& position);

	//void SetMatrixUsafe(const dQuaternion& rotation, const dVector& position);

	virtual void ResetMatrix(const dMatrix& matrix);
	virtual void InterpolateMatrix (dFloat32 param);
	dMatrix CalculateInterpolatedGlobalMatrix (const ndDemoEntity* const root = nullptr) const;

	void RenderBone() const;
	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;
	ndShapeInstance* CreateCollisionFromchildren(ndWorld* const world) const;

	protected:
	mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step  

	dMatrix m_meshMatrix;
	ndDemoMeshInterface* m_mesh;
	UserData* m_userData;
	dList <ndDemoEntity*>::dNode* m_rootNode;
	bool m_isVisible;

	friend class ndDemoEntityNotify;
	friend class ndDemoEntityManager;
};

#endif