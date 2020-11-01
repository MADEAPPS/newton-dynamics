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

#ifndef __DEMO_ENTITY_H__
#define __DEMO_ENTITY_H__

#include "ndDemoEntityManager.h"

class ndDemoEntity;
class ndShaderPrograms;
class ndDemoMeshInterface;

class ndDemoEntityNotify: public ndBodyNotify
{
	public:
	ndDemoEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, dFloat32 gravity = -10.0f);
	virtual ~ndDemoEntityNotify();

	void* GetUserData() const
	{
		return m_entity;
	}

	virtual void OnTranform(dInt32 threadIndex, const dMatrix& matrix);
	virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep);

	ndDemoEntity* m_entity;
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
	//ndDemoEntity(ndDemoEntityManager& world, const dScene* const scene, dScene::dTreeNode* const rootSceneNode, dTree<ndDemoMeshInterface*, dScene::dTreeNode*>& meshCache, ndDemoEntityManager::EntityDictionary& entityDictionary, ndDemoEntity* const parent = nullptr);
	virtual ~ndDemoEntity(void);

	ndDemoMeshInterface* GetMesh() const;
	void SetMesh (ndDemoMeshInterface* const m_mesh, const dMatrix& meshMatrix);

	const dMatrix& GetMeshMatrix() const;  
	void SetMeshMatrix(const dMatrix& matrix);  

	//void SetMatrix(const dMatrix& matrix);

	UserData* GetUserData ();
	void SetUserData (UserData* const data);

	dNodeBaseHierarchy* CreateClone () const;
	//static ndDemoEntity* LoadNGD_mesh (const char* const fileName, NewtonWorld* const world, const ndShaderPrograms& shaderCache);

	const dMatrix& GetRenderMatrix () const;
	dMatrix CalculateGlobalMatrix (const ndDemoEntity* const root = nullptr) const;

	dMatrix GetNextMatrix () const;
	dMatrix GetCurrentMatrix () const;
	virtual void SetMatrix(ndDemoEntityManager& world, const dQuaternion& rotation, const dVector& position);
	virtual void SetNextMatrix (ndDemoEntityManager& world, const dQuaternion& rotation, const dVector& position);

	void InterpolateMatrixUnsafe(dFloat32 param);
	void SetMatrixUsafe(const dQuaternion& rotation, const dVector& position);

	virtual void ResetMatrix(ndDemoEntityManager& world, const dMatrix& matrix);
	virtual void InterpolateMatrix (ndDemoEntityManager& world, dFloat32 param);
	dMatrix CalculateInterpolatedGlobalMatrix (const ndDemoEntity* const root = nullptr) const;

	void RenderBone() const;
	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;
	//NewtonCollision* CreateCollisionFromchildren(NewtonWorld* const world) const;

	//static void TransformCallback(const NewtonBody* body, const dFloat32* matrix, int threadIndex);

	protected:
	mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step  

	dMatrix m_meshMatrix;
	ndDemoMeshInterface* m_mesh;
	UserData* m_userData;
	dList <ndDemoEntity*>::dListNode* m_rootNode;
	dSpinLock m_lock;
	bool m_isVisible;

	friend class ndDemoEntityNotify;
	friend class ndDemoEntityManager;
};

#endif