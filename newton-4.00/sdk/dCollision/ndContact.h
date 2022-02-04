/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_CONTACT_H__
#define __ND_CONTACT_H__

#include "ndCollisionStdafx.h"
#include "ndConstraint.h"
#include "ndContactNotify.h"
#include "ndContactSolver.h"

class ndBodyKinematic;
class ndShapeInstance;

#define D_MAX_CONTATCS					128
#define D_CONSTRAINT_MAX_ROWS			(3 * 16)
#define D_RESTING_CONTACT_PENETRATION	(D_PENETRATION_TOL + ndFloat32 (1.0f / 1024.0f))
#define D_DIAGONAL_PRECONDITIONER		ndFloat32 (25.0f)

D_MSV_NEWTON_ALIGN_32
class ndContactPoint
{
	public:
	ndVector m_point;
	ndVector m_normal;
	const ndBodyKinematic* m_body0;
	const ndBodyKinematic* m_body1;
	const ndShapeInstance* m_shapeInstance0;
	const ndShapeInstance* m_shapeInstance1;
	ndInt64 m_shapeId0;
	ndInt64 m_shapeId1;
	ndFloat32 m_penetration;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndContactMaterial: public ndContactPoint
{
	public:
	ndContactMaterial()
		:m_dir0(ndVector::m_zero)
		,m_dir1(ndVector::m_zero)
		,m_material()
	{
		m_dir0_Force.Clear();
		m_dir1_Force.Clear();
		m_normal_Force.Clear();
	}
	ndVector m_dir0;
	ndVector m_dir1;
	ndForceImpactPair m_normal_Force;
	ndForceImpactPair m_dir0_Force;
	ndForceImpactPair m_dir1_Force;
	ndMaterial m_material;
} D_GCC_NEWTON_ALIGN_32;

class ndContactPointList : public ndList<ndContactMaterial, ndContainersFreeListAlloc<ndContactMaterial>>
{
	public:
	ndContactPointList()
		:ndList<ndContactMaterial, ndContainersFreeListAlloc<ndContactMaterial>>()
	{
	}
};

D_MSV_NEWTON_ALIGN_32 
class ndContact: public ndConstraint
{
	public:
	D_COLLISION_API ndContact();
	D_COLLISION_API virtual ~ndContact();

	D_COLLISION_API virtual ndBodyKinematic* GetBody0() const;
	D_COLLISION_API virtual ndBodyKinematic* GetBody1() const;

	D_COLLISION_API void AttachToBodies();
	D_COLLISION_API void DetachFromBodies();

	ndContact* GetAsContact() { return this; }
	ndFloat32 GetPruningTolerance() const;

	const ndMaterial* GetMaterial() const;

	virtual ndUnsigned32 GetRowsCount() const;
	virtual void JacobianDerivative(ndConstraintDescritor& desc);
	virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc);

	ndContactPointList& GetContactPoints();
	const ndContactPointList& GetContactPoints() const;

	bool IsSkeletonSelftCollision() const;
	bool IsSkeletonIntraCollision() const;
	
	private:
	void SetBodies(ndBodyKinematic* const body0, ndBodyKinematic* const body1);
	void CalculatePointDerivative(ndInt32 index, ndConstraintDescritor& desc, const ndVector& dir, const dgPointParam& param) const;
	void JacobianContactDerivative(ndConstraintDescritor& desc, const ndContactMaterial& contact, ndInt32 normalIndex, ndInt32& frictionIndex);

	ndVector m_positAcc;
	ndQuaternion m_rotationAcc;
	ndVector m_separatingVector;
	ndContactPointList m_contacPointsList;
	ndBodyKinematic* m_body0;
	ndBodyKinematic* m_body1;
	ndMaterial* m_material;
	ndFloat32 m_timeOfImpact;
	ndFloat32 m_separationDistance;
	ndFloat32 m_contactPruningTolereance;
	ndUnsigned32 m_maxDOF;
	ndUnsigned32 m_sceneLru;
	ndUnsigned32 m_isDead : 1;
	ndUnsigned32 m_isAttached : 1;
	ndUnsigned32 m_isIntersetionTestOnly : 1;
	ndUnsigned32 m_skeletonIntraCollision : 1;
	ndUnsigned32 m_skeletonSelftCollision : 1;
	static ndVector m_initialSeparatingVector;

	friend class ndScene;
	friend class ndContactArray;
	friend class ndBodyKinematic;
	friend class ndContactSolver;
	friend class ndShapeInstance;
	friend class ndConvexCastNotify;
	friend class ndShapeConvexPolygon;
	friend class ndBodyPlayerCapsuleContactSolver;
} D_GCC_NEWTON_ALIGN_32 ;

inline const ndMaterial* ndContact::GetMaterial() const
{
	return m_material;
}

inline ndFloat32 ndContact::GetPruningTolerance() const
{
	return m_contactPruningTolereance;
}

inline ndUnsigned32 ndContact::GetRowsCount() const
{
	return m_maxDOF;
}

inline ndBodyKinematic* ndContact::GetBody0() const
{
	return m_body0;
}

inline ndBodyKinematic* ndContact::GetBody1() const
{
	return m_body1;
}

inline ndContactPointList& ndContact::GetContactPoints()
{
	return m_contacPointsList;
}

inline const ndContactPointList& ndContact::GetContactPoints() const
{
	return m_contacPointsList;
}

//inline bool ndContact::IsActive() const
//{
//	return m_active ? true : false;
//}

inline bool ndContact::IsSkeletonSelftCollision() const
{
	return m_skeletonSelftCollision ? true : false;;
}

inline bool ndContact::IsSkeletonIntraCollision() const
{
	return m_skeletonIntraCollision ? true : false;
}

#endif 

