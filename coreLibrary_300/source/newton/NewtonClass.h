/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __NewotnClass_3HL6356GYL459020__
#define __NewotnClass_3HL6356GYL459020__

#include "NewtonStdAfx.h"
#include "Newton.h"



#define DG_TIMESTEP (1.0f / 60.0f)	
#define DG_MAX_TIMESTEP (1.0f / 30.0f)	
#define DG_MIN_TIMESTEP (1.0f / 1000.0f)	

class Newton; 


class NewtonDeadBodies: public dgTree<dgBody*, void* >
{
	public: 
	NewtonDeadBodies(dgMemoryAllocator* const allocator);
	void DestroyBodies(Newton& world);
};


class NewtonDeadJoints: public dgTree<dgConstraint*, void* >
{
	public: 
	NewtonDeadJoints(dgMemoryAllocator* const allocator);
	void DestroyJoints(Newton& world);
};


class Newton:
	public dgWorld, 
	public NewtonDeadBodies,
	public NewtonDeadJoints
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	Newton (dgFloat32 scale, dgMemoryAllocator* const allocator);
	~Newton ();

	void DestroyBody(dgBody* const body);
	void DestroyJoint(dgConstraint* const joint);

	void UpdatePhysics (dgFloat32 timestep);
	void UpdatePhysicsAsync (dgFloat32 timestep);
	static void* DefaultAllocMemory (dgInt32 size);
	static void DefaultFreeMemory (void* const ptr, dgInt32 size);

	dgFloat32 m_maxTimeStep;


	NewtonWorldDestructorCallback m_destructor;
};



class NewtonUserJoint: public dgUserConstraint  
{	
	public:
	NewtonUserJoint (dgWorld* const world, dgInt32 maxDof, NewtonUserBilateralCallback callback, NewtonUserBilateralGetInfoCallback getInfo, dgBody* const dyn0, dgBody* const dyn1);
	~NewtonUserJoint ();

	dgUnsigned32 JacobianDerivative (dgContraintDescritor& params); 

	void AddAngularRowJacobian (const dgVector& dir, dgFloat32 relAngle);
	void AddGeneralRowJacobian (const dgFloat32* const jacobian0, const dgFloat32* const jacobian1);
	void AddLinearRowJacobian (const dgVector& pivot0, const dgVector& pivot1, const dgVector& dir);

	dgFloat32 GetRowForce (dgInt32 row) const;
	void SetHighFriction (dgFloat32 friction);
	void SetLowerFriction (dgFloat32 friction);
	void SetRowStiffness (dgFloat32 stiffness);
	void SetAcceleration (dgFloat32 acceleration);
	void SetSpringDamperAcceleration (dgFloat32 springK, dgFloat32 springD);
	void GetInfo (dgConstraintInfo* const info) const;

	void SetUpdateFeedbackFunction (NewtonUserBilateralCallback getFeedback);
		
	void SetMaxContactsForExactSolver (bool mode, dgInt32 MaxCount);

	private:
	NewtonUserBilateralCallback m_jacobianFnt;
	NewtonUserBilateralGetInfoCallback m_getInfoCallback;

	dgInt32 m_rows;
	dgForceImpactPair* m_forceArray;
	dgContraintDescritor* m_param;

	dgFloat32 m_lastJointAngle;
	dgVector m_lastPosit0;
	dgVector m_lastPosit1;
};



#endif
