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

#if !defined(AFX_DGCONSTRAINT_H__F9EC24E0_6E0F_4CD5_909E_A5F5E1AC7C0B_H)
#define AFX_DGCONSTRAINT_H__F9EC24E0_6E0F_4CD5_909E_A5F5E1AC7C0B_H

#include "dgBodyMasterList.h"

#define DG_MAX_BOUND						dgFloat32 (1.0e15f)
#define DG_MIN_BOUND						(-DG_MAX_BOUND)


#define DG_BILATERAL_CONSTRAINT				-1
#define DG_NORMAL_CONSTRAINT				-2 
#define DG_BILATERAL_FRICTION_CONSTRAINT	-3 

#define DG_CONSTRAINT_MAX_ROWS				 (3 * 16)

#define MIN_JOINT_PIN_LENGTH				dgFloat32 (50.0f)

class dgBody;
class dgWorld;
class dgConstraint;
class dgBilateralBounds;

typedef void (dgApi *ConstraintsForceFeeback) (const dgConstraint& me, dgFloat32 timestep, dgInt32 threadIndex);

class dgConstraintInfo
{
	public:
	dgMatrix m_attachMatrix_0;
	dgMatrix m_attachMatrix_1;
	dgFloat32 m_minLinearDof[3];
	dgFloat32 m_maxLinearDof[3];
	dgFloat32 m_minAngularDof[3];
	dgFloat32 m_maxAngularDof[3];
	dgBody* m_attachBody_0;
	dgBody* m_attachBody_1;
	dgFloat32 m_extraParameters[64];
	dgInt32 m_collideCollisionOn;
	char m_discriptionType[64];
};


class dgJointCallbackParam
{
	public:
	dgFloat32 m_accel;
	dgFloat32 m_minFriction;
	dgFloat32 m_maxFriction;
	dgFloat32 m_timestep;
};


class dgForceImpactPair
{
	public:
	dgFloat32 m_force;
	dgFloat32 m_impact;
};

class dgBilateralBounds
{
	public:
	dgFloat32 m_low;
	dgFloat32 m_upper;
	dgInt32 m_normalIndex;
	dgForceImpactPair* m_jointForce;
};

DG_MSC_VECTOR_ALIGMENT
class dgJacobian
{
	public:
	dgVector m_linear;
	dgVector m_angular;
} DG_GCC_VECTOR_ALIGMENT;

DG_MSC_VECTOR_ALIGMENT
class dgJacobianPair
{
	public:
	dgJacobian m_jacobianM0;
	dgJacobian m_jacobianM1;
} DG_GCC_VECTOR_ALIGMENT;

class dgJacobianMatrixElement;
class dgJointAccelerationDecriptor
{
	public: 
	dgInt32 m_rowsCount;
	dgFloat32 m_timeStep;
	dgFloat32 m_invTimeStep;
	dgFloat32 m_firstPassCoefFlag;
	dgJacobianMatrixElement *m_rowMatrix;
};


DG_MSC_VECTOR_ALIGMENT
class dgContraintDescritor
{
	public:
	dgJacobianPair m_jacobian[DG_CONSTRAINT_MAX_ROWS];
	dgBilateralBounds m_forceBounds[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_jointAccel[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_jointStiffness[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_restitution[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_penetration[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 m_penetrationStiffness[DG_CONSTRAINT_MAX_ROWS];
	bool m_isMotor[DG_CONSTRAINT_MAX_ROWS];
	dgWorld* m_world;
	dgInt32 m_threadIndex;
	dgFloat32 m_timestep;
	dgFloat32 m_invTimestep;
} DG_GCC_VECTOR_ALIGMENT;


typedef void (dgApi *OnConstraintDestroy) (dgConstraint& me);

//DG_MSC_VECTOR_ALIGMENT
DG_MSC_VECTOR_ALIGMENT
class dgConstraint
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	enum dgConstraintID
	{
		m_ballConstraint,
		m_hingeConstraint,
		m_sliderConstraint,
		m_contactConstraint,
		m_upVectorConstraint,
		m_universalConstraint,
		m_corkScrewConstraint,
		m_unknownConstraint
	};


	dgUnsigned32 GetId () const;
	dgBody* GetBody0 ()	const;
	dgBody* GetBody1 ()	const;

	void SetBodies (dgBody* const body0, dgBody* const body1);

	dgBodyMasterListRow::dgListNode* GetLink0()	const;
	dgBodyMasterListRow::dgListNode* GetLink1()	const;
	void* GetUserData () const;

	bool IsActive() const;
	bool IsCollidable () const;
	
	virtual void ResetMaxDOF();
	dgInt32 GetMaxDOF() const;
	
	void SetUserData (void *userData);
	void SetCollidable (bool state);
	virtual void SetDestructorCallback (OnConstraintDestroy destructor) = 0;

	virtual dgFloat32 GetStiffness() const;
	virtual void SetStiffness(dgFloat32 stiffness);
	virtual void GetInfo (dgConstraintInfo* const info) const;
	

	class dgPointParam
	{
		public:
		dgVector m_r0;
		dgVector m_r1;
		dgVector m_posit0;
		dgVector m_posit1;
		dgVector m_veloc0;
		dgVector m_veloc1;
		dgVector m_centripetal0;
		dgVector m_centripetal1;
		dgFloat32 m_stiffness;
	};


	protected:
	dgConstraint();
	virtual ~dgConstraint();

	virtual bool IsBilateral () const;
	

	virtual dgUnsigned32 JacobianDerivative (dgContraintDescritor& params) = 0; 

	virtual void JointAccelerations(dgJointAccelerationDecriptor* const params) = 0; 
	virtual void JointVelocityCorrection(dgJointAccelerationDecriptor* const params) = 0; 

	void SetUpdateFeedbackFunction (ConstraintsForceFeeback function);
	void InitPointParam (dgPointParam& param, dgFloat32 stiffness, const dgVector& p0Global, const dgVector& p1Global) const;

	
	void InitInfo (dgConstraintInfo* const info) const;

	void* m_userData;
	dgBody* m_body0;
	dgBody* m_body1;
	dgBodyMasterListRow::dgListNode* m_link0;
	dgBodyMasterListRow::dgListNode* m_link1;
	ConstraintsForceFeeback m_updaFeedbackCallback;
	dgUnsigned32 m_dynamicsLru;
	dgUnsigned32 m_index;
	
	dgUnsigned32 m_maxDOF			:  6;
	dgUnsigned32 m_constId			:  6;		
	dgUnsigned32 m_enableCollision	:  1;
	dgUnsigned32 m_useExactSolver	:  1;
	dgUnsigned32 m_solverActive		:  1;
	dgUnsigned32 m_contactActive	:  1;
	
	friend class dgWorld;
	friend class dgAmpInstance;
	friend class dgJacobianMemory;
	friend class dgBodyMasterList;
	friend class dgWorldDynamicUpdate;
	friend class dgParallelSolverJointAcceleration;
	friend class dgParallelSolverInitFeedbackUpdate;
	friend class dgParallelSolverBuildJacobianMatrix;
	friend class dgBroadPhaseMaterialCallbackWorkerThread;
} DG_GCC_VECTOR_ALIGMENT;

inline dgConstraint::dgConstraint() 
	:m_userData(NULL)
	,m_body0(NULL)
	,m_body1(NULL)
	,m_link0(NULL)
	,m_link1(NULL)
	,m_updaFeedbackCallback(NULL)
	,m_dynamicsLru(0)
	,m_index(0)
	,m_maxDOF(6)
	,m_constId(m_unknownConstraint)
	,m_enableCollision(false)
	,m_useExactSolver(false)
	,m_solverActive(false)
	,m_contactActive(false)
{
	dgAssert ((((dgUnsigned64) this) & 15) == 0);
}

inline dgConstraint::~dgConstraint()
{
}

inline void dgConstraint::SetUpdateFeedbackFunction (ConstraintsForceFeeback function)
{
	m_updaFeedbackCallback = function;
}

inline bool dgConstraint::IsCollidable () const
{
	return m_enableCollision ? true : false;
}

inline void dgConstraint::SetCollidable (bool state)
{
	m_enableCollision = dgUnsigned32 (state);
}

inline dgUnsigned32 dgConstraint::GetId () const
{
	return m_constId;
}

inline dgBody* dgConstraint::GetBody0 () const
{
	return m_body0;
}

inline dgBody* dgConstraint::GetBody1 () const
{
	return m_body1;
}

inline void dgConstraint::SetBodies (dgBody* const body0, dgBody* const body1)
{
	m_body0 = body0;
	m_body1 = body1;
}


inline dgBodyMasterListRow::dgListNode* dgConstraint::GetLink0()	const
{
	return m_link0;
}
inline dgBodyMasterListRow::dgListNode* dgConstraint::GetLink1()	const
{
	return m_link1;
}


inline dgFloat32 dgConstraint::GetStiffness() const
{
	return dgFloat32 (1.0f);
}

inline void dgConstraint::SetStiffness(dgFloat32 stiffness)
{
}


inline void dgConstraint::ResetMaxDOF()
{
}

inline dgInt32 dgConstraint::GetMaxDOF() const
{
	return dgInt32 (m_maxDOF);
}

inline bool dgConstraint::IsActive() const
{
	return m_contactActive ? true : false;
}

#endif // !defined(AFX_DGCONSTRAINT_H__F9EC24E0_6E0F_4CD5_909E_A5F5E1AC7C0B_H)

