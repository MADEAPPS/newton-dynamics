/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __D_VEHICLE_NODE_H__
#define __D_VEHICLE_NODE_H__

#include "dStdafxVehicle.h"

class dVehicleInterface;
class dKinematicLoopJoint;
class dVehicleTireInterface;


class dVehicleNode: public dContainersAlloc
{
	public:
	DVEHICLE_API dVehicleNode(dVehicleNode* const parent, bool isLoop = false);
	DVEHICLE_API virtual ~dVehicleNode();

	DVEHICLE_API void* GetUserData();
	DVEHICLE_API void SetUserData(void* const userData);
	DVEHICLE_API virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	
	DVEHICLE_API virtual void ApplyExternalForce();
	DVEHICLE_API virtual int GetKinematicLoops(dKinematicLoopJoint** const jointArray);
	DVEHICLE_API virtual void CalculateNodeAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const;

	dVehicleNode* GetParent() const {return m_parent;}
	const dList<dVehicleNode*>& GetChildren() const {return m_children;}
	NewtonWorld* GetWorld() const {return m_world;}
	void SetWorld(NewtonWorld* const world) {m_world = world;}
	virtual dComplementaritySolver::dBodyState* GetBody() {return &m_body;}
	virtual dComplementaritySolver::dBilateralJoint* GetJoint() {return NULL;}

	bool IsLoopNode() const{return m_isLoop;}
	void SetIndex(int index){m_solverIndex = index;}
	void SetLoopNode(bool staste){m_isLoop = staste;}

	virtual dVehicleInterface* GetAsVehicle() const { return NULL; }
	virtual dVehicleTireInterface* GetAsTire() const { return NULL; }

	protected:	
	virtual void RigidBodyToStates();
	virtual void Integrate(dFloat timestep);
	virtual void StatestoRigidBody(dFloat timestep);
	void CalculateAABB(const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP) const;

	void* m_userData;
	NewtonWorld* m_world;
	dVehicleNode* m_parent;
	dList<dVehicleNode*> m_children;
	
	dComplementaritySolver::dBodyState m_body;
	int m_solverIndex;
	bool m_isLoop;

	friend class dVehicleSolver;
};

class dKinematicLoopJoint: public dComplementaritySolver::dBilateralJoint
{
	public:
	dKinematicLoopJoint()
		:dComplementaritySolver::dBilateralJoint()
		,m_owner0(NULL)
		,m_owner1(NULL)
		,m_isActive(false)
	{
	}

	dVehicleNode* GetOwner0() const
	{
		return m_owner0;
	}

	dVehicleNode* GetOwner1() const
	{
		return m_owner1;
	}
	
	void SetOwners (dVehicleNode* const owner0, dVehicleNode* const owner1)
	{
		m_owner0 = owner0;
		m_owner1 = owner1;
		Init (m_owner0->GetBody(), m_owner1->GetBody());
	}

	bool IsActive () const
	{
		return m_isActive;
	}	

	virtual int GetMaxDof() const = 0;

	dVehicleNode* m_owner0;
	dVehicleNode* m_owner1;
	bool m_isActive;
};

#endif 

