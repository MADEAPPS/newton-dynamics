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


#ifndef __D_VEHICLE_NODE_H__
#define __D_VEHICLE_NODE_H__

#include "dStdafxVehicle.h"

class dVehicleNode;
class dVehicleChassis;

class dVehicleNodeChildrenList: public dList<dVehicleNode*>
{
	public:
	dVehicleNodeChildrenList()
		:dList<dVehicleNode*>()
	{
	}

	~dVehicleNodeChildrenList()
	{
	}
};


class dVehicleNode: public dCustomAlloc
{
	public:
	DVEHICLE_API dVehicleNode(dVehicleNode* const parent);
	DVEHICLE_API virtual ~dVehicleNode();
	
	//DVEHICLE_API virtual void CalculateNodeAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const;
	virtual dVehicleChassis* GetAsVehicle() const { return NULL; }
	//virtual dVehicleTireInterface* GetAsTire() const { return NULL; }
		
	void* GetUserData() const {return m_usedData;}
	void SetUserData(void* const userData) {m_usedData = userData;}

	dComplementaritySolver::dBodyState& GetProxyBody() { return m_proxyBody; }

	protected:
	//virtual void RigidBodyToStates();
	virtual void RigidBodyToProxyBody();
	//virtual void Integrate(dFloat timestep);
	//virtual void StatesToRigidBody(dFloat timestep);
	//void CalculateAABB(const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP) const;
	//friend class dVehicleSolver;

	
	virtual const void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	dComplementaritySolver::dBodyState m_proxyBody;
	void* m_usedData;
	dVehicleNode* m_parent;
	dVehicleNodeChildrenList m_children;

	friend class dVehicleManager;
	friend class dVehicleChassis;
};


#endif 

