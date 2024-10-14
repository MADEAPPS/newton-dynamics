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

#ifndef __D_WORLD_DYNAMICS_UPDATE_CUDA_H__
#define __D_WORLD_DYNAMICS_UPDATE_CUDA_H__

#include "ndNewtonStdafx.h"
#include "ndDynamicsUpdate.h"

class ndCudaContext;

class ndDynamicsUpdateCuda : public ndDynamicsUpdate
{
	public:
	ndDynamicsUpdateCuda(ndWorld* const world);
	virtual ~ndDynamicsUpdateCuda();

	virtual const char* GetStringId() const;

	protected:
	virtual void Update();

	public:
	//void SortJoints();
	//void SortIslands();
	//void BuildIsland();
	//void InitWeights();
	//void InitBodyArray();
	//void InitSkeletons();
	//void CalculateForces();
	//void IntegrateBodies();
	//void UpdateSkeletons();
	//void FinishGpuUpdate();
	//void InitJacobianMatrix();
	//void UpdateForceFeedback();
	//void CalculateJointsForce();
	//void IntegrateBodiesVelocity();
	//void CalculateJointsAcceleration();
	//void IntegrateUnconstrainedBodies();
	//
	//void DeviceUpdate();
	//void DetermineSleepStates();
	//void GetJacobianDerivatives(ndConstraint* const joint);

	ndCudaContext* m_context;
};



#endif

