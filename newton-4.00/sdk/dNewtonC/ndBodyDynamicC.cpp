/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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


#include <ndNewton.h>
#include "ndBodyDynamicC.h"

class ndBodyNotiFyC: public ndBodyNotify
{
	public:
	ndBodyNotiFyC(void* const usedData, ndForceAndTorque forceAndTorque, ndSetTransform transform)
		:ndBodyNotify()
		,m_usedData(usedData)
		,m_transform(transform)
		,m_forceAndTorque(forceAndTorque)
	{
	}

	virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep)
	{
		if (m_forceAndTorque)
		{
			m_forceAndTorque((ndBodyDynamicC)GetBody(), timestep);
		}
	}

	virtual void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
	{
		if (m_transform)
		{
			m_transform((ndBodyDynamicC)GetBody(), &matrix[0][0]);
		}
	}

	void* m_usedData;
	ndSetTransform m_transform;
	ndForceAndTorque m_forceAndTorque;
};

ndBodyDynamicC ndCreateBodyDynamic()
{
	return (ndBodyDynamicC)new ndBodyKinematic();
}

void ndDestroyCreateBodyDynamic(ndBodyDynamicC bodyC)
{
	ndBodyDynamic* const body = (ndBodyDynamic*)bodyC;
	delete body;
}

void ndBodyDynamicSetCallbacks(ndBodyDynamicC bodyc, void* const usedData, ndForceAndTorque forceAndTorque, ndSetTransform transform)
{
	ndBodyDynamic* const body = (ndBodyDynamic*)bodyc;
	body->SetNotifyCallback(new ndBodyNotiFyC(usedData, forceAndTorque, transform));
}

void ndBodyDynamicSetMatrix(ndBodyDynamicC bodyc, dFloat32* const matrixPtr)
{
	ndBodyDynamic* const body = (ndBodyDynamic*)bodyc;
	body->SetMatrix(dMatrix(matrixPtr));
}