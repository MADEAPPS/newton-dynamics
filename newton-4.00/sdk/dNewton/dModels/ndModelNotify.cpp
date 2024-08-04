/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndModelNotify.h"

ndModelNotify::ndModelNotify(const ndModelNotify& src)
	:ndContainersFreeListAlloc<ndModelNotify>()
	,m_model(src.m_model)
{
}

ndModelNotify::ndModelNotify()
	:ndContainersFreeListAlloc<ndModelNotify>()
	,m_model(nullptr)
{
}

ndModelNotify* ndModelNotify::Clone() const
{
	ndAssert(0);
	return nullptr;
}

ndModelNotify::~ndModelNotify()
{
}

#if 0
ndBody* ndModelNotify::GetBody()
{ 
	return m_body; 
}

const ndBody* ndModelNotify::GetBody() const
{ 
	return m_body; 
}

void* ndModelNotify::GetUserData() const
{ 
	return nullptr; 
}

ndVector ndModelNotify::GetGravity() const
{
	return m_defaultGravity;
}

void ndModelNotify::SetGravity(const ndVector & defaultGravity)
{
	m_defaultGravity = defaultGravity;
}

void ndModelNotify::OnTransform(ndInt32, const ndMatrix&)
{
}

bool ndModelNotify::OnSceneAabbOverlap(const ndBody* const) const
{
	return true;
}

void ndModelNotify::OnApplyExternalForce(ndInt32, ndFloat32)
{
	ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
	ndAssert(body);
	if (body->GetInvMass() > 0.0f)
	{
		ndVector massMatrix(body->GetMassMatrix());
		ndVector force (m_defaultGravity.Scale(massMatrix.m_w));
		body->SetForce(force);
		body->SetTorque(ndVector::m_zero);

		//ndVector L(body->CalculateAngularMomentum());
		//dTrace(("%f %f %f\n", L.m_x, L.m_y, L.m_z));
	}
}
#endif
