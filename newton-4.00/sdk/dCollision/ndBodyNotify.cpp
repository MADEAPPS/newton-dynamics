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
#include "ndCollisionStdafx.h"
#include "ndBodyNotify.h"
#include "ndBodyKinematic.h"


ndBodyNotify::ndBodyNotify(const ndBodyNotify& src)
	:ndContainersFreeListAlloc<ndBodyNotify>()
	,m_defaultGravity(src.m_defaultGravity)
	,m_body(src.m_body)
{
}

ndBodyNotify::ndBodyNotify(const ndVector& defaultGravity)
	:ndContainersFreeListAlloc<ndBodyNotify>()
	,m_defaultGravity(defaultGravity)
	,m_body(nullptr)
{
}

ndBodyNotify::~ndBodyNotify()
{
}

ndBodyNotify* ndBodyNotify::Clone() const
{
	return new ndBodyNotify(*this);
}

ndBody* ndBodyNotify::GetBody()
{ 
	return m_body; 
}

const ndBody* ndBodyNotify::GetBody() const
{ 
	return m_body; 
}

ndBodyNotify* ndBodyNotify::GetAsBodyNotify()
{
	return this;
}

ndUrdfBodyNotify* ndBodyNotify::GetAsUrdfBodyNotify()
{
	return nullptr;
}

ndVector ndBodyNotify::GetGravity() const
{
	return m_defaultGravity;
}

void ndBodyNotify::SetGravity(const ndVector & defaultGravity)
{
	m_defaultGravity = defaultGravity;
}

void ndBodyNotify::OnTransform(ndInt32, const ndMatrix&)
{
}

bool ndBodyNotify::OnSceneAabbOverlap(const ndBody* const) const
{
	return true;
}

void ndBodyNotify::OnApplyExternalForce(ndInt32, ndFloat32)
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

