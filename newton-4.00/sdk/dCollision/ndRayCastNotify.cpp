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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndRayCastNotify.h"
#include "ndBodyKinematic.h"

bool ndRayCastNotify::TraceShape(const ndVector& globalOrigin, const ndVector& globalDestination, const ndShapeInstance& shapeInstance, const ndMatrix& shapeGlobal)
{
	ndContactPoint contactOut;

	ndBodyKinematic tmpBody;
	tmpBody.SetMatrix(globalOrigin);
	tmpBody.SetCollisionShape(shapeInstance);
	tmpBody.SetMassMatrix(ndVector::m_one);

	const ndVector& localOrigin(shapeGlobal.UntransformVector(globalOrigin) & ndVector::m_triplexMask);
	const ndVector& localDestination(shapeGlobal.UntransformVector(globalDestination) & ndVector::m_triplexMask);
	ndFloat32 t = shapeInstance.RayCast(*this, localOrigin, localDestination, &tmpBody, contactOut);
	bool state = false;
	if (t <= ndFloat32 (1.0f))
	{
		ndVector p(shapeGlobal.TransformVector(localOrigin + (localDestination - localOrigin).Scale(t)));
		dAssert(t >= ndFloat32(0.0f));
		dAssert(t <= ndFloat32(1.0f));
		m_param = t;
		state = true;
		m_contact.m_body0 = nullptr;
		m_contact.m_body1 = nullptr;
		m_contact.m_point = p;
		m_contact.m_normal = shapeGlobal.RotateVector(contactOut.m_normal);
	}
	return state;
}



