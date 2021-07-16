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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndScene.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndConvexCastNotify.h"

bool ndConvexCastNotify::CastShape(const ndShapeInstance& castingInstance, const dMatrix& globalOrigin, const dVector& globalDest, const ndShapeInstance& targetShape, const dMatrix& targetMatrix)
{
	ndContact contact;
	ndBodyKinematic body0;
	ndBodyKinematic body1;
	ndContactNotify notify;
	dFixSizeArray<ndContactPoint, D_MAX_CONTATCS> contactBuffer;
	contactBuffer.SetCount(D_MAX_CONTATCS);

	body0.SetCollisionShape(castingInstance);
	body1.SetCollisionShape(targetShape);
	body0.SetMatrix(globalOrigin);
	body1.SetMatrix(targetMatrix);
	body0.SetMassMatrix(dVector::m_one);
	body0.SetVelocity(globalDest - globalOrigin.m_posit);

	contact.SetBodies(&body0, &body1);

	ndShapeInstance& shape0 = body0.GetCollisionShape();
	ndShapeInstance& shape1 = body1.GetCollisionShape();
	shape0.SetGlobalMatrix(shape0.GetLocalMatrix() * body0.GetMatrix());
	shape1.SetGlobalMatrix(shape1.GetLocalMatrix() * body1.GetMatrix());

	m_contacts.SetCount(0);
	ndContactSolver contactSolver(&contact, &notify, dFloat32(1.0f));
	contactSolver.m_contactBuffer = &contactBuffer[0];

	m_param = dFloat32(1.2f);
	const dInt32 count = dMin(contactSolver.CalculateContactsContinue(), m_contacts.GetCapacity());
	if (count)
	{
		for (dInt32 i = 0; i < count; i++)
		{
			m_contacts.PushBack(contactBuffer[i]);
		}
		m_param = contactSolver.m_timestep;
		m_normal = contactSolver.m_separatingVector;
		m_closestPoint0 = contactSolver.m_closestPoint0;
		m_closestPoint1 = contactSolver.m_closestPoint1;
	}
	return count > 0;
}
