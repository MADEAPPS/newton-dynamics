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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndShapeCapsule.h"
#include "ndBodyPlayerCapsule.h"

ndBodyPlayerCapsule::ndBodyPlayerCapsule(const dMatrix& localAxis, dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight)
	:ndBodyKinematic()
{
	m_impulse = dVector::m_zero;
	m_headingAngle = dFloat32(0.0f);
	m_forwardSpeed = dFloat32(0.0f);
	m_lateralSpeed = dFloat32(0.0f);
	m_stepHeight = dFloat32(0.0f);
	m_contactPatch = dFloat32(0.0f);
	m_height = height;
	m_weistScale = dFloat32(3.0f);
	m_crouchScale = dFloat32(0.5f);
	m_isAirbone = false;
	m_isOnFloor = false;
	m_isCrouched = false;

	dMatrix shapeMatrix(localAxis);
	shapeMatrix.m_posit = shapeMatrix.m_front.Scale(height * 0.5f);
	shapeMatrix.m_posit.m_w = 1.0f;
	
	height = dMax(height - 2.0f * radius / m_weistScale, dFloat32(0.1f));
	ndShapeInstance instance(new ndShapeCapsule(radius / m_weistScale, radius / m_weistScale, height));
	instance.SetLocalMatrix(shapeMatrix);
	instance.SetScale(dVector (dFloat32 (1.0f), m_weistScale, m_weistScale, dFloat32 (0.0f)));
	ndBodyKinematic::SetCollisionShape(instance);
	
	SetMassMatrix(mass, instance);
		
	m_localFrame = shapeMatrix;
	m_localFrame.m_posit = dVector::m_wOne;
	m_contactPatch = radius / m_weistScale;
	m_stepHeight = dMax(stepHeight, m_contactPatch * dFloat32(2.0f));
}

ndBodyPlayerCapsule::~ndBodyPlayerCapsule()
{
}

