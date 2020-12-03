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
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndJointHinge.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"

ndMultiBodyVehicle::ndMultiBodyVehicle(const dVector& frontDir, const dVector& upDir)
	:ndModel()
	,m_localFrame(dGetIdentityMatrix())
	,m_chassis(nullptr)
{
	m_localFrame.m_front = frontDir & dVector::m_triplexMask;
	m_localFrame.m_up = upDir & dVector::m_triplexMask;
	m_localFrame.m_right = m_localFrame.m_front.CrossProduct(m_localFrame.m_up).Normalize();
	m_localFrame.m_up = m_localFrame.m_right.CrossProduct(m_localFrame.m_front).Normalize();
}

ndMultiBodyVehicle::ndMultiBodyVehicle(const nd::TiXmlNode* const xmlNode)
	:ndModel(xmlNode)
{
}

ndMultiBodyVehicle::~ndMultiBodyVehicle()
{
}

void ndMultiBodyVehicle::AddChassis(ndBodyDynamic* const chassis)
{
	m_chassis = chassis;
}

void ndMultiBodyVehicle::AddTire(ndWorld* const world, ndBodyDynamic* const tire)
{
	dMatrix tireFrame(dGetIdentityMatrix());
	tireFrame.m_front = dVector(1.0f, 0.0f, 0.0f, 0.0f);
	tireFrame.m_up = dVector(0.0f, 1.0f, 0.0f, 0.0f);
	tireFrame.m_right = tireFrame.m_front.CrossProduct(tireFrame.m_up);
	dMatrix matrix (tireFrame * tire->GetMatrix());

	//ndJointBallAndSocket* const joint = new ndJointBallAndSocket(matrix, chassis, tireBody);
	ndJointHinge* const joint = new ndJointHinge(matrix, tire, m_chassis);
	world->AddJoint(joint);
}

void ndMultiBodyVehicle::Update(const ndWorld* const world, dFloat32 timestep) const
{

}
