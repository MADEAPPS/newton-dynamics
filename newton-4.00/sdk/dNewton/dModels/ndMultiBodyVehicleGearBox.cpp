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
#include "ndMultiBodyVehicleGearBox.h"

ndJointVehicleMotorGearBox::ndJointVehicleMotorGearBox(ndBodyKinematic* const motor, ndBodyKinematic* const differential)
	:ndJointGear(dFloat32 (1.0f), motor->GetMatrix().m_front, differential,	motor->GetMatrix().m_front, motor)
{
	SetRatio(dFloat32(4.0f));
	SetSolverModel(m_jointkinematicCloseLoop);
}

void ndJointVehicleMotorGearBox::JacobianDerivative(ndConstraintDescritor& desc)
{
//return;
	//dMatrix matrix0;
	//dMatrix matrix1;
	//CalculateGlobalMatrix(matrix0, matrix1);
	//
	//AddAngularRowJacobian(desc, matrix1.m_right, dFloat32(0.0f));
	//
	//ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	//ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	//
	//jacobian0.m_angular = matrix0.m_front;
	//jacobian1.m_angular = matrix1.m_front;
	//
	//const dVector& omega0 = m_body0->GetOmega();
	//const dVector& omega1 = m_body1->GetOmega();
	//
	//const dVector relOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
	//dFloat32 w = (relOmega.m_x + relOmega.m_y + relOmega.m_z) * dFloat32(0.5f);
	//SetMotorAcceleration(desc, -w * desc.m_invTimestep);

	ndJointGear::JacobianDerivative(desc);
}


