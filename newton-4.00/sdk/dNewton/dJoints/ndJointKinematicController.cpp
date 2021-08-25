/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointKinematicController.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointKinematicController)

#if 0
void ndJointKinematicController::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	dAssert (0);
}

void ndJointKinematicController::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dAssert (0);
}

void ndJointKinematicController::ResetAutoSleep ()
{
	ndBodyKinematicSetAutoSleep(GetBody0(), 0);
}

dMatrix ndJointKinematicController::GetBodyMatrix () const
{
	dMatrix matrix0;
	ndBodyKinematicGetMatrix(m_body0, &matrix0[0][0]);
	return m_localMatrix0 * matrix0;
}


void ndJointKinematicController::SubmitConstraints (dFloat32 timestep, dInt32 threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	SubmitLinearConstraints (matrix0, matrix1, timestep);

	if (m_controlMode != m_linear) {
		dVector omega0(0.0f);
		dVector omega1(0.0f);
		dVector veloc0(0.0f);
		dVector veloc1(0.0f);
		dComplementaritySolver::dJacobian jacobian0;
		dComplementaritySolver::dJacobian jacobian1;

		ndBodyKinematicGetOmega(m_body0, &omega0[0]);
		ndBodyKinematicGetVelocity(m_body0, &veloc0[0]);
		if (m_body1) {
			ndBodyKinematicGetOmega(m_body1, &omega1[0]);
			ndBodyKinematicGetVelocity(m_body1, &veloc1[0]);
		}

		const dFloat32 damp = 0.3f;
		const dFloat32 invTimestep = 1.0f / timestep;
		if (m_controlMode == m_linearPlusAngularFriction) {
			dFloat32 omegaMag2 = omega0.DotProduct3(omega0);
			dFloat32 angularFriction = m_maxAngularFriction + m_angularFrictionCoefficient * omegaMag2;
			for (dInt32 i = 0; i < 3; i++) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[i][0]);
				NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

				dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
				dFloat32 relSpeed = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
				dFloat32 relAccel = relSpeed * invTimestep;

				NewtonUserJointSetRowAcceleration(m_joint, -relAccel);
				NewtonUserJointSetRowMinimumFriction(m_joint, -angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, angularFriction);
			}
		} else {

			dFloat32 pitchAngle = 0.0f;
			const dFloat32 maxAngle = 2.0f * m_maxOmega * timestep;
			dFloat32 cosAngle = matrix1[0].DotProduct3(matrix0[0]);
			if (cosAngle > dFloat32 (0.998f)) {
				if ((m_controlMode == m_linearAndCone) || (m_controlMode == m_full6dof)) {
					for (dInt32 i = 1; i < 3; i++) {
						dFloat32 coneAngle = -damp * CalculateAngle(matrix0[0], matrix1[0], matrix1[i]);
						NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[i][0]);
						NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

						dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
						dFloat32 relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;

						dFloat32 w = m_maxOmega * dSign(coneAngle);
						if ((coneAngle < maxAngle) && (coneAngle > -maxAngle)) {
							w = damp * coneAngle * invTimestep;
						}

						dAssert(dAbs(w) <= m_maxOmega);
						dFloat32 relAlpha = (w + relOmega) * invTimestep;

						NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
						NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
						NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
					}
				}
				pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);

			} else {

				dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
				dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
				lateralDir = lateralDir.Normalize();
				dFloat32 coneAngle = dAcos(dClamp(cosAngle, dFloat32(-1.0f), dFloat32(1.0f)));
				dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);

				if ((m_controlMode == m_linearAndCone) || (m_controlMode == m_full6dof)) {
					NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
					NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);

					dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
					dFloat32 relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;

					dFloat32 w = m_maxOmega * dSign(coneAngle);
					if ((coneAngle < maxAngle) && (coneAngle > -maxAngle)) {
						w = damp * coneAngle * invTimestep;
					}
					dFloat32 relAlpha = (w + relOmega) * invTimestep;

					NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
					NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
					NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);


					dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
					NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
					NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
					pointOmega = omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular;
					relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
					relAlpha = relOmega * invTimestep;

					NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
					NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
					NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
				}

				dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
				pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
			}

			if ((m_controlMode == m_linearAndTwist) || (m_controlMode == m_full6dof)) {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
				NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
				dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
				dFloat32 relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;

				dFloat32 w = m_maxOmega * dSign(pitchAngle);
				if ((pitchAngle < maxAngle) && (pitchAngle > -maxAngle)) {
					w = damp * pitchAngle * invTimestep;
				}
				dAssert(dAbs(w) <= m_maxOmega);
				dFloat32 relAlpha = (w + relOmega) * invTimestep;

				NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
			}
		}
	}
}

#endif

ndJointKinematicController::ndJointKinematicController(ndBodyKinematic* const body, ndBodyKinematic* const referenceBody, const dVector& attachmentPointInGlobalSpace)
	:ndJointBilateralConstraint(6, body, referenceBody, dMatrix(attachmentPointInGlobalSpace))
{
	dAssert(GetBody0() == body);
	dMatrix matrix(body->GetMatrix());
	matrix.m_posit = attachmentPointInGlobalSpace;
	matrix.m_posit.m_w = dFloat32(1.0f);

	Init(matrix);
}

ndJointKinematicController::ndJointKinematicController(ndBodyKinematic* const referenceBody, ndBodyKinematic* const body, const dMatrix& attachmentMatrixInGlobalSpace)
	:ndJointBilateralConstraint(6, referenceBody, body, attachmentMatrixInGlobalSpace)
{
	Init(attachmentMatrixInGlobalSpace);
}

ndJointKinematicController::ndJointKinematicController(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	dMatrix attachmentMatrixInGlobalSpace(m_localMatrix0 * GetBody0()->GetMatrix());
	Init(attachmentMatrixInGlobalSpace);

	m_maxSpeed = xmlGetFloat(xmlNode, "maxSpeed");
	m_maxOmega = xmlGetFloat(xmlNode, "maxOmega");
	m_maxLinearFriction = xmlGetFloat(xmlNode, "maxLinearFriction");
	m_maxAngularFriction = xmlGetFloat(xmlNode, "maxAngularFriction");
	m_angularFrictionCoefficient = xmlGetFloat(xmlNode, "angularFrictionCoefficient");
	m_controlMode = ndControlModes(xmlGetInt(xmlNode, "controlMode"));
	m_autoSleepState = xmlGetInt(xmlNode, "autoSleepState") ? true : false;

}

ndJointKinematicController::~ndJointKinematicController()
{
	m_body0->SetAutoSleep(m_autoSleepState);
}

void ndJointKinematicController::Init(const dMatrix& globalMatrix)
{
	CalculateLocalMatrix(globalMatrix, m_localMatrix0, m_localMatrix1);

	m_autoSleepState = m_body0->GetAutoSleep();
	m_body0->SetAutoSleep(false);

	SetControlMode(m_full6dof);
	SetMaxLinearFriction(1.0f);
	SetMaxAngularFriction(1.0f);
	SetAngularViscuosFrictionCoefficient(1.0f);
	SetMaxSpeed(30.0f);
	SetMaxOmega(10.0f * 360.0f * dDegreeToRad);
	
	// set as soft joint
	SetSolverModel(m_jointkinematicAttachment);
}

void ndJointKinematicController::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	const dVector omega0(m_body0->GetOmega());
	const dVector omega1(m_body1->GetOmega());
	const dVector veloc0(m_body0->GetVelocity());
	const dVector veloc1(m_body1->GetVelocity());

	const dFloat32 timestep = desc.m_timestep;
	const dFloat32 invTimestep = desc.m_invTimestep;

	const dFloat32 damp = dFloat32 (0.3f);
	const dFloat32 maxDistance = 2.0f * m_maxSpeed * timestep;
	for (dInt32 i = 0; i < 3; i++) 
	{
		const dInt32 index = desc.m_rowsCount;
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1[i]);
		const ndJacobianPair& jacobianPair = desc.m_jacobian[index];
	
		const dVector pointPosit(
			matrix0.m_posit * jacobianPair.m_jacobianM0.m_linear + 
			matrix1.m_posit * jacobianPair.m_jacobianM1.m_linear);

		const dVector pointVeloc(
			veloc0 * jacobianPair.m_jacobianM0.m_linear + omega0 * jacobianPair.m_jacobianM0.m_angular + 
			veloc1 * jacobianPair.m_jacobianM1.m_linear + omega1 * jacobianPair.m_jacobianM1.m_angular);
	
		const dFloat32 dist = pointPosit.AddHorizontal().GetScalar();
		const dFloat32 speed = pointVeloc.AddHorizontal().GetScalar();
	
		dFloat32 v = m_maxSpeed * dSign(dist);
		if ((dist < maxDistance) && (dist > -maxDistance)) 
		{
			v = damp * dist * invTimestep;
		}
		dAssert(dAbs(v) <= m_maxSpeed);
	
		const dFloat32 relAccel = dClamp ((v + speed) * invTimestep, dFloat32 (-400.0f), dFloat32 (400.0f));

		SetMotorAcceleration(desc, -relAccel);
		SetLowerFriction(desc, -m_maxLinearFriction);
		SetHighFriction(desc, m_maxLinearFriction);
	}

	if (m_controlMode != m_linear) 
	{
		if (m_controlMode == m_linearPlusAngularFriction) 
		{
			const dFloat32 omegaMag2 = omega0.DotProduct(omega0).GetScalar();
			const dFloat32 angularFriction = m_maxAngularFriction + m_angularFrictionCoefficient * omegaMag2;
			for (dInt32 i = 0; i < 3; i++) 
			{
				const dInt32 index = desc.m_rowsCount;
				AddAngularRowJacobian(desc, matrix1[i], dFloat32 (0.0f));
				const ndJacobianPair& jacobianPair = desc.m_jacobian[index];
				
				const dVector pointOmega(omega0 * jacobianPair.m_jacobianM0.m_angular + omega1 * jacobianPair.m_jacobianM1.m_angular);
				const dFloat32 relSpeed = pointOmega.AddHorizontal().GetScalar();
				const dFloat32 relAccel = dClamp (relSpeed * invTimestep, dFloat32 (-3000.0f), dFloat32(3000.0f));
				SetMotorAcceleration(desc, -relAccel);
				SetLowerFriction(desc, -angularFriction);
				SetHighFriction(desc, angularFriction);
			}
		}
		else 
		{
			dAssert(0);
		//	dFloat32 pitchAngle = 0.0f;
		//	const dFloat32 maxAngle = 2.0f * m_maxOmega * timestep;
		//	dFloat32 cosAngle = matrix1[0].DotProduct3(matrix0[0]);
		//	if (cosAngle > dFloat32(0.998f)) {
		//		if ((m_controlMode == m_linearAndCone) || (m_controlMode == m_full6dof)) {
		//			for (dInt32 i = 1; i < 3; i++) {
		//				dFloat32 coneAngle = -damp * CalculateAngle(matrix0[0], matrix1[0], matrix1[i]);
		//				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[i][0]);
		//				NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
		//
		//				dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
		//				dFloat32 relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
		//
		//				dFloat32 w = m_maxOmega * dSign(coneAngle);
		//				if ((coneAngle < maxAngle) && (coneAngle > -maxAngle)) {
		//					w = damp * coneAngle * invTimestep;
		//				}
		//
		//				dAssert(dAbs(w) <= m_maxOmega);
		//				dFloat32 relAlpha = (w + relOmega) * invTimestep;
		//
		//				NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
		//				NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
		//				NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
		//			}
		//		}
		//		pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		//
		//	}
		//	else {
		//
		//		dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
		//		dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
		//		lateralDir = lateralDir.Normalize();
		//		dFloat32 coneAngle = dAcos(dClamp(cosAngle, dFloat32(-1.0f), dFloat32(1.0f)));
		//		dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
		//
		//		if ((m_controlMode == m_linearAndCone) || (m_controlMode == m_full6dof)) {
		//			NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
		//			NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
		//
		//			dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
		//			dFloat32 relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
		//
		//			dFloat32 w = m_maxOmega * dSign(coneAngle);
		//			if ((coneAngle < maxAngle) && (coneAngle > -maxAngle)) {
		//				w = damp * coneAngle * invTimestep;
		//			}
		//			dFloat32 relAlpha = (w + relOmega) * invTimestep;
		//
		//			NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
		//			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
		//			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
		//
		//
		//			dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
		//			NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
		//			NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
		//			pointOmega = omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular;
		//			relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
		//			relAlpha = relOmega * invTimestep;
		//
		//			NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
		//			NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
		//			NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
		//		}
		//
		//		dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
		//		pitchAngle = -dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
		//	}
		//
		//	if ((m_controlMode == m_linearAndTwist) || (m_controlMode == m_full6dof)) {
		//		NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
		//		NewtonUserJointGetRowJacobian(m_joint, &jacobian0.m_linear[0], &jacobian0.m_angular[0], &jacobian1.m_linear[0], &jacobian1.m_angular[0]);
		//		dVector pointOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
		//		dFloat32 relOmega = pointOmega.m_x + pointOmega.m_y + pointOmega.m_z;
		//
		//		dFloat32 w = m_maxOmega * dSign(pitchAngle);
		//		if ((pitchAngle < maxAngle) && (pitchAngle > -maxAngle)) {
		//			w = damp * pitchAngle * invTimestep;
		//		}
		//		dAssert(dAbs(w) <= m_maxOmega);
		//		dFloat32 relAlpha = (w + relOmega) * invTimestep;
		//
		//		NewtonUserJointSetRowAcceleration(m_joint, -relAlpha);
		//		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxAngularFriction);
		//		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxAngularFriction);
		//	}
		}
	}
}

void ndJointKinematicController::CheckSleep() const
{
	GetBody0()->SetSleepState(false);

	//dAssert(0);
	//dMatrix matrix0;
	//dMatrix matrix1;
	//CalculateGlobalMatrix(matrix0, matrix1);
	//
	//dMatrix matrix(matrix1 * matrix0.Inverse());
	//matrix.m_posit.m_w = 0.0f;
	//if (matrix.m_posit.DotProduct3(matrix.m_posit) > dFloat32(1.0e-6f)) 
	//{
	//	ndBodyKinematicSetSleepState(m_body0, 0);
	//}
	//else 
	//{
	//	switch (m_controlMode)
	//	{
	//		case m_full6dof:
	//		case m_linearPlusAngularFriction:
	//		{
	//			dFloat32 trace = matrix[0][0] * matrix[1][1] * matrix[2][2];
	//			if (trace < 0.9995f) 
	//			{
	//				ndBodyKinematicSetSleepState(m_body0, 0);
	//			}
	//			break;
	//		}
	//
	//		case m_linearAndCone:
	//		{
	//			dFloat32 cosAngle = matrix1[0].DotProduct3(matrix0[0]);
	//			if (cosAngle > 0.998f) 
	//			{
	//				ndBodyKinematicSetSleepState(m_body0, 0);
	//			}
	//			break;
	//		}
	//
	//		case m_linearAndTwist:
	//		{
	//			dFloat32 pitchAngle = 0.0f;
	//			dFloat32 cosAngle = matrix1[0].DotProduct3(matrix0[0]);
	//			if (cosAngle > 0.998f) 
	//			{
	//				pitchAngle = CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
	//			}
	//			else 
	//			{
	//				dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
	//				dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
	//				lateralDir = lateralDir.Normalize();
	//				dFloat32 coneAngle = dAcos(dClamp(cosAngle, dFloat32(-1.0f), dFloat32(1.0f)));
	//				dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
	//				dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
	//				pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	//			}
	//
	//			if (dAbs(pitchAngle) > (1.0 * dDegreeToRad)) 
	//			{
	//				ndBodyKinematicSetSleepState(m_body0, 0);
	//			}
	//			break;
	//		}
	//	}
	//}
}

void ndJointKinematicController::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "maxSpeed", m_maxSpeed);
	xmlSaveParam(childNode, "maxOmega", m_maxOmega);
	xmlSaveParam(childNode, "maxLinearFriction", m_maxLinearFriction);
	xmlSaveParam(childNode, "maxAngularFriction", m_maxAngularFriction);
	xmlSaveParam(childNode, "angularFrictionCoefficient", m_angularFrictionCoefficient);
	xmlSaveParam(childNode, "controlMode", dInt32(m_controlMode));
	xmlSaveParam(childNode, "autoSleepState", dInt32(m_autoSleepState));
}
