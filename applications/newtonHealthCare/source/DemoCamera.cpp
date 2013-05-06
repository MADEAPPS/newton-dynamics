/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/



// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include <toolbox_stdafx.h>
#include "DemoCamera.h"
#include "MousePick.h"
#include "NewtonDemos.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MOUSE_PICK_DAMP			 10.0f
#define MOUSE_PICK_STIFFNESS	 100.0f


dFloat DemoCamera::m_pickedBodyParam;
dVector DemoCamera::m_pickedBodyDisplacement;
dVector DemoCamera::m_pickedBodyLocalAtachmentPoint;
dVector DemoCamera::m_pickedBodyLocalAtachmentNormal;
NewtonApplyForceAndTorque DemoCamera::m_chainPickBodyForceCallback; 




DemoCamera::DemoCamera()
	:DemoEntity (GetIdentityMatrix(), NULL) 
	,m_yawRate (0.01f)
	,m_cameraYaw(0.0f)
	,m_cameraPitch(0.0f)
	,m_fov (60.0f * 3.1416f / 180.0f)
	,m_frontPlane (0.1f)
	,m_backPlane(2000.0f)
	,m_pitchRate (0.01f)
	,m_sidewaysSpeed(10.0f)
	,m_cameraFrontSpeed(15.0f)
	,m_mousePosX(0)
	,m_mousePosY(0)
	,m_prevMouseState(false)
	,m_targetPicked(NULL)
{
}

	
DemoCamera::~DemoCamera()
{
}

void DemoCamera::Render(dFloat timeStep) const
{
}


void DemoCamera::PhysicsApplyPickForce (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dVector com;
	dVector veloc;
	dVector omega;
	dMatrix matrix;

	// apply the thew body forces
	if (m_chainPickBodyForceCallback) {
		m_chainPickBodyForceCallback (body, timestep, threadIndex);
	}

	// add the mouse pick penalty force and torque
	NewtonBodyGetVelocity(body, &veloc[0]);

	NewtonBodyGetOmega(body, &omega[0]);
	NewtonBodyGetVelocity(body, &veloc[0]);
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);


	dVector force (m_pickedBodyDisplacement.Scale (mass * MOUSE_PICK_STIFFNESS));
	dVector dampForce (veloc.Scale (MOUSE_PICK_DAMP * mass));
	force -= dampForce;

	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonBodyGetCentreOfMass (body, &com[0]);

	// calculate local point relative to center of mass
	dVector point (matrix.RotateVector (m_pickedBodyLocalAtachmentPoint - com));
	dVector torque (point * force);

	dVector torqueDamp (omega.Scale (mass * 0.1f));

	NewtonBodyAddForce (body, &force.m_x);
	NewtonBodyAddTorque (body, &torque.m_x);

	// make sure the body is unfrozen, if it is picked
	NewtonBodySetFreezeState (body, 0);
}



void DemoCamera::SetViewMatrix(int width, int height)
{
	// set the view port for this render section
	glViewport(0, 0, (GLint) width, (GLint) height);

	// set the projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

//m_backPlane = 10000.0f;
	gluPerspective(m_fov * 180.0f / 3.1416f, GLfloat (width) /GLfloat(height), m_frontPlane, m_backPlane);

	// set the model view matrix 
//	glMatrixMode(GL_MODELVIEW_MATRIX);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	dVector pointOfInterest (m_matrix.m_posit + m_matrix.m_front);
	gluLookAt(m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z, 
		      pointOfInterest.m_x, pointOfInterest.m_y, pointOfInterest.m_z, 
			  m_matrix.m_up.m_x, m_matrix.m_up.m_y, m_matrix.m_up.m_z);	

//float xxx[16];
//glGetFloatv(GL_MODELVIEW_MATRIX, xxx);
//glGetFloatv(GL_MODELVIEW, xxx);
}

void DemoCamera::SetMatrix (DemoEntityManager& world, const dQuaternion& rotation, const dVector& position)
{
	dMatrix matrix (rotation, position);
	m_cameraPitch = dAsin (matrix.m_front.m_y);
	m_cameraYaw = dAtan2 (-matrix.m_front.m_z, matrix.m_front.m_x);

	DemoEntity::SetMatrix (world, rotation, position);
}


void DemoCamera::RenderPickedTarget () const
{
	if (m_targetPicked) {
		dMatrix matrix;
		NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

		dVector p0 (matrix.TransformVector(m_pickedBodyLocalAtachmentPoint));
		dVector p1 (p0 + matrix.RotateVector (m_pickedBodyLocalAtachmentNormal.Scale (0.5f)));
		ShowMousePicking (p0, p1);
	}
}


void DemoCamera::UpdateInputs (float timestep, const NewtonDemos* const mainWin, DemoEntityManager& world) 
{
//	m_cannonBallRate -= int (timestep * 1000000.0f);

	dMatrix targetMatrix (GetNextMatrix());

	int mouseX;
	int mouseY;
	mainWin->GetMousePosition (mouseX, mouseY);

	// slow down the Camera if we have a Body
	float slowDownFactor = 1.0f;
	if (m_targetPicked) {
		//slowDownFactor *= 0.125f;
	}

	// do camera translation
	if (mainWin->GetKeyState ('W')) {
		targetMatrix.m_posit += targetMatrix.m_front.Scale(m_cameraFrontSpeed * timestep * slowDownFactor);
	}
	if (mainWin->GetKeyState ('S')) {
		targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_cameraFrontSpeed * timestep * slowDownFactor);
	}
	if (mainWin->GetKeyState ('A')) {
		targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}
	if (mainWin->GetKeyState ('D')) {
		targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (mainWin->GetKeyState ('Q')) {
		targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (mainWin->GetKeyState ('E')) {
		targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	// do camera rotation, only if we do not have anything picked
	if (!m_targetPicked && mainWin->GetMouseKeyState(0)) {
		int mouseSpeedX = mouseX - m_mousePosX;
		int mouseSpeedY = mouseY - m_mousePosY;
		if (mouseSpeedX > 0) {
			m_cameraYaw = dMod(m_cameraYaw + m_yawRate, 2.0f * 3.1416f);
		} else if (mouseSpeedX < 0){
			m_cameraYaw = dMod(m_cameraYaw - m_yawRate, 2.0f * 3.1416f);
		}

		if (mouseSpeedY > 0) {
			m_cameraPitch += m_pitchRate;
		} else if (mouseSpeedY < 0){
			m_cameraPitch -= m_pitchRate;
		}
		if (m_cameraPitch > 80.0f * 3.1416f / 180.0f) {
			m_cameraPitch = 80.0f * 3.1416f / 180.0f;
		} else if (m_cameraPitch < -80.0f * 3.1416f / 180.0f) {
			m_cameraPitch = -80.0f * 3.1416f / 180.0f;
		}
	}


	// handle pick body from the screen
	if (!m_targetPicked) {
		if (!m_prevMouseState && mainWin->GetMouseKeyState(0)) {
			dFloat param;
			dVector posit;
			dVector normal;
			NewtonBody* const body = MousePickByForce (world.GetNewton(), mouseX, mouseY,  param, posit, normal);
			if (body) {
				m_targetPicked = body;

				dMatrix matrix;
				NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);

				// save point local to the body matrix
				m_pickedBodyParam = param;
				m_pickedBodyDisplacement = dVector (0.0f, 0.0f, 0.0f, 0.0f);
				m_pickedBodyLocalAtachmentPoint = matrix.UntransformVector (posit);

				// convert normal to local space
				m_pickedBodyLocalAtachmentNormal = matrix.UnrotateVector(normal);

				// link th force and torque callback
				m_chainPickBodyForceCallback = NewtonBodyGetForceAndTorqueCallback (m_targetPicked);
				_ASSERTE (m_chainPickBodyForceCallback != PhysicsApplyPickForce);

				// set a new call back
				NewtonBodySetForceAndTorqueCallback (m_targetPicked, PhysicsApplyPickForce);
			}
		}
	} else {
		if (mainWin->GetMouseKeyState(0)) {
			dMatrix matrix;
			NewtonBodyGetMatrix(m_targetPicked, &matrix[0][0]);
			dVector p2 (matrix.TransformVector (m_pickedBodyLocalAtachmentPoint));

			float x = dFloat (mouseX);
			float y = dFloat (mouseY);
			dVector p0 (ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));
			dVector p (p0 + (p1 - p0).Scale (m_pickedBodyParam));
			m_pickedBodyDisplacement = p - p2;
			dFloat mag2 = m_pickedBodyDisplacement % m_pickedBodyDisplacement;
			if (mag2 > dFloat (20 * 20)) {
				m_pickedBodyDisplacement = m_pickedBodyDisplacement.Scale (20.0f / dSqrt (m_pickedBodyDisplacement % m_pickedBodyDisplacement));
			}
		} else {
			// unchain the call back
			NewtonBodySetForceAndTorqueCallback (m_targetPicked, m_chainPickBodyForceCallback);
			m_targetPicked = NULL;
		}
	}


	m_mousePosX = mouseX;
	m_mousePosY = mouseY;
	m_prevMouseState = mainWin->GetMouseKeyState(0);

//	case _shotCannonBall:
//	{
//		if (m_cannonBallRate <= 0) {
//			m_cannonBallRate = int (0.25f * 1000000.0f);
//
//			if (!m_meshBallMesh) {
//				NewtonCollision* const ball = NewtonCreateSphere (m_world, 0.25f, 0.25f, 0.25f, 0, NULL);
//				m_meshBallMesh = new DemoMesh("ball", ball, "base_road.tga", "base_road.tga", "base_road.tga");
//				NewtonReleaseCollision(m_world, ball);
//			}
//
//			dMatrix matrix (GetIdentityMatrix());
//			matrix.m_posit = targetMatrix.m_posit;
//
//			dVector veloc (targetMatrix.m_front.Scale (PROJECTILE_INITIAL_SPEED));
//			NewtonCollision* const ballCollision = NewtonCreateSphere (m_world, 0.25f, 0.25f, 0.25f, 0, NULL);
//			NewtonBody* const body = CreateSimpleSolid (this, m_meshBallMesh, 100.0f, matrix, ballCollision, 0);
//			NewtonReleaseCollision(m_world, ballCollision);
//			NewtonBodySetVelocity(body, &veloc[0]);
//		}
//	}

	
	dMatrix matrix (dRollMatrix(m_cameraPitch) * dYawMatrix(m_cameraYaw));
	dQuaternion rot (matrix);
	DemoEntity::SetMatrix (world, rot, targetMatrix.m_posit);
}



ThirdPersonCamera::ThirdPersonCamera(DemoEntity* const targetEntity)
:DemoCamera()
,m_camYaw (0.0f)
,m_camPitch(-30.0f * 3.1416f / 180.0f)
,m_camDist (10.0f)
,m_targetEntity (targetEntity)
,m_toggleFreeCamera (false)
{
}

void ThirdPersonCamera::UpdateInputs (float timestep, const NewtonDemos* const mainWin, DemoEntityManager& world) 
{
	if (m_toggleFreeCamera) {
		//free camera mode 
		DemoCamera::UpdateInputs (timestep, mainWin, world);
	} else {
		// third person view mode
		_ASSERTE (m_targetEntity);
		dMatrix targetMatrix (m_targetEntity->GetNextMatrix());

		targetMatrix.m_right = targetMatrix.m_front * dVector (0.0f, 1.0f, 0.0f, 0.0f);
		targetMatrix.m_right = targetMatrix.m_right.Scale (1.0f / dSqrt (targetMatrix.m_right % targetMatrix.m_right));
		targetMatrix.m_up = targetMatrix.m_right * targetMatrix.m_front;

		dVector offset (-m_camDist, 0.0f, 0.0f, 1.0f);
		dMatrix matrix (dRollMatrix(m_camPitch) * dYawMatrix(m_camYaw));
		offset = matrix.RotateVector(offset);

		dMatrix camMatrix;
		camMatrix.m_front = offset.Scale (-1.0f / dSqrt(offset % offset));
		camMatrix.m_right = camMatrix.m_front * dVector (0.0f, 1.0f, 0.0f, 0.0f);
		camMatrix.m_right = camMatrix.m_right.Scale (1.0f / dSqrt (camMatrix.m_right % camMatrix.m_right));
		camMatrix.m_up = camMatrix.m_right * camMatrix.m_front;
		camMatrix.m_posit = offset;
		camMatrix.m_front.m_w = 0.0f;
		camMatrix.m_up.m_w = 0.0f;
		camMatrix.m_right.m_w = 0.0f;
		camMatrix.m_posit.m_w = 1.0f;

		camMatrix = camMatrix * targetMatrix;


		// do camera translation
		if (mainWin->GetKeyState ('W')) {
			m_camPitch -= 0.005f;
			if (m_camPitch < (-80.0f * 3.1416f / 180.0f) ) {
				m_camPitch = -80.0f * 3.1416f / 180.0f;
			}
		}
		if (mainWin->GetKeyState ('S')) {
			m_camPitch += 0.005f;
			if (m_camPitch > (-10.0f * 3.1416f / 180.0f) ) {
				m_camPitch = -10.0f * 3.1416f / 180.0f;
			}
		}

		if (mainWin->GetKeyState ('D')) {
			m_camYaw = dMod(m_camYaw + 0.005f, 2.0f * 3.1416f);				
		}

		if (mainWin->GetKeyState ('A')) {
			m_camYaw = dMod(m_camYaw - 0.005f, 2.0f * 3.1416f);				
		}

		//if (mainWin->GetKeyState ('E')) {
		//	targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
		//}
		dQuaternion rot (camMatrix);
		DemoEntity::SetMatrix (world, rot, camMatrix.m_posit);
	}
}
