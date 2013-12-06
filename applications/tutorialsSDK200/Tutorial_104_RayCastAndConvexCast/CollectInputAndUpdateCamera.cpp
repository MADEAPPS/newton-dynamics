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

#include "StdAfx.h"
#include "dMatrix.h"
#include "MousePick.h"
#include "OpenGlUtil.h"
#include "TutorialCode.h"

#include "CollectInputAndUpdateCamera.h"


#define CAMERA_SPEED	4.0f

static dFloat gYawAngle = 0.0f * 3.1416 / 180.0f;
static dFloat gPrevYawAngle = 0.0f * 3.1416 / 180.0f;

static dFloat gRollAngle = 0.0f;
static dFloat gPrevRollAngle = 0.0f;

static dVector gCurrCameraDir (1.0f, 0.0f, 0.0f, 0.0f);
static dVector gCameraEyepoint (-15.0f, 5.0f, 0.0f, 0.0f);
static dVector gPrevCameraEyepoint (gCameraEyepoint);


#pragma warning (disable: 4100) //unreferenced formal parameter

void InitCamera (const dVector& eyePoint, const dVector& dir)
{
	gCameraEyepoint = eyePoint;

	gCurrCameraDir = dir.Scale (1.0f / sqrt (dir % dir));
	gRollAngle = dAsin (gCurrCameraDir.m_y);
	gPrevRollAngle = gYawAngle;

	gYawAngle = dAtan2 (-gCurrCameraDir.m_z, gCurrCameraDir.m_x);
	gPrevYawAngle = gYawAngle;
}


// this function update the interpolated position of the Camera
void UpdateCamera (dFloat interpolationParam)
{
	dFloat rollAngle;
	dFloat yawAngle;

	yawAngle = gPrevYawAngle + (gYawAngle - gPrevYawAngle) * interpolationParam;
	rollAngle = gPrevRollAngle + (gRollAngle - gPrevRollAngle) * interpolationParam;
	dMatrix cameraDirMat (dRollMatrix(rollAngle) * dYawMatrix(yawAngle));

	dVector eye (gPrevCameraEyepoint + (gCameraEyepoint - gPrevCameraEyepoint).Scale (interpolationParam));
	dVector target (eye + cameraDirMat.m_front);

	// set up the view orientation looking at the origin from slightly above
	SetCamera (eye, target);
}





//	This function read KeyBoard and Mouse control to control this scene
//  The control are read at the simulation rate, and two states are kept 
void InputControl(NewtonWorld* world)
{
	// read the mouse position and set the camera direction
	
	static dVector mouse0 (GetMousePos());
	dVector mouse1 (GetMousePos());

	gPrevYawAngle = gYawAngle;
	gPrevRollAngle = gRollAngle;

	if (!MousePick (world, mouse1)) {
		int leftKetDown;

		leftKetDown = IsKeyDown (KeyCode_L_BUTTON);
		// we are not in mouse pick mode, then we are in camera tracking mode
		if (leftKetDown) {
			// when click left mouse button the first time, we reset the camera
			// convert the mouse x position to delta yaw angle
			if (mouse1.m_x > (mouse0.m_x + 1)) {
				gYawAngle += 1.0f * 3.1416f / 180.0f;
				if (gYawAngle > (360.0f * 3.1416f / 180.0f)) {
					gYawAngle -= (360.0f * 3.1416f / 180.0f);
				}
			} else if (mouse1.m_x < (mouse0.m_x - 1)) {
				gYawAngle -= 1.0f * 3.1416f / 180.0f;
				if (gYawAngle < 0.0f) {
					gYawAngle += (360.0f * 3.1416f / 180.0f);
				}
			}

			if (mouse1.m_y > (mouse0.m_y + 1)) {
				gRollAngle += 1.0f * 3.1416f / 180.0f;
				if (gRollAngle > (80.0f * 3.1416f / 180.0f)) {
					gRollAngle = 80.0f * 3.1416f / 180.0f;
				}
			} else if (mouse1.m_y < (mouse0.m_y - 1)) {
				gRollAngle -= 1.0f * 3.1416f / 180.0f;
				if (gRollAngle < -(80.0f * 3.1416f / 180.0f)) {
					gRollAngle = -80.0f * 3.1416f / 180.0f;
				}
			}

			dMatrix cameraDirMat (dRollMatrix(gRollAngle) * dYawMatrix(gYawAngle));
			gCurrCameraDir = cameraDirMat.m_front;
		}
	}

	// save mouse position and left mouse key state for next frame
	mouse0 = mouse1;


	// camera control
	gPrevCameraEyepoint = gCameraEyepoint;
	if (IsKeyDown ('W')) {
		gCameraEyepoint += gCurrCameraDir.Scale (CAMERA_SPEED / 60.0f);
	} else if (IsKeyDown  ('S')) {
		gCameraEyepoint -= gCurrCameraDir.Scale (CAMERA_SPEED / 60.0f);
	}

	if (IsKeyDown ('D')) {
		dVector up (0.0f, 1.0f, 0.0f);
		dVector right (gCurrCameraDir * up);
		gCameraEyepoint += right.Scale (CAMERA_SPEED / 60.0f);
	} else if (IsKeyDown ('A')) {
		dVector up (0.0f, 1.0f, 0.0f);
		dVector right (gCurrCameraDir * up);
		gCameraEyepoint -= right.Scale (CAMERA_SPEED / 60.0f);
	}
} 

