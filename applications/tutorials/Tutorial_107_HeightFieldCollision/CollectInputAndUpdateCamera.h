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

#ifndef __COLLECT_INPUTS_AND_UPDATE_CAMERA__H
#define __COLLECT_INPUTS_AND_UPDATE_CAMERA__H

struct NewtonWorld;
void InputControl (NewtonWorld* world);
void UpdateCamera (float inerpolation);
dVector GetCameraEyePoint ();
void InitCamera (const dVector& eyePoint, const dVector& dir);

#endif
