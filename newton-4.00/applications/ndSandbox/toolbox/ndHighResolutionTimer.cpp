/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndHighResolutionTimer.h"

dFloat32 dGetElapsedSeconds()
{
	dFloat32 timeStep;
	dUnsigned64 miliseconds;
	const dFloat64 TICKS2SEC = 1.0e-6f;
	static dUnsigned64 m_prevTime = dGetTimeInMicrosenconds();

	miliseconds = dGetTimeInMicrosenconds();

	// optimal keep the fps below 120 fps
	timeStep = dFloat32 ((miliseconds - m_prevTime) * TICKS2SEC);
	m_prevTime = miliseconds;

	return timeStep;
} 

