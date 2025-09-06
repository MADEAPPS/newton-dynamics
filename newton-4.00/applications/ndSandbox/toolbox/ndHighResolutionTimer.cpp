/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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

static ndUnsigned64 m_prevTime = 0;

void ndResetTimer()
{
	m_prevTime = ndGetTimeInMicroseconds();
}

ndFloat32 ndGetElapsedSeconds()
{
	const ndFloat64 TICKS2SEC = ndFloat32(1.0e-6f);
	ndUnsigned64 microseconds = ndGetTimeInMicroseconds();

	ndFloat32 timeStep = ndFloat32 ((ndFloat64)(microseconds - m_prevTime) * TICKS2SEC);
	m_prevTime = microseconds;

	return timeStep;
} 

