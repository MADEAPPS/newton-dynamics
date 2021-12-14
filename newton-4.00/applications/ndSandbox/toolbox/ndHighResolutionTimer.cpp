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

#include "ndSandboxStdafx.h"
#include "ndOpenGlUtil.h"
#include "ndHighResolutionTimer.h"

static ndUnsigned64 m_prevTime = 0;

void dResetTimer()
{
	m_prevTime = dGetTimeInMicroseconds();
}

ndFloat32 dGetElapsedSeconds()
{
	const ndFloat64 TICKS2SEC = 1.0e-6f;
	ndUnsigned64 microseconds = dGetTimeInMicroseconds();

	ndFloat32 timeStep = ndFloat32 ((microseconds - m_prevTime) * TICKS2SEC);
	m_prevTime = microseconds;

	return timeStep;
} 

