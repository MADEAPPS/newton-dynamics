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

#include "toolbox_stdafx.h"
#include "DemoCamera.h"
#include "DemoEntityManager.h"
#include "DemoVisualDebugerListener.h"

#if 0

#define USE_VISUAL_DEBUGGER

DemoVisualDebugerListener::DemoVisualDebugerListener(DemoEntityManager* const scene)
	:DemoListenerBase (scene, "vidualFebuger")
{
#ifdef USE_VISUAL_DEBUGGER 
	// create a visual debugger server to visualize this physics world
	m_visualDebugger = NewtonDebuggerCreateServer (scene->GetNewton());
#endif

}

DemoVisualDebugerListener::~DemoVisualDebugerListener()
{
#ifdef USE_VISUAL_DEBUGGER 
	if (m_visualDebugger) {
		// destroy the debugger before destroying the world
		NewtonDebuggerDestroyServer (m_visualDebugger);
		m_visualDebugger = NULL;
	}
#endif
}

void DemoVisualDebugerListener::PreUpdate (const NewtonWorld* const world, dFloat timestep)
{
	#ifdef USE_VISUAL_DEBUGGER 
		// present change in the world to the debugger
		if (m_visualDebugger) {
			NewtonDebuggerServe (m_visualDebugger, timestep);
		}
	#endif
}

void DemoVisualDebugerListener::PostUpdate (const NewtonWorld* const world, dFloat timestep)
{

}

#endif