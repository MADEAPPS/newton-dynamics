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

// Tutorial_101_GettingStarted.cpp : Defines the entry point for the console application.
//
#include <stdafx.h>
#include <dVector.h>
#include "OpenGlUtil.h"
#include "SceneManager.h"
#include "TutorialCode.h"
#include "CollectInputAndUpdateCamera.h"

//#define USE_VISUAL_DEBUGGER

#define DEMO_PHYSICS_FPS		  120.0f
#define DEMO_FPS_IN_MICROSECUNDS  (int (1000000.0f/DEMO_PHYSICS_FPS))
#define MAX_PHYSICS_LOOPS		  1

static int g_currentTime;
static int g_physicTime;
static int g_timeAccumulator = DEMO_FPS_IN_MICROSECUNDS;

static void* g_newtonDebugger;
static NewtonWorld* g_world;
static SceneManager* g_sceneManager;

static void ShutDown ();
static void* AllocMemory (int sizeInBytes);
static void FreeMemory (void *ptr, int sizeInBytes);
static void AdvanceSimulation (int timeInMilisecunds);

#pragma warning (disable: 4100) //unreferenced formal parameter
#pragma warning (disable: 4702) //unreachable code



int main (int argc, char* argv[])
{

	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF|_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF));

	// initialize graphics system
	if (!InitGraphicsSystem (800, 600)) {
		exit (0);
	}
	// set a callback for destroying the world ate termination
	atexit (ShutDown);

	// Create a Simple Scene Manager
	g_sceneManager = new SceneManager();

	// set the memory allocators
	NewtonSetMemorySystem (AllocMemory, FreeMemory);

	// create the Newton World
	g_world = NewtonCreate ();

	// use the standard x87 floating point model  
	NewtonSetPlatformArchitecture (g_world, 0);

	// set a fix world size
//	dVector minSize (-500.0f, -500.0f, -500.0f);
//	dVector maxSize ( 500.0f,  500.0f,  500.0f);
//	NewtonSetWorldSize (g_world, &minSize[0], &maxSize[0]); 

	// initialize Newton Visual Debugger
#ifdef USE_VISUAL_DEBUGGER
	g_newtonDebugger = NewtonDebuggerCreateServer ();
#endif

	// configure the Newton world to use iterative solve mode 0
	// this is the most efficient but the less accurate mode
	NewtonSetSolverModel (g_world, 1);

	// now populate the world with Graphic and physical entities 
	CreateScene(g_world, g_sceneManager);

    // run the main application loop until the user terminate the app
    for (;;) {
        // Draw the screen. 
        AdvanceSimulation (GetTimeInMicrosenconds ());
    }

    // Never reached. 
    return 0;
}


// on termination the application must destroy the Newton world 
void ShutDown ()
{
#ifdef USE_VISUAL_DEBUGGER
	// destroy the debugger server
	NewtonDebuggerDestroyServer (g_newtonDebugger);
#endif

	// destroy all rigid bodies, this is no necessary because Newton Destroy world will also destroy all bodies
	// but if you want to change level and restart you can call this function to clean the world without destroying the world.
	NewtonDestroyAllBodies (g_world);

	// finally destroy the newton world 
	NewtonDestroy (g_world);

	// now we need to destroy the Graphics entities
	delete g_sceneManager;
}


// this is the call back for allocation newton memory
void* AllocMemory (int sizeInBytes)
{
	return malloc (sizeInBytes);
}

// this is the callback for freeing Newton Memory
void FreeMemory (void *ptr, int sizeInBytes)
{
	free (ptr);
}


void AdvanceSimulation (int timeInMilisecunds)
{
	// do the physics simulation here
	int deltaTime;
	int physicLoopsTimeAcc;
	dFloat fps;
	dFloat physicTime;


	// get the time step
	deltaTime = timeInMilisecunds - g_currentTime;
	g_currentTime = timeInMilisecunds;
	g_timeAccumulator += deltaTime;

	physicTime = 0;
	// advance the simulation at a fix step
	int loops = 0;
	physicLoopsTimeAcc = 0;


	while ((loops < MAX_PHYSICS_LOOPS) && (g_timeAccumulator >= DEMO_FPS_IN_MICROSECUNDS))
	{
		loops ++;

		// Process incoming events. 
		ProcessEvents (g_world);

		// sample time before the Update
		g_physicTime = GetTimeInMicrosenconds ();

		// run the newton update function
		NewtonUpdate (g_world, (1.0f / DEMO_PHYSICS_FPS));

		// calculate the time spent in the physical Simulation
		g_physicTime = GetTimeInMicrosenconds () - g_physicTime;

		// call the visual debugger to show the physics scene
#ifdef USE_VISUAL_DEBUGGER
		NewtonDebuggerServe (g_newtonDebugger, g_world);
#endif

		// subtract time from time accumulator
		g_timeAccumulator -= DEMO_FPS_IN_MICROSECUNDS;
		physicTime ++;

		physicLoopsTimeAcc += g_physicTime;
	}

	if (loops > MAX_PHYSICS_LOOPS) {
		g_physicTime = physicLoopsTimeAcc;
		g_timeAccumulator = DEMO_FPS_IN_MICROSECUNDS;
	}


	// Clear the color and depth buffers. 
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// calculate the interpolation parameter for smooth rendering 
	g_sceneManager->SetIntepolationParam(dFloat (g_timeAccumulator) / dFloat(DEMO_FPS_IN_MICROSECUNDS));

	// do the rendering here entire Scene
	g_sceneManager->Render ();

	// display the frame rate
	// smooth the fps by averaging the last few rendering time steps;
	int dtAcc = 0;
	static int fpsIndex;
	static long int smoothFPS[128];

	smoothFPS[fpsIndex] = deltaTime;
	fpsIndex = (fpsIndex + 1) % (sizeof (smoothFPS) / sizeof (smoothFPS[0]));
	for (int i = 0; i < (sizeof (smoothFPS) / sizeof (smoothFPS[0])); i ++) {
		dtAcc += smoothFPS[i];
	}
	dtAcc /= (sizeof (smoothFPS) / sizeof (smoothFPS[0]));

	fps = 1000000.0f / dFloat (dtAcc);
	physicTime = g_physicTime * dFloat(1000.0f / 1000000.0f);
	Print (dVector (1.0f, 1.0f, 0.0f, 0.0f), 10, 10, "fps: %6.2f", fps);
	Print (dVector (1.0f, 1.0f, 0.0f, 0.0f), 10, 22, "physic time (milliseconds): %5.2f", physicTime);

	// show GL rendered Scene 
	glFlush();
	SDL_GL_SwapBuffers( );
}






