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


#include <toolbox_stdafx.h>
#include "NewtonModelEditor.h"
#include "NewtonModelEditorApp.h"

IMPLEMENT_APP(NewtonModelEditorApp)

void *operator new (size_t size) 
{ 
	return dContainersAlloc::Alloc (size);
}                                          

void operator delete (void* ptr) 
{ 
	dContainersAlloc::Free (ptr);
}

int NewtonModelEditorApp::m_totalMemoryUsed;

NewtonModelEditorApp::NewtonModelEditorApp()
{
}

NewtonModelEditorApp::~NewtonModelEditorApp()
{
}


bool NewtonModelEditorApp::OnInit()
{
	// check for memory leaks
#if defined(_DEBUG) && defined(_MSC_VER)
	// Track all memory leaks at the operating system level.
	// make sure no Newton tool or utility leaves leaks behind.
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF|_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF));
#endif

	// Set the memory allocation function before creation the newton world
	// this is the only function that can be called before the creation of the newton world.
	// it should be called once, and the the call is optional 
	NewtonSetMemorySystem (PhysicsAlloc, PhysicsFree);

//	int version = NewtonWorldGetVersion();
//	wxString tittle;
//	tittle.Printf (wxT ("Newton %d.%02d SDK demos"), version / 100, version % 100);

	NewtonModelEditor* const frame = new NewtonModelEditor (APPLICATION_NAME, wxDefaultPosition, wxSize(1024, 768));
	frame->Show(true);
	SetTopWindow(frame);
/*
	// initialize open gl graphics
	if (frame->m_scene) {
		frame->m_scene->InitGraphicsSystem();
	}

	// load the default Scene		
	wxMenuEvent loadDemo (wxEVT_COMMAND_MENU_SELECTED, NewtonDemos::ID_RUN_DEMO + DEFAULT_SCENE);
	frame->GetEventHandler()->ProcessEvent(loadDemo);
*/
	return true;
}

// memory allocation for Newton
void* NewtonModelEditorApp::PhysicsAlloc (int sizeInBytes)
{
	m_totalMemoryUsed += sizeInBytes;
	return new char[sizeInBytes];
}

// memory free use by the engine
void NewtonModelEditorApp::PhysicsFree (void* ptr, int sizeInBytes)
{
	m_totalMemoryUsed -= sizeInBytes;
	delete[] (char*)ptr;
}




