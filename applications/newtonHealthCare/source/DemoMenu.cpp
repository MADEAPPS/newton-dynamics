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
#include "DemoMenu.h"
#include "NewtonDemos.h"
#include "DemoEntityManager.h"





DemoMenu::DemoMenu(FXComposite* const parent, NewtonDemos* const mainFrame)
	:FXMenuBar(parent, new FXToolBarShell(mainFrame, FRAME_RAISED), LAYOUT_DOCK_SAME|LAYOUT_SIDE_TOP|LAYOUT_FILL_X|FRAME_RAISED)
{
	new FXToolBarGrip(this, this, FXMenuBar::ID_TOOLBARGRIP,TOOLBARGRIP_DOUBLE);

	m_threadsTracks[0] = 1;
	m_threadsTracks[1] = 2;
	m_threadsTracks[2] = 3;
	m_threadsTracks[3] = 4;
	m_threadsTracks[4] = 8;
	m_threadsTracks[5] = 12;
	m_threadsTracks[6] = 16;


	// adding the file menu
	{
		m_fileMenu = new FXMenuPane(this);
		new FXMenuTitle(this, "File", NULL, m_fileMenu);

		new FXMenuCommand(m_fileMenu, "New", NULL, mainFrame, NewtonDemos::ID_NEW);
		new FXMenuSeparator(m_fileMenu);

		new FXMenuCommand(m_fileMenu, "Load", NULL, mainFrame, NewtonDemos::ID_LOAD);
		new FXMenuCommand(m_fileMenu, "Save", NULL, mainFrame, NewtonDemos::ID_SAVE);
		new FXMenuSeparator(m_fileMenu);

		new FXMenuCommand(m_fileMenu, "Import serialize scene", NULL, mainFrame, NewtonDemos::ID_DESERIALIZE);
		new FXMenuCommand(m_fileMenu, "Export serialize scene", NULL, mainFrame, NewtonDemos::ID_SERIALIZE);
		new FXMenuSeparator(m_fileMenu);

		new FXMenuCommand(m_fileMenu, "Quit", NULL, getApp(), FXApp::ID_QUIT);
	}

	// option menu
	{
		m_optionsMenu = new FXMenuPane(this);
		new FXMenuTitle(this, "Options", NULL, m_optionsMenu);

		new FXMenuCheck(m_optionsMenu, "Autosleep off", mainFrame, NewtonDemos::ID_AUTOSLEEP_MODE);
		new FXMenuCheck(m_optionsMenu, "Hide visual meshes", mainFrame, NewtonDemos::ID_HIDE_VISUAL_MESHES);
		new FXMenuCheck(m_optionsMenu, "Show collision Mesh", mainFrame, NewtonDemos::ID_SHOW_COLLISION_MESH);
		new FXMenuCheck(m_optionsMenu, "Show contact points", mainFrame, NewtonDemos::ID_SHOW_CONTACT_POINTS);
		new FXMenuCheck(m_optionsMenu, "Show aabb", mainFrame, NewtonDemos::ID_SHOW_AABB);
		new FXMenuCheck(m_optionsMenu, "Parallel solver on", mainFrame, NewtonDemos::ID_USE_PARALLEL_SOLVER);
//		if (mainFrame->m_useParallelSolver) {
	//		parallel->setCheck(true);
		//}

		new FXMenuSeparator(m_optionsMenu);

		m_cpuModes[0] = new FXMenuRadio(m_optionsMenu, "Use x87 instructions", &mainFrame->m_cpuInstructionSelection, NewtonDemos::ID_USE_X87_INSTRUCTIONS);
		m_cpuModes[1] = new FXMenuRadio(m_optionsMenu, "Use sse instructions", &mainFrame->m_cpuInstructionSelection, NewtonDemos::ID_USE_SIMD_INSTRUCTIONS);
		m_cpuModes[2] = new FXMenuRadio(m_optionsMenu, "Use avx instructions", &mainFrame->m_cpuInstructionSelection, NewtonDemos::ID_USE_AVX_INSTRUCTIONS);


		new FXMenuSeparator(m_optionsMenu);
		new FXMenuRadio(m_optionsMenu, "dynamics broad phase", &mainFrame->m_broadPhaseSelection, NewtonDemos::ID_DYNAMICS_BROADPHASE);
		new FXMenuRadio(m_optionsMenu, "static broad phase", &mainFrame->m_broadPhaseSelection, NewtonDemos::ID_STATIC_BROADPHASE);
		new FXMenuRadio(m_optionsMenu, "hybrid broad phase", &mainFrame->m_broadPhaseSelection, NewtonDemos::ID_HYBRID_BROADPHASE);


		new FXMenuSeparator(m_optionsMenu);
		FXMenuCheck* const concurrentProfiler = new FXMenuCheck(m_optionsMenu, "Show concurrent profiler", mainFrame, NewtonDemos::ID_SHOW_CONCURRENCE_PROFILER);
		if (mainFrame->m_concurrentProfilerState) {
			concurrentProfiler->setCheck(true);
		}
		FXMenuCheck* const threadProfiler = new FXMenuCheck(m_optionsMenu, "Show micro thread profiler", mainFrame, NewtonDemos::ID_SHOW_PROFILER);
		if (mainFrame->m_threadProfilerState) {
			threadProfiler->setCheck(true);
		}

		new FXMenuSeparator(m_optionsMenu);
		new FXMenuCommand(m_optionsMenu, "select all profiler", NULL, mainFrame, NewtonDemos::ID_SELECT_ALL_PROFILERS);
		new FXMenuCommand(m_optionsMenu, "udeselect all profiler", NULL, mainFrame, NewtonDemos::ID_UNSELECT_ALL_PROFILERS);

		m_profilerSubMenu = new FXMenuPane(this);
		m_profilerTracksMenu[0] = new FXMenuCheck(m_profilerSubMenu, "show global physics update performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 0);
		m_profilerTracksMenu[1] = new FXMenuCheck(m_profilerSubMenu, "global collision update performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 1);
		m_profilerTracksMenu[2] = new FXMenuCheck(m_profilerSubMenu, "broad phase collision performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 2);
		m_profilerTracksMenu[3] = new FXMenuCheck(m_profilerSubMenu, "narrow phase collision performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 3);
		m_profilerTracksMenu[4] = new FXMenuCheck(m_profilerSubMenu, "global dynamics update performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 4);
		m_profilerTracksMenu[5] = new FXMenuCheck(m_profilerSubMenu, "dynamics setup performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 5);
		m_profilerTracksMenu[6] = new FXMenuCheck(m_profilerSubMenu, "dynamics solver performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 6);
		m_profilerTracksMenu[7] = new FXMenuCheck(m_profilerSubMenu, "force and torque callback performance chart", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 7);
		m_profilerTracksMenu[8] = new FXMenuCheck(m_profilerSubMenu, "pre simulation listener", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 8);
		m_profilerTracksMenu[9] = new FXMenuCheck(m_profilerSubMenu, "post simulation listener", mainFrame, NewtonDemos::ID_SHOW_PHYSICS_PROFILER + 9);
		if (mainFrame->m_physicProfilerState) {
			m_profilerTracksMenu[0]->setCheck(true);
		}
		new FXMenuCascade(m_optionsMenu, "select sub profiler", NULL, m_profilerSubMenu);


		new FXMenuSeparator(m_optionsMenu);
		new FXMenuCheck(m_optionsMenu, "Concurrent physics update", mainFrame, NewtonDemos::ID_CONCURRENT_PHYSICS_UPDATE);
		m_microThreadedsSubMenu = new FXMenuPane(this);
		FXMenuRadio* threadMenus[128];
		for (int i = 0 ; i < sizeof (m_threadsTracks)/ sizeof (m_threadsTracks[0]); i ++) {
			char label[1024];
			sprintf (label, "%d micro threads", m_threadsTracks[i]);
			threadMenus[i] = new FXMenuRadio(m_microThreadedsSubMenu, label, &mainFrame->m_microthreadCountSelection, NewtonDemos::ID_SELECT_MICROTHREADS + i);
		}
		threadMenus[0]->setCheck(true);
		new FXMenuCascade(m_optionsMenu, "select microThread count", NULL, m_microThreadedsSubMenu);
		
	}

	// add help menu
	{
		m_helpMenu = new FXMenuPane(this);
		new FXMenuTitle(this, "Help", NULL, m_helpMenu);
		new FXMenuCommand(m_helpMenu, "About", NULL, mainFrame, NewtonDemos::ID_ABOUT);
	}
}

DemoMenu::~DemoMenu(void)
{
	delete m_microThreadedsSubMenu;
	delete m_profilerSubMenu;
	
	delete m_optionsMenu;
	delete m_helpMenu;
	delete m_fileMenu;
}

void DemoMenu::LoadDemo (DemoEntityManager* const scene, int index)
{
	scene->makeCurrent();

	scene->swapBuffers();  // added this line
	scene->makeNonCurrent();
}

