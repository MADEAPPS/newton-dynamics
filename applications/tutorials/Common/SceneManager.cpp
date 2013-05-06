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
#include "Entity.h"
#include "SceneManager.h"
#include "SoundManager.h"
#include "CollectInputAndUpdateCamera.h"

SceneManager::SceneManager(void)
{
	m_entCount = 0;
	m_soundManager = new SoundManager;
	if (m_soundManager->m_audioOn == 0) {
		delete m_soundManager;
		m_soundManager = NULL;
	}
}

SceneManager::~SceneManager(void)
{
	delete m_soundManager;
	for (int i = 0; i < m_entCount; i ++) {
		delete m_entityArray[i];
	}
}

SoundManager* SceneManager::GetSoundManager() const
{
	return m_soundManager;
}

Entity* SceneManager::CreateEntity()
{
	Entity*	ent;
	ent = new Entity;

	m_entityArray[m_entCount] = ent;
	m_entCount ++;

	return ent;
}


void SceneManager::SetIntepolationParam(dFloat param)
{
	if (param > 1.0f) {
		param = 1.0f;
	}
	m_intepolationParam = param;

}

void SceneManager::Render ()
{
	// Update the Camera interpolated position
	UpdateCamera (m_intepolationParam);

	// Render the complete Scene
	glShadeModel (GL_SMOOTH);

	// Culling. 
	glCullFace( GL_BACK );
	glFrontFace( GL_CCW );
	glEnable( GL_CULL_FACE );

	// z buffer test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);

	glEnable (GL_LIGHTING);
	for (int i = 0; i < m_entCount; i ++) {
		m_entityArray[i]->Render (m_intepolationParam);
	}
}

