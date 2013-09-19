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

#ifndef __NEWTON_MODEL_EDITOR_APP_H_
#define __NEWTON_MODEL_EDITOR_APP_H_

#define APPLICATION_NAME		wxT("Newton Model Editor")
#define APPLICATION_VERSION		100


class NewtonModelEditorApp: public wxApp
{
	virtual bool OnInit();
	static void* PhysicsAlloc (int sizeInBytes);
	static void PhysicsFree (void* ptr, int sizeInBytes);

	static int m_totalMemoryUsed;
};



#endif