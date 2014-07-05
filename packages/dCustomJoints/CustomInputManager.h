/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/


#ifndef D_CUSTOM_IMPUT_MANAGER_H_
#define D_CUSTOM_IMPUT_MANAGER_H_

#include <CustomJointLibraryStdAfx.h>
#include <CustomControllerManager.h>

#define INPUT_PLUGIN_NAME				"__inputManager__"


class CustomInputController: public CustomControllerBase
{
	public:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex)
	{
		//do nothing;
	}

	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex)
	{
		//do nothing;
	}
};


class CustomInputManager: public CustomControllerManager<CustomInputController> 
{
	public:
	CUSTOM_JOINTS_API CustomInputManager (NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~CustomInputManager();

	virtual void OnBeginUpdate (dFloat timestepInSecunds) = 0;
	virtual void OnEndUpdate (dFloat timestepInSecunds) = 0;

	protected:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep);
	CUSTOM_JOINTS_API virtual void PostUpdate (dFloat timestep);
	CUSTOM_JOINTS_API virtual void Debug () const {};
};

#endif
