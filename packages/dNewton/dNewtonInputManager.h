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


#ifndef _D_NEWTON_RAY_PEEKING_H_
#define _D_NEWTON_RAY_PEEKING_H_


class dNewton;


#define D_NEWTON_INPUT_MANGER_PLUGIN_NAME	"__dnetwon_inputManager__"


class dNewtonInputController: public CustomControllerBase
{
	public:
	virtual void PreUpdate(dFloat timestep, int threadIndex)
	{
		//do nothing;
	}

	virtual void PostUpdate(dFloat timestep, int threadIndex)
	{
		//do nothing;
	}
};


class dNewtonInputManager: public CustomControllerManager<dNewtonInputController> 
{
	public:
	CNEWTON_API dNewtonInputManager (dNewton* const world);
	CNEWTON_API virtual ~dNewtonInputManager();

//	unsigned* GetLockHandle____()
//	{
//		return &m_lock____;
//	}
	virtual void OnBeginUpdate (dFloat timestepInSecunds) = 0;
	virtual void OnEndUpdate (dFloat timestepInSecunds) = 0;

	private:
	CNEWTON_API void PreUpdate(dFloat timestep);
	CNEWTON_API void PostUpdate (dFloat timestep);
	virtual void Debug () const {};

	protected:
	dNewton* m_world;
//	unsigned m_lock____;
};

#endif
