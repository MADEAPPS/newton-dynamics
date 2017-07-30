/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _NEWTON_DEBUGGER_H
#define _NEWTON_DEBUGGER_H



#ifdef __cplusplus 
extern "C" {
#endif

	#define NEWTON_DEBUGGER_ADD_BODIES		0
	#define NEWTON_DEBUGGER_REMOVE_BODY		1
	#define NEWTON_DEBUGGER_BEGIN_UPDATE	2
	#define NEWTON_DEBUGGER_END_UPDATE		3
	#define NEWTON_CHECK_CONNECTED			4

	void* NewtonDebuggerCreateServer (NewtonWorld* const world);
	void NewtonDebuggerDestroyServer (void* const server);
	void NewtonDebuggerServe (void* const server, dFloat timestep);


#ifdef __cplusplus 
}
#endif

#endif