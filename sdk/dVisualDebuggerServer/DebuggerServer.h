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

#ifndef __DEBUGGER_SERVER_CLASS__H
#define __DEBUGGER_SERVER_CLASS__H

#include <enet/enet.h>

#ifdef WIN32
	#include <windows.h>
#endif


#include <Newton.h>
#include <dVector.h>
#include <dMatrix.h>
#include <dQuaternion.h>
#include <dMathDefines.h>

#include <dCRC.h>
#include <dHeap.h>
#include <dList.h>
#include <dTree.h>
#include <dRtti.h>

#include "DebuggerServer.h"
#include "ServerBodyProxyMap.h"
#include "NewtonDebuggerServer.h"

#define D_DEBUGGER_PORT_ID				1734
#define D_VISUAL_DEBUGER_PACKET_NAME	"localhost"



class DebuggerServer: public ServerBodyProxyMap
{
	public:
	DebuggerServer(NewtonWorld* const world);
	~DebuggerServer(void);

	bool ServerStatus() const;
	void Serve(dFloat timestep);
	bool SendData (const void* const buffer, int size);

	private:
	void* m_server;
	void* m_address; 
	void* m_event;
	NewtonWorld* m_world;
	bool m_connected;

	friend class ServerBodyProxyMap;
};


#endif