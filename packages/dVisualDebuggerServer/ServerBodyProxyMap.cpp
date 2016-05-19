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

#include "DebuggerServer.h"
#include "ServerBodyProxyMap.h"
#include "NewtonDebuggerServer.h"

#define DEBUGGER_MEM_BUFFER_SIZE	(1024 * 32)

#if 0

/*
ServerBodyProxyMap::ServerBodyProxyMap()
	:m_lru(0)
	,m_bodyEnumerator(0)
	,m_bodyBufferCapacity (32)
	,m_memCount(0)
	,m_memBufferCapacity (DEBUGGER_MEM_BUFFER_SIZE)
{
	m_bodyBuffer = new NewtonBody*[m_bodyBufferCapacity];
	m_memBuffer = new char[m_memBufferCapacity];
}

ServerBodyProxyMap::~ServerBodyProxyMap()
{
	delete[] m_memBuffer;
	delete[] m_bodyBuffer;
}

void ServerBodyProxyMap::AddNewBodies (NewtonWorld* const world)
{
	m_lru ++;

	// see if there are new bodies to send to the debugger
	int bodyCount = 0;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		NewtonCollision* const collisionKey = NewtonBodyGetCollision(body);
		dTreeNode* node = Find (collisionKey);
		if (!node) {
			node = Insert (collisionKey);
			BodyProxy& proxy = node->GetInfo();
			proxy.m_body = body;
			proxy.m_bodyId = m_bodyEnumerator;
			m_bodyEnumerator ++;

			if (bodyCount == m_bodyBufferCapacity) {
				NewtonBody** const buffer = new NewtonBody*[m_bodyBufferCapacity * 2];
				memcpy (buffer, m_bodyBuffer, bodyCount * sizeof (NewtonBody*));
				delete[] m_bodyBuffer; 
				m_bodyBuffer = buffer;
				m_bodyBufferCapacity *= 2;
			}
			m_bodyBuffer[bodyCount] = body;
			bodyCount ++;
		}
		BodyProxy& proxy = node->GetInfo();
		proxy.m_lru = m_lru;
	}


	if (bodyCount) {
		int code = NEWTON_DEBUGGER_ADD_BODIES;
		SerializeData (&code, sizeof (code));
		NewtonSerializeBodyArray(world, m_bodyBuffer, bodyCount, BodySerialization, SerializeData, this);

		// Send Command tis command here, before resetting the buffer
		if (!SendBuffer ()) {
			RemoveAll();
		}

		if (m_memBufferCapacity > DEBUGGER_MEM_BUFFER_SIZE) {
			delete[] m_memBuffer;
			m_memBufferCapacity = DEBUGGER_MEM_BUFFER_SIZE;
			m_memBuffer = new char[m_memBufferCapacity];
		}
	}
	
	// remove all deleted bodies 
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dTreeNode* const node = iter.GetNode();
		iter ++;
		BodyProxy& proxy = node->GetInfo();
		if (proxy.m_lru < m_lru) {
			int code = NEWTON_DEBUGGER_REMOVE_BODY;
			SerializeData (&code, sizeof (code));
			SerializeData (&proxy.m_bodyId, sizeof (proxy.m_bodyId));
			Remove (node);
		}
	}


/*

		BodyProxy& proxy = node->GetInfo();
		proxy.m_lru = m_lru;

		int state = NewtonBodyGetSleepState(body);
		if (state) {
			dFloat mass; 
			dFloat Ixx; 
			dFloat Iyy; 
			dFloat Izz; 
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			state = (mass == 0.0f); 
		}

		if (!state) {
			if (count == m_bodyBufferCapacity) {
				NewtonBody** const buffer = new NewtonBody*[m_bodyBufferCapacity * 2];
				memcpy (buffer, m_buffer, count * sizeof (NewtonBody*));
				delete[] m_bodyBuffer; 
				m_bodyBuffer = buffer;
				m_bodyBufferCapacity *= 2;
			}
			m_bodyBuffer[count] = body;
			count ++;
		}
	}
*/
}

void ServerBodyProxyMap::RemoveBodies ()
{
	int code = NEWTON_DEBUGGER_REMOVE_BODY;

	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		BodyProxy& proxy = node->GetInfo();
		SerializeData (&code, sizeof (code));
		SerializeData (&proxy.m_bodyId, sizeof (proxy.m_bodyId));
	}
	SendBuffer ();
}






bool ServerBodyProxyMap::SendBuffer ()
{
	DebuggerServer* const me = (DebuggerServer*) this;
	bool state = me->SendData (m_memBuffer, m_memCount);
	m_memCount = 0;
	return state;
}

/*
int DebuggerServer::ServerBodyProxyMap::GetBodies(NewtonWorld* const world)
{
	m_lru ++;
	int count = 0;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		NewtonCollision* const collisionKey = NewtonBodyGetCollision(body);
		dTreeNode* node = Find (collisionKey);
		if (!node) {
			//BodyProxy proxy;
			node = Insert (collisionKey);
			BodyProxy& proxy = node->GetInfo();
			proxy.m_body = body;
		}
		BodyProxy& proxy = node->GetInfo();
		proxy.m_lru = m_lru;

		
		int state = NewtonBodyGetSleepState(body);
		if (state) {
			dFloat mass; 
			dFloat Ixx; 
			dFloat Iyy; 
			dFloat Izz; 
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			state = (mass == 0.0f); 
		}

		if (!state) {
			if (count == m_capacity) {
				NewtonBody** const buffer = new NewtonBody*[m_capacity * 2];
				memcpy (buffer, m_buffer, count * sizeof (NewtonBody*));
				delete[] m_buffer; 
				m_buffer = buffer;
				m_capacity *= 2;
			}
			m_buffer[count] = body;
			count ++;
		}
	}

	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dTreeNode* const node = iter.GetNode();
		iter ++;
		BodyProxy& proxy = node->GetInfo();
		if (proxy.m_lru < m_lru) {
			Remove (node);
		}
	}

	return count;	
}
*/

DebuggerServer::DebuggerServer(NewtonWorld* const world)
	:ServerBodyProxyMap()
	,m_server(NULL)
	,m_packet(NULL)
	,m_peer(NULL)
	,m_world (world)
{
	if (!enet_initialize ()) {
		ENetAddress address;

		// Bind the server to the default localhost.
		// A specific host address can be specified by
		// enet_address_set_host (& address, "x.x.x.x");
		address.host = ENET_HOST_ANY;

		// Bind the server to port 1234. 
		address.port = DEBUGGER_PORT_ID;

		m_server = enet_host_create (& address, // the address to bind the server host to  
									1, // allow up to 32 clients and/or outgoing connections
									0, // assume any amount of incoming bandwidth
									0); // assume any amount of outgoing bandwidth

		if (m_server) {
			/* Initiate the connection, allocating the two channels 0 and 1. */
			m_peer = enet_host_connect ((ENetHost*) m_server, & address, 2);    
			if(m_peer) {
				m_packet = enet_packet_create (D_VISUAL_DEBUGER_PACKET_NAME, strlen (D_VISUAL_DEBUGER_PACKET_NAME) + 1, ENET_PACKET_FLAG_RELIABLE);
				dAssert (m_packet);
			}
		} else {
			enet_deinitialize ();
		}

	}
}

DebuggerServer::~DebuggerServer(void)
{
	RemoveBodies ();

	if (m_server) {
		enet_peer_reset ((ENetPeer*)m_peer);
		enet_packet_destroy ((ENetPacket*) m_packet);
		enet_host_destroy ((ENetHost*) m_server);
		enet_deinitialize ();
	}
}


bool DebuggerServer::ServerStatus() const
{
	return m_server ? true : false;
}




void DebuggerServer::Serve(dFloat timestep)
{
	AddNewBodies (m_world);

	int code = NEWTON_DEBUGGER_BEGIN_UPDATE;
	SerializeData (&code, sizeof (code));

	SerializeData (&timestep, sizeof (timestep));


	code = NEWTON_DEBUGGER_END_UPDATE;
	SerializeData (&code, sizeof (code));

		
	SendBuffer();
}

bool DebuggerServer::SendData (const void* const buffer, int size) const
{

//ENET_API int  enet_peer_send (ENetPeer *, enet_uint8, ENetPacket *);
//	ENetPacket* const packet = (ENetPacket*)m_packet;
//	enet_peer_send ((ENetPeer*)m_peer, 0, packet);

	ENetEvent event;
	bool state = false;
	if (enet_host_service ((ENetHost*) m_server, &event, 1) > 0 && event.type == ENET_EVENT_TYPE_CONNECT) {
		state = true;
		puts ("Connection to some.server.net:1234 succeeded.");
	}


	return state;
//	
//	enet_packet_resize (packet, size);
//	enet_peer_send (peer, 0, packet);
//	enet_packet_resize (buffer, size);
}
#endif



ServerBodyProxyMap::ServerBodyProxyMap()
	:m_lru(1)
	,m_bodyBufferCapacity (32)
	,m_memCount(0)
	,m_memBufferCapacity (DEBUGGER_MEM_BUFFER_SIZE)
{
	m_bodyBuffer = new NewtonBody*[m_bodyBufferCapacity];
	m_memBuffer = new char[m_memBufferCapacity];
}

ServerBodyProxyMap::~ServerBodyProxyMap()
{
	delete[] m_memBuffer;
	delete[] m_bodyBuffer;
}


void ServerBodyProxyMap::SerializeData (const void* const buffer, int size)
{
	while ((m_memCount + size) >= m_memBufferCapacity) {
		char* const memBuffer = new char[m_memBufferCapacity * 2];
		memcpy (memBuffer, m_memBuffer, m_memCount);
		delete[] m_memBuffer; 
		m_memBuffer = memBuffer;
		m_memBufferCapacity *= 2;
	}
	memcpy (&m_memBuffer[m_memCount], buffer, size);
	m_memCount += size;
}

void ServerBodyProxyMap::SerializeData (void* const serializeHandle, const void* const buffer, int size)
{
	ServerBodyProxyMap* const me = (DebuggerServer*) serializeHandle;
	me->SerializeData (buffer, size);
}


void ServerBodyProxyMap::BodySerialization (NewtonBody* const body, NewtonSerializeCallback serializeCallback, void* const serializeHandle)
{
	ServerBodyProxyMap* const me = (DebuggerServer*) serializeHandle;
	me->BodySerialization (body, serializeCallback);
}


void ServerBodyProxyMap::BodySerialization (NewtonBody* const body, NewtonSerializeCallback serializeCallback)
{
	int key = NewtonBodyGetID(body);

	//dTreeNode* const node = Find (key);
	//BodyProxy& proxy = node->GetInfo();
	//serializeCallback (this, &proxy.m_bodyId, sizeof (proxy.m_bodyId));
	serializeCallback (this, &key, sizeof (key));
}



void ServerBodyProxyMap::AddNewBodies ()
{
	 DebuggerServer* const me = (DebuggerServer*) this;
	 NewtonWorld* const world = me->m_world;

	m_lru ++;

	// see if there are new bodies to send to the debugger
	int bodyCount = 0;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		int key = NewtonBodyGetID(body);
		dTreeNode* node = Find(key);
		if (!node) {
			node = Insert (key);

			BodyProxy& proxy = node->GetInfo();
			proxy.m_body = body;

			if (bodyCount == m_bodyBufferCapacity) {
				NewtonBody** const buffer = new NewtonBody*[m_bodyBufferCapacity * 2];
				memcpy (buffer, m_bodyBuffer, bodyCount * sizeof (NewtonBody*));
				delete[] m_bodyBuffer; 
				m_bodyBuffer = buffer;
				m_bodyBufferCapacity *= 2;
			}
			m_bodyBuffer[bodyCount] = body;
			bodyCount ++;
		}
		BodyProxy& proxy = node->GetInfo();
		proxy.m_lru = m_lru;
	}


	if (bodyCount) {
		int code = NEWTON_DEBUGGER_ADD_BODIES;
		SerializeData (&code, sizeof (code));

		dAssert (0);
//		NewtonSerializeBodyArray(world, m_bodyBuffer, bodyCount, BodySerialization, SerializeData, this);

		// Send Command this command here, before resetting the buffer
		if (!me->SendData (m_bodyBuffer, m_memCount)) {
			RemoveAll();
		}
		m_memCount = 0;
	}
	
	// remove all deleted bodies 
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dTreeNode* const node = iter.GetNode();
		iter ++;

		BodyProxy& proxy = node->GetInfo();
		if (proxy.m_lru < m_lru) {
			int key = node->GetKey();
			int code = NEWTON_DEBUGGER_REMOVE_BODY;
			SerializeData (&code, sizeof (code));
			SerializeData (&key, sizeof (key));
			Remove (node);
		} else {
			dAssert (0);
/*
			NewtonBody* const body = proxy.m_body;
			int state = NewtonBodyGetSleepState(body);
			if (state) {
				dFloat mass; 
				dFloat Ixx; 
				dFloat Iyy; 
				dFloat Izz; 
				NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
				state = (mass == 0.0f); 
			}

			if (!state) {
				if (count == m_bodyBufferCapacity) {
					NewtonBody** const buffer = new NewtonBody*[m_bodyBufferCapacity * 2];
					memcpy (buffer, m_buffer, count * sizeof (NewtonBody*));
					delete[] m_bodyBuffer; 
					m_bodyBuffer = buffer;
					m_bodyBufferCapacity *= 2;
				}
				m_bodyBuffer[count] = body;
				count ++;
			}
*/
		}
	}
}


