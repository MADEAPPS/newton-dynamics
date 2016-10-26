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

#include "DebuggerServer.h"



DebuggerServer::DebuggerServer(NewtonWorld* const world)
	:ServerBodyProxyMap()
	,m_server(NULL)
	,m_address(NULL)
	,m_world (world)
	,m_connected(false)
{
	if (!enet_initialize()) {
		m_event = new ENetEvent;
		ENetAddress* const address = new ENetAddress;

		m_address = address;
		address->host = ENET_HOST_ANY;
		address->port = D_DEBUGGER_PORT_ID;
		m_server = enet_host_create(address, 1, 2, 0, 0);
				
		if (m_server == NULL) {
			delete address;
			delete (ENetEvent*) m_event;
		}
	}
}

DebuggerServer::~DebuggerServer(void)
{
	if (m_server) {
		ENetHost* const server = (ENetHost*)m_server;	
		enet_peer_reset_queues (&server->peers[0]);

		delete (ENetEvent*) m_event;
		delete (ENetAddress*) m_address;
		enet_host_destroy((ENetHost*) m_server);
		enet_deinitialize();
	}
}


bool DebuggerServer::ServerStatus() const
{
	return m_server ? true : false;
}




void DebuggerServer::Serve(dFloat timestep)
{
timestep = 0;
return;

//	if (m_connected) {
//		AddNewBodies ();
//	}
/*
	int code = NEWTON_DEBUGGER_BEGIN_UPDATE;
	SerializeData (&code, sizeof (code));

	SerializeData (&timestep, sizeof (timestep));


	code = NEWTON_DEBUGGER_END_UPDATE;
	SerializeData (&code, sizeof (code));
	SendBuffer();
*/


//	int connected = NEWTON_CHECK_CONNECTED;
//	SendData (&connected, sizeof (connected));
}

bool DebuggerServer::SendData (const void* const buffer, int size)
{
	ENetEvent* const event = (ENetEvent*) m_event;
	ENetHost* const server = (ENetHost*)m_server;	
	while (enet_host_service(server, event, 1) > 0) 
	{
		switch (event->type) 
		{
			case ENET_EVENT_TYPE_CONNECT:
				m_connected = true;
				break;

			case ENET_EVENT_TYPE_RECEIVE:
				dAssert (m_connected);
				if (event->peer->data == NULL) {
					enet_packet_destroy (event->packet);

/*	
					// send any data here
					m_packet = enet_packet_create(buffer, sizeof (buffer), 0);
					enet_host_broadcast(server, 1, (ENetPacket*) m_packet);
					enet_host_flush(server);
*/
				} else {
					dAssert (0);
/*
					for (i=0; ipeerCount; i++) {
						if (&server->peers[i] != event->peer) {
							sprintf(buffer, "%s: %s", 
								(char*) event->peer->data, (char*) event->packet->data);
							packet = enet_packet_create(buffer, strlen(buffer)+1, 0);
							enet_peer_send(&server->peers[i], 0, packet);
							enet_host_flush(server);
						} else {

						}
					}
*/
				}
				break;

			case ENET_EVENT_TYPE_DISCONNECT:
				m_connected = false;
/*
				sprintf(buffer, "%s has disconnected.", (char*) event->peer->data);
				packet = enet_packet_create(buffer, strlen(buffer)+1, 0);
				enet_host_broadcast(server, 1, packet);
				free(event->peer->data);
				event->peer->data = NULL;
*/
				break;

			default:
				dAssert (0);
				//printf("Tick tock.\n");
				break;

		}
	}

	if (m_connected) {
		ENetPacket* const packet = enet_packet_create (buffer, size, ENET_PACKET_FLAG_RELIABLE | ENET_PACKET_FLAG_NO_ALLOCATE);
		enet_peer_send(&server->peers[0], 0, packet);
	}


	return m_connected;
}
