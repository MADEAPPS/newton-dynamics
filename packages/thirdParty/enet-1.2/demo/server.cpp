#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <enet/enet.h>
#include "config.h"



ENetEvent  event;
ENetAddress  address;

ENetHost   *server;
ENetPacket  *packet;

char    buffer[BUFFERSIZE];

int  main(int argc, char ** argv) 
{
	int  i;

	if (enet_initialize() != 0) {
		printf("Could not initialize enet.");
		return 0;
	}

	address.host = ENET_HOST_ANY;
	address.port = PORT;

	server = enet_host_create(&address, 100, 2, 0, 0);

	if (server == NULL) {
		printf("Could not start server.\n");
		return 0;
	}

	while (1) {
		while (enet_host_service(server, &event, 1000) > 0) {
			switch (event.type) {
				case ENET_EVENT_TYPE_CONNECT:
					break;

				case ENET_EVENT_TYPE_RECEIVE:
					if (event.peer->data == NULL) {
						event.peer->data = malloc(strlen((char*) event.packet->data)+1);
						strcpy((char*) event.peer->data, (char*) event.packet->data);

						sprintf(buffer, "%s has connected\n", (char*) event.packet->data);
						packet = enet_packet_create(buffer, strlen(buffer)+1, 0);
						enet_host_broadcast(server, 1, packet);
						enet_host_flush(server);
					} else {
						for (i=0; ipeerCount; i++) {
							if (&server->peers[i] != event.peer) {
								sprintf(buffer, "%s: %s", 
									(char*) event.peer->data, (char*) event.packet->data);
								packet = enet_packet_create(buffer, strlen(buffer)+1, 0);
								enet_peer_send(&server->peers[i], 0, packet);
								enet_host_flush(server);
							} else {

							}
						}
					}
					break;

				case ENET_EVENT_TYPE_DISCONNECT:
					sprintf(buffer, "%s has disconnected.", (char*) event.peer->data);
					packet = enet_packet_create(buffer, strlen(buffer)+1, 0);
					enet_host_broadcast(server, 1, packet);
					free(event.peer->data);
					event.peer->data = NULL;
					break;

				default:
					printf("Tick tock.\n");
					break;
			}

		}
	}

	enet_host_destroy(server);
	enet_deinitialize();
}
