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

#ifndef _D_SERVER__BODY_PROXY_H_
#define _D_SERVER__BODY_PROXY_H_


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



class BodyProxy
{
	public:
	BodyProxy ()
		:m_body(NULL)
	{
	}
	int m_lru;
	NewtonBody* m_body;
};


class ServerBodyProxyMap: public dTree<BodyProxy, int>
{
	public:
	ServerBodyProxyMap();
	~ServerBodyProxyMap();

	void AddNewBodies ();

	void SerializeData (const void* const buffer, int size);
	void BodySerialization (NewtonBody* const body, NewtonSerializeCallback serializeCallback);

//	bool SendBuffer ();
//	void RemoveBodies ();
	

	static void SerializeData (void* const me, const void* const buffer, int size);
	static void BodySerialization (NewtonBody* const body, NewtonSerializeCallback serializeCallback, void* const me);

	int m_lru;
	int m_bodyBufferCapacity;
	NewtonBody** m_bodyBuffer;
	int m_memCount;
	int m_memBufferCapacity;
	char* m_memBuffer;
};


#endif