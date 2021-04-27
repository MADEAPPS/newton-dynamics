/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _D_DATA_SET_H_
#define _D_DATA_SET_H_

#include "ndCollisionStdafx.h"

class ndSetData
{
	public:
	ndSetData(dUnsigned32 value);
	ndSetData(const ndSetData& value);
	ndSetData(dUnsigned32 index, dUnsigned32 rank);

	void SetStateSleep0(bool state);
	void SetStateSleep1(bool state);
	void SetConstrainedAndRestingState(bool constraint, bool resting);

	union
	{
		struct
		{
			dUnsigned32	m_rootIndex : 21;
			dUnsigned32	m_rank : 3;
			dUnsigned32	m_hasStaticBody : 1;
			dUnsigned32	m_solverSleep0 : 1;
			dUnsigned32	m_solverSleep1 : 1;
			dUnsigned32	m_resting : 1;
			dUnsigned32	m_bodyIsConstrained : 1;
		};
		dUnsigned32 m_value;
		dAtomic<dUnsigned32> m_transaction;
	};
};

inline ndSetData::ndSetData(dUnsigned32 value)
	:m_value(value)
{
}

inline ndSetData::ndSetData(const ndSetData& value)
	:m_value(value.m_value)
{
}

inline ndSetData::ndSetData(dUnsigned32 index, dUnsigned32 rank)
	:m_value(0)
{
	m_rank = rank;
	m_rootIndex = index;
}

inline void ndSetData::SetStateSleep0(bool state)
{
	ndSetData oldData(m_transaction);
	ndSetData newData(m_transaction);
	oldData.m_solverSleep0 = 1;
	newData.m_solverSleep0 = state;
	m_transaction.compare_exchange_weak(oldData.m_value, newData.m_value);
}

inline void ndSetData::SetStateSleep1(bool state)
{
	ndSetData oldData(m_transaction);
	ndSetData newData(m_transaction);
	oldData.m_solverSleep1 = 1;
	newData.m_solverSleep1 = state;
	m_transaction.compare_exchange_weak(oldData.m_value, newData.m_value);
}

inline void ndSetData::SetConstrainedAndRestingState(bool constrained, bool resting)
{
	ndSetData oldData(m_transaction);
	ndSetData newData(m_transaction);
	oldData.m_resting = 1;
	oldData.m_bodyIsConstrained = 0;
	newData.m_resting = resting;
	newData.m_bodyIsConstrained = constrained;
	m_transaction.compare_exchange_weak(oldData.m_value, newData.m_value);
}

#endif 

