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

#include "dStdAfxNewton.h"
#include "dMaterialPairManager.h"


dMaterialPairManager::dMaterialPairManager ()
	:dNewtonAlloc()
	,m_cache(-1)
	,m_maxCount(8)
	,m_entryCount(0)
{
	dAssert (0);
	m_pool = new dMaterialPair[m_maxCount];
}

dMaterialPairManager::~dMaterialPairManager ()
{
	dAssert (0);
	delete[] m_pool;
}


void dMaterialPairManager::AddPair (int materialId_0, int materialId_1, const dMaterialPair& pair)
{
	dAssert (0);
	if (m_entryCount >= m_maxCount) {
		// the interaction pool is full we need to reallocate a larger Pool 
		dMaterialPair* const newPool = new dMaterialPair[m_maxCount * 2];
		memcpy (newPool, m_pool, m_entryCount * sizeof (dMaterialPair));
		delete[] m_pool;
		m_pool = newPool;
		m_maxCount *= 2;
	}

	materialId_0 &= 0xff;
	materialId_1 &= 0xff;
	unsigned key = (materialId_1 >= materialId_0) ? (materialId_1 << 8) + materialId_0 : (materialId_0 << 8) + materialId_1;

	int index = m_entryCount;
	for ( ; (index > 0) && m_pool[index-1].m_key > key; index --) {
		m_pool[index] = m_pool[index - 1];
	}
	m_pool[index] = pair;
	m_pool[index].m_key = key;
	m_entryCount ++;
}


dMaterialPairManager::dMaterialPair* dMaterialPairManager::GetPair (int materialId_0, int materialId_1) const
{
	dAssert (0);
	return NULL;
}

