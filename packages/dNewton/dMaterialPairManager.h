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

#ifndef __MATERIAL_MANAGER_H__
#define __MATERIAL_MANAGER_H__

#include "dStdAfxNewton.h"
#include "dNewtonAlloc.h"



class dMaterialPairManager: public dNewtonAlloc
{
	public:
	class dMaterialPair
	{
		public:
		dLong  m_userData[8];
		dFloat m_restitution;
		dFloat m_staticFriction;
		dFloat m_kineticFriction;

		private:
		unsigned m_key;

		friend class dMaterialPairManager;
	};

	CNEWTON_API dMaterialPairManager ();
	CNEWTON_API ~dMaterialPairManager ();
	
	CNEWTON_API dMaterialPair* GetPair (int materialId_0, int materialId_1) const;
	CNEWTON_API void AddPair (int materialId_0, int materialId_1, const dMaterialPair& pair);

	private:
	int m_cache;
	int m_maxCount;
	int m_entryCount;
	dMaterialPair* m_pool;
};
#endif
