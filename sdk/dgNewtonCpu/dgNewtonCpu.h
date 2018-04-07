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

#include "dgNewtonCpuStdafx.h"


#ifdef NEWTONCPU_EXPORTS
#define NEWTONCPU_API __declspec(dllexport)
#else
#define NEWTONCPU_API __declspec(dllimport)
#endif

class dgNewtonCpu: public dgWorldPlugin
{
	public:
	dgNewtonCpu(void);
	virtual const char* GetId() const;
	virtual void CalculateJointForces(const dgBodyCluster& cluster, const dgBodyInfo* const bodyArray, const dgJointInfo* const jointArray, dgFloat32 timestep);

	private:
	const dgBodyCluster* m_cluster;
	const dgBodyInfo* m_bodyArray;
	const dgJointInfo* m_jointArray;
	dgFloat32 m_timestep;
};

#ifdef __cplusplus 
extern "C" 
{
	NEWTONCPU_API dgWorldPlugin* GetPlugin(void);
}
#endif

