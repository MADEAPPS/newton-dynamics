/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_BODY_SPH_FLUID_CPU_H__
#define __ND_BODY_SPH_FLUID_CPU_H__

#include "ndNewtonStdafx.h"
#include "ndBodyParticleSet.h"

D_MSV_NEWTON_ALIGN_32
class ndBodySphFluid: public ndBodyParticleSet
{
	public:
	D_NEWTON_API ndBodySphFluid();
	D_NEWTON_API ndBodySphFluid(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API virtual ~ndBodySphFluid ();

	ndFloat32 GetViscosity() const;
	void SetViscosity(ndFloat32 viscosity);
	
	ndFloat32 GetParticleMass() const;
	void SetParticleMass(ndFloat32 mass);

	ndFloat32 GetRestDensity() const;
	void SetRestDensity(ndFloat32 resDensity);
	
	ndFloat32 GetGasConstant() const;
	void SetGasConstant(ndFloat32 gasConst);

	ndFloat32 GetSphGridSize() const;

	virtual ndBodySphFluid* GetAsBodySphFluid();
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	D_NEWTON_API void Execute();

	protected:
	D_NEWTON_API virtual void Update(const ndWorld* const world, ndFloat32 timestep);
	virtual bool RayCast(ndRayCastNotify& callback, const ndFastRay& ray, const ndFloat32 maxT) const;

	private:
	enum ndGridType
	{
		ndAdjacentGrid = 0,
		ndHomeGrid = 1,
	};

	class ndGridHash;
	class ndWorkingData;
	class ndParticlePair;
	class ndParticleKernelDistance;

	ndWorkingData& WorkingData();
	void Update(ndThreadPool* const threadPool);
	void SortGrids(ndThreadPool* const threadPool);
	void BuildPairs(ndThreadPool* const threadPool);
	void CreateGrids(ndThreadPool* const threadPool);
	void CaculateAabb(ndThreadPool* const threadPool);
	void SortXdimension(ndThreadPool* const threadPool);
	void CalculateScans(ndThreadPool* const threadPool);
	void SortCellBuckects(ndThreadPool* const threadPool);
	void IntegrateParticles(ndThreadPool* const threadPool);
	void CalculateAccelerations(ndThreadPool* const threadPool);
	void CalculateParticlesDensity(ndThreadPool* const threadPool);

	ndFloat32 m_mass;
	ndFloat32 m_viscosity;
	ndFloat32 m_restDensity;
	ndFloat32 m_gasConstant;
	ndFloat32 m_timestep;
} D_GCC_NEWTON_ALIGN_32 ;

inline bool ndBodySphFluid::RayCast(ndRayCastNotify&, const ndFastRay&, const ndFloat32) const
{
	return false;
}

inline ndBodySphFluid* ndBodySphFluid::GetAsBodySphFluid()
{ 
	return this; 
}

inline ndFloat32 ndBodySphFluid::GetViscosity() const
{
	return m_viscosity;
}

inline void ndBodySphFluid::SetViscosity(ndFloat32 viscosity)
{
	m_viscosity = viscosity;
}

inline ndFloat32 ndBodySphFluid::GetParticleMass() const
{
	return m_mass;
}

inline void ndBodySphFluid::SetParticleMass(ndFloat32 mass)
{
	m_mass = mass;
}

inline ndFloat32 ndBodySphFluid::GetRestDensity() const
{
	return m_restDensity;
}

inline void ndBodySphFluid::SetRestDensity(ndFloat32 restDensity)
{
	m_restDensity = restDensity;
}

inline ndFloat32 ndBodySphFluid::GetGasConstant() const
{
	return m_gasConstant;
}

inline void ndBodySphFluid::SetGasConstant(ndFloat32 gasConst)
{
	m_gasConstant = gasConst;
}

inline ndFloat32 ndBodySphFluid::GetSphGridSize() const
{
	return GetParticleRadius() * ndFloat32(2.0f) * ndFloat32(1.5f);
	//return GetParticleRadius() * ndFloat32(2.0f) * ndFloat32(0.75f);
}

#endif 


