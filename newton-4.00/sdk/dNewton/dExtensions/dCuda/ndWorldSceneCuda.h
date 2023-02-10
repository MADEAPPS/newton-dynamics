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

#include <ndNewtonStdafx.h>
#include <ndCudaContext.h>
#include <ndCudaSphFluid.h>
#include <ndCudaBodyProxy.h>
#include <ndDynamicsUpdate.h>

class ndCudaContext;

class ndWorldSceneCuda : public ndWorldScene
{
	public:
	ndWorldSceneCuda(const ndWorldScene& src);
	virtual ~ndWorldSceneCuda();

	virtual void Begin();
	virtual void End();
	virtual void Cleanup();
	virtual void PrepareCleanup();
	virtual bool IsValid() const;
	virtual bool IsHighPerformanceCompute() const;
	
	//virtual void ApplyExtForce();
	//virtual void BalanceScene();
	//virtual void InitBodyArray();
	//virtual void UpdateBodyList();
	//virtual void CalculateContacts();
	//virtual void FindCollidingPairs();
	//
	////virtual void FindCollidingPairs(ndBodyKinematic* const body);
	//virtual void CalculateContacts(ndInt32 threadIndex, ndContact* const contact);
	
	//void LoadBodyData();
	//void GetBodyTransforms();
	//ndCudaContext* GetContext();

	virtual void UpdateTransform();

	virtual void ParticleUpdate(ndFloat32 timestep);
	virtual bool AddParticle(ndSharedPtr<ndBody>& particle);
	virtual bool RemoveParticle(ndSharedPtr<ndBody>& particle);


	//ndArray<ndCudaBodyProxy> m_bodyBufferCpu;
	//
	//friend class ndDynamicsUpdateCuda;

	private:
	void Addparticle(ndBodySphFluid* const fluid);

	ndCudaContext* m_context;
	ndSpecialList<ndCudaSphFluid> m_fluidParticles;

	friend class ndDynamicsUpdateCuda;
};
