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


#include "dgAMP.h"
#include "dgAmpInstance.h"

#define DG_BODY_START_ARRAY_SIZE	256

dgAmpInstance::dgBodySoA::dgBodySoA ()
	:m_bodyInvMass(DG_BODY_START_ARRAY_SIZE)
	,m_bodyDamp(DG_BODY_START_ARRAY_SIZE)
	,m_bodyVelocity(DG_BODY_START_ARRAY_SIZE)
	,m_bodyNetForce(DG_BODY_START_ARRAY_SIZE)
	,m_bodyInternalForce(DG_BODY_START_ARRAY_SIZE)
	,m_bodyMatrix(DG_BODY_START_ARRAY_SIZE)
	,m_bodyInvInertiaMatrix(DG_BODY_START_ARRAY_SIZE)
	,m_bodyDampCpu()
//	,m_bodyDamp_view(m_bodyDamp)
//	,m_bodyInvMass_view (m_bodyInvMass)
//	,m_bodyVelocity_view(m_bodyVelocity)
//	,m_bodyMatrix_view (m_bodyMatrix)
{
}


void dgAmpInstance::dgBodySoA::Alloc (dgWorld* const world, dgInt32 count)
{
	dgInt32 size = m_bodyMatrix.get_extent().size();
	if (size < count) {
		while (size < count) {
			size *= 2;
		}
		m_bodyInvMass = array<float_4, 1> (size);
		m_bodyDamp = array<Jacobian, 1>(size);
		m_bodyVelocity = array<Jacobian, 1>(size);
		m_bodyNetForce = array<Jacobian, 1>(size);
		m_bodyInternalForce = array<Jacobian, 1>(size);
		m_bodyMatrix = array<Matrix4x4 , 1> (size);
		m_bodyInvInertiaMatrix = array<Matrix4x4 , 1> (size);

		m_bodyDampCpu.resize (size);
	}

//	dgInt32 stride = sizeof (m_bodyDampCpu[0]);
//	world->m_solverRightSideMemory.ExpandCapacityIfNeessesary (count + 4, stride);
//	m_bodyDampCpu = (Jacobian*) &world->m_solverMatrixMemory[0];
//	m_bodyDamp_view = array_view<Jacobian, 1> (m_bodyDamp);
//	m_bodyInvMass_view = array_view<float_4, 1> (m_bodyInvMass);
//	m_bodyMatrix_view = array_view<Matrix4x4, 1> (m_bodyMatrix);
//	m_bodyVelocity_view = array_view<Jacobian, 1> (m_bodyVelocity);
}

dgAmpInstance::dgConstraintSoA::dgConstraintSoA ()
	:m_jacobians(DG_BODY_START_ARRAY_SIZE)
	,m_jacobians_view(m_jacobians)
{
}


void dgAmpInstance::dgConstraintSoA::Alloc (dgInt32 size)
{
	m_jacobians = array<Jacobian, 1> (size);
	m_jacobians_view = array_view<Jacobian, 1> (m_jacobians);
}



dgAmpInstance::dgAmpInstance(dgWorld* const world)
	:m_world (world)
	,m_accelerator()
	,m_acceleratorList(world->GetAllocator())
	,m_bodySOA()
	,m_constraintSOA()
{
	const wchar_t* reference = accelerator::direct3d_ref;
	const wchar_t* cpuAccel = accelerator::cpu_accelerator;

    std::vector<accelerator> accs = accelerator::get_all();
    for (int i = 0; i < int (accs.size()); i++) {
		accelerator &accel = accs[i];
		std::wstring path  (accel.get_device_path());
		if (!((path == reference) || (path == cpuAccel))) { 
			std::wstring desc  (accel.get_description());
			std::string p (path.begin(), path.end() );
			std::string d (desc.begin(), desc.end() );
			dgAcceleratorDescription& info = m_acceleratorList.Append()->GetInfo();
			strncpy (info.m_path, p.c_str(), sizeof (info.m_path));
			strncpy (info.m_description, d.c_str(), sizeof (info.m_description));
		}
    }
}


dgAmpInstance::~dgAmpInstance(void)
{
}

void dgAmpInstance::CleanUp()
{
	m_bodySOA.Alloc (m_world, DG_BODY_START_ARRAY_SIZE);
	m_constraintSOA.Alloc (DG_BODY_START_ARRAY_SIZE);
}

dgInt32 dgAmpInstance::GetPlatformsCount() const
{
	return m_acceleratorList.GetCount();
}


void dgAmpInstance::SelectPlaform(dgInt32 deviceIndex)
{
	m_bodySOA.Alloc (m_world, DG_BODY_START_ARRAY_SIZE);
	m_constraintSOA.Alloc (DG_BODY_START_ARRAY_SIZE);

	int index = 0;
	for (dgList<dgAcceleratorDescription>::dgListNode* node = m_acceleratorList.GetFirst(); node; node = node->GetNext()) {
		if (index == deviceIndex) {
			const dgAcceleratorDescription &accel = node->GetInfo();
			std::string p (accel.m_path);
			std::wstring path (p.begin(), p.end());
			accelerator::set_default (path);  
			m_accelerator = accelerator (path);
			break;
		}
		index ++;
	}
}


void dgAmpInstance::GetVendorString(dgInt32 deviceIndex, char* const name, dgInt32 maxlength) const
{
	name[0] = 0;
	int index = 0;
	for (dgList<dgAcceleratorDescription>::dgListNode* node = m_acceleratorList.GetFirst(); node; node = node->GetNext()) {
		if (index == deviceIndex) {
			const dgAcceleratorDescription &accel = node->GetInfo();
			strncpy (name, accel.m_description, maxlength);
			break;
		}
		index ++;
	}
}
