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

dgAmpBodyData::dgAmpBodyData (dgMemoryAllocator* const allocator)
	:m_bodyInvMass(1)
	,m_bodyDamp(1)
	,m_bodyVelocity(1)
	,m_bodyNetForce(1)
	,m_bodyInternalForce(1)
	,m_bodyMatrix(1)
	,m_bodyInvInertiaMatrix(1)
	,m_bodyInvMassCpu(dgAmpAllocator<float_4> (allocator))
	,m_bodyDampCpu(dgAmpAllocator<dgAmpJacobian> (allocator))
	,m_bodyMatrixCpu(dgAmpAllocator<dgAmpJacobian> (allocator))
	,m_bodyVelocityCpu(dgAmpAllocator<dgAmpJacobian> (allocator))
{
}


void dgAmpBodyData::Alloc (dgInt32 count)
{
	dgInt32 size = m_bodyMatrix.get_extent().size();
	if (size < count) {
		while (size < count) {
			size *= 2;
		}
		m_bodyInvMass = array<float_4, 1> (size);
		m_bodyDamp = array<dgAmpJacobian, 1>(size);
		m_bodyVelocity = array<dgAmpJacobian, 1>(size);
		m_bodyNetForce = array<dgAmpJacobian, 1>(size);
		m_bodyInternalForce = array<dgAmpJacobian, 1>(size);
		m_bodyMatrix = array<dgAmpMatrix4x4 , 1> (size);
		m_bodyInvInertiaMatrix = array<dgAmpMatrix4x4 , 1> (size);
		
		m_bodyDampCpu.resize (size);
		m_bodyMatrixCpu.resize (size);
		m_bodyInvMassCpu.resize (size);
		m_bodyVelocityCpu.resize (size);
	}
}

dgAmpConstraintData::dgAmpConstraintData (dgMemoryAllocator* const allocator)
	:m_dgAmpJacobians(DG_BODY_START_ARRAY_SIZE)
	,m_dgAmpJacobians_view(m_dgAmpJacobians)
{
}


void dgAmpConstraintData::Alloc (dgInt32 size)
{
	m_dgAmpJacobians = array<dgAmpJacobian, 1> (size);
	m_dgAmpJacobians_view = array_view<dgAmpJacobian, 1> (m_dgAmpJacobians);
}



dgAmpInstance::dgAmpInstance(dgWorld* const world)
	:dgAmpBodyData(world->GetAllocator())
	,dgAmpConstraintData(world->GetAllocator())
	,m_world (world)
	,m_accelerator()
	,m_acceleratorList(world->GetAllocator())
{
	const wchar_t* const reference = accelerator::direct3d_ref;
	const wchar_t* const cpuAccel = accelerator::cpu_accelerator;

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
}

dgInt32 dgAmpInstance::GetPlatformsCount() const
{
	return m_acceleratorList.GetCount();
}


void dgAmpInstance::SelectPlaform(dgInt32 deviceIndex)
{
	dgAmpBodyData::Alloc (DG_BODY_START_ARRAY_SIZE);
	dgAmpConstraintData::Alloc (DG_BODY_START_ARRAY_SIZE);

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
