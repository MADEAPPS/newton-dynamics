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


dgAmpBodyData::dgAmpBodyData (dgMemoryAllocator* const allocator)
	:m_currentSize(1)
	,m_bodyInvMass(m_currentSize)
	,m_bodyDamp(m_currentSize)
	,m_bodyVelocity(m_currentSize)
	,m_bodyNetForce(m_currentSize)
	,m_bodyInternalForce(m_currentSize)
	,m_bodyMatrix(m_currentSize)
	,m_bodyInvInertiaMatrix(m_currentSize)
	,m_bodyInvMassCpu(m_bodyInvMass)
	,m_bodyDampCpu(m_bodyDamp)
	,m_bodyMatrixCpu(m_bodyMatrix)
	,m_bodyVelocityCpu(m_bodyVelocity)
{
}

void dgAmpBodyData::CLean ()
{
	m_currentSize = 1;
	Alloc (2);

	//m_bodyInvMassCpu.shrink_to_fit();
	//m_bodyDampCpu.shrink_to_fit();
	//m_bodyMatrixCpu.shrink_to_fit();
	//m_bodyVelocityCpu.shrink_to_fit();
}

void dgAmpBodyData::Alloc (dgInt32 count)
{
	if (m_currentSize < count) {
		while (m_currentSize < count) {
			m_currentSize *= 2;
		}
		m_bodyInvMass = array<float_4, 1> (m_currentSize);
		m_bodyDamp = array<dgAmpJacobian, 1>(m_currentSize);
		m_bodyVelocity = array<dgAmpJacobian, 1>(m_currentSize);
		m_bodyNetForce = array<dgAmpJacobian, 1>(m_currentSize);
		m_bodyInternalForce = array<dgAmpJacobian, 1>(m_currentSize);
		m_bodyMatrix = array<dgAmpMatrix4x4, 1> (m_currentSize);
		m_bodyInvInertiaMatrix = array<dgAmpMatrix4x4 , 1> (m_currentSize);
		
		//m_bodyDampCpu.resize (m_currentSize);
		//m_bodyMatrixCpu.resize (m_currentSize);
		//m_bodyInvMassCpu.resize (m_currentSize);
		//m_bodyVelocityCpu.resize (m_currentSize);

		m_bodyDampCpu = array_view<dgAmpJacobian, 1>(m_bodyDamp);
		m_bodyMatrixCpu = array_view<dgAmpMatrix4x4, 1>(m_bodyMatrix);
		m_bodyInvMassCpu = array_view<float_4, 1>(m_bodyInvMass);
		m_bodyVelocityCpu = array_view<dgAmpJacobian, 1>(m_bodyVelocity);
	}
}

dgAmpConstraintData::dgAmpConstraintData (dgMemoryAllocator* const allocator)
	:m_currentSize(1)
	,m_matrixData(m_currentSize)
//	,m_matrixDataCpu(dgAmpAllocator<dgAmpJacobianMatrixElement> (allocator))
	,m_matrixDataCpu(m_matrixData)
{
	dgAssert ((sizeof (dgAmpJacobianMatrixElement) & 0x0f) == 0);
}

void dgAmpConstraintData::CLean ()
{
	m_currentSize = 1;
	Alloc (2);
//	m_matrixDataCpu.shrink_to_fit();
}

void dgAmpConstraintData::Alloc (dgInt32 count)
{
	if (m_currentSize < count) {
		while (m_currentSize < count) {
			m_currentSize *= 2;
		}
		m_matrixData = array<dgAmpJacobianMatrixElement, 1> (m_currentSize);

		//m_matrixDataCpu.resize (m_currentSize);
		m_matrixDataCpu = array_view<dgAmpJacobianMatrixElement, 1> (m_matrixData);
	}
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
	dgAmpBodyData::CLean();
	dgAmpConstraintData::CLean();
}

dgInt32 dgAmpInstance::GetPlatformsCount() const
{
	return m_acceleratorList.GetCount();
}


void dgAmpInstance::SelectPlaform(dgInt32 deviceIndex)
{
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
