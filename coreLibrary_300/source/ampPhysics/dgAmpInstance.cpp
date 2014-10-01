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

#define DG_BODY_START_ARRAY_SIZE	1

dgAmpInstance::dgAmpInstance(dgWorld* const world)
	:m_world (world)
	,m_accelerator()
	,m_acceleratorList(world->GetAllocator())
	,m_bodyMatrix(DG_BODY_START_ARRAY_SIZE)
	,m_bodyInvInertiaMatrix(DG_BODY_START_ARRAY_SIZE)
	,m_bodyMatrix_view (m_bodyMatrix)
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

//SelectPlaform(0);
//AddArraysWithFunction();
}


dgAmpInstance::~dgAmpInstance(void)
{
	CleanUp();
}


void dgAmpInstance::CleanUp()
{
	//m_acceleratorList.RemoveAll();
//	if (m_bodyMatrix) {
//		delete m_bodyMatrix;
//		delete m_bodyInvInertiaMatrix;
//	}
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


/*
#include <iostream> 
void dgAmpInstance::AddElements(index<1> idx, array_view<int, 1> sum, array_view<int, 1> a, array_view<int, 1> b) restrict(amp)
{
    sum[idx] = a[idx] + b[idx];
}


void dgAmpInstance::AddArraysWithFunction() 
{
    int aCPP[] = {1, 2, 3, 4, 5};
    int bCPP[] = {6, 7, 8, 9, 10};
    int sumCPP[5] = {0, 0, 0, 0, 0};

    array_view<int, 1> a(5, aCPP);
    array_view<int, 1> b(5, bCPP);
    array_view<int, 1> sum(5, sumCPP);

    parallel_for_each (sum.extent, [=](index<1> idx) restrict(amp)
    {
        AddElements(idx, sum, a, b);
    });

    for (int i = 0; i < 5; i++) {
        std::cout << sum[i] << "\n";
    }
}
*/



