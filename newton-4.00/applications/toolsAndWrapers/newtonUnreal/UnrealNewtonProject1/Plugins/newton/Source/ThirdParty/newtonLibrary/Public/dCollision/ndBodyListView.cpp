/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBodyListView.h"
#include "ndBodyKinematic.h"

ndBodyListView::ndBodyListView(const ndBodyListView& src)
	:ndList<ndSharedPtr<ndBody>, ndContainersFreeListAlloc<ndSharedPtr<ndBody>*>>()
	,m_view(1024)
	,m_listIsDirty(1)
{
	ndNode* nextNode;
	ndBodyListView* const stealData = (ndBodyListView*)&src;
	for (ndNode* node = stealData->GetFirst(); node; node = nextNode)
	{
		nextNode = node->GetNext();
		stealData->Unlink(node);
		Append(node);
	}
	m_view.Swap(stealData->m_view);
}
	
bool ndBodyListView::UpdateView()
{
	bool ret = false;
	if (m_listIsDirty)
	{
		D_TRACKTIME();
		ret = true;
		m_listIsDirty = 0;
		ndInt32 index = 0;
		m_view.SetCount(GetCount());
		for (ndNode* node = GetFirst(); node; node = node->GetNext())
		{
			m_view[index] = node->GetInfo()->GetAsBodyKinematic();
			index++;
		}
	}
	return ret;
}

