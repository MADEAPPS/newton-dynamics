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

#ifndef __ND_LIST_VIEW_H__
#define __ND_LIST_VIEW_H__

#include "ndCollisionStdafx.h"

class ndBodyKinematic;

template<class T>
class ndListView : public ndList<T*, ndContainersFreeListAlloc<T*>>
{
	public:
	ndListView();
	ndListView(const ndListView& src);

#ifdef _MSC_VER 
	typename ndNode* AddItem(T* const item);
#else
	typename ndListView<T>::ndNode* AddItem(T* const item);
#endif
	void RemoveItem(typename ndListView<T>::ndNode* const node);

	bool UpdateView();
	bool IsListDirty() const;

	ndArray<T*>& GetView();
	const ndArray<T*>& GetView() const;

	protected:
	ndArray<T*> m_view;
	ndUnsigned8 m_listIsDirty;
};

class ndBodyList: public ndListView<ndBodyKinematic>
{
	public:
	ndBodyList()
		:ndListView<ndBodyKinematic>()
	{
	}

	ndBodyList(const ndBodyList& src)
		:ndListView<ndBodyKinematic>(src)
	{
	}
};

template<class T>
ndListView<T>::ndListView()
	:ndList<T*, ndContainersFreeListAlloc<T*>>()
	,m_view(1024)
	,m_listIsDirty(1)
{
}

template<class T>
ndListView<T>::ndListView(const ndListView& src)
	:ndList<T*, ndContainersFreeListAlloc<T*>>()
	,m_view()
	,m_listIsDirty(1)
{
	typename ndListView<T>::ndNode* nextNode;
	ndListView* const stealData = (ndListView*)&src;
	for (typename ndListView<T>::ndNode* node = stealData->GetFirst(); node; node = nextNode)
	{
		nextNode = node->GetNext();
		stealData->Unlink(node);
		ndListView<T>::Append(node);
	}
	m_view.Swap(stealData->m_view);
}

template<class T>
ndArray<T*>& ndListView<T>::GetView()
{
	return m_view;
}

template<class T>
const ndArray<T*>& ndListView<T>::GetView() const
{
	return m_view;
}

template<class T>
typename ndListView<T>::ndNode* ndListView<T>::AddItem(T* const item)
{
	m_listIsDirty = 1;
	return ndListView<T>::Append(item);
}

template<class T>
void ndListView<T>::RemoveItem(typename ndListView<T>::ndNode* const node)
{
	m_listIsDirty = 1;
	ndListView<T>::Remove(node);
}

template<class T>
bool ndListView<T>::IsListDirty() const
{
	return m_listIsDirty ? true : false;
}

template<class T>
bool ndListView<T>::UpdateView()
{
	bool ret = false;
	if (m_listIsDirty)
	{
		D_TRACKTIME();
		ret = true;
		m_listIsDirty = 0;
		m_view.SetCount(ndListView<T>::GetCount());
		ndInt32 index = 0;
		for (typename ndListView<T>::ndNode* node = ndListView<T>::GetFirst(); node; node = node->GetNext())
		{
			m_view[index] = node->GetInfo();
			index++;
		}
	}
	return ret;
}

#endif