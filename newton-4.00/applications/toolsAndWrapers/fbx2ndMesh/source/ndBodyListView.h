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

class ndBody;
class ndBodyKinematic;

template<class T>
class ndSpecialList : public ndList<T*, ndContainersFreeListAlloc<T*>>
{
	public:
	ndSpecialList();
};

template<class T>
class ndSharedList : public ndList<ndSharedPtr<T>, ndContainersFreeListAlloc<ndSharedPtr<T>*>>
{
	public:
	ndSharedList();
};

//class ndBodyList : public ndList<ndSharedPtr<ndBody>, ndContainersFreeListAlloc<ndSharedPtr<ndBody>*>>
class ndBodyList : public ndSharedList<ndBody>
{
	public:
	ndBodyList();
};

class ndBodyListView: public ndList<ndSharedPtr<ndBody>, ndContainersFreeListAlloc<ndSharedPtr<ndBody>*>>
{
	public:
	ndBodyListView();
	ndBodyListView(const ndBodyListView& src);

	ndArray<ndBodyKinematic*>& GetView();
	const ndArray<ndBodyKinematic*>& GetView() const;

	bool UpdateView();
	bool IsListDirty() const;
	void RemoveItem(ndNode* const node);
	ndNode* AddItem(ndSharedPtr<ndBody> item);

	ndArray<ndBodyKinematic*> m_view;
	ndUnsigned8 m_listIsDirty;
};

template<class T>
ndSpecialList<T>::ndSpecialList()
	:ndList<T*, ndContainersFreeListAlloc<T*>>()
{
}

template<class T>
ndSharedList<T>::ndSharedList()
	:ndList<ndSharedPtr<T>, ndContainersFreeListAlloc<ndSharedPtr<T>*>>()
{
}

inline ndBodyList::ndBodyList()
	//:ndList<ndSharedPtr<ndBody>, ndContainersFreeListAlloc<ndSharedPtr<ndBody>*>>()
	:ndSharedList<ndBody>()
{
}

inline ndBodyListView::ndBodyListView()
	:ndList<ndSharedPtr<ndBody>, ndContainersFreeListAlloc<ndSharedPtr<ndBody>*>>()
	,m_view(1024)
	,m_listIsDirty(1)
{
}

inline ndArray<ndBodyKinematic*>& ndBodyListView::GetView()
{
	return m_view;
}

inline const ndArray<ndBodyKinematic*>& ndBodyListView::GetView() const
{
	return m_view;
}

inline ndBodyListView::ndNode* ndBodyListView::AddItem(ndSharedPtr<ndBody> item)
{
	m_listIsDirty = 1;
	return Append(item);
}

inline void ndBodyListView::RemoveItem(ndNode* const node)
{
	m_listIsDirty = 1;
	Remove(node);
}

inline bool ndBodyListView::IsListDirty() const
{
	return m_listIsDirty ? true : false;
}
#endif