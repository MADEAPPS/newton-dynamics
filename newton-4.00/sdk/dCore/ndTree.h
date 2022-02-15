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

#ifndef __ND_TREE_H__
#define __ND_TREE_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndMemory.h"
#include "ndClassAlloc.h"
#include "ndContainersAlloc.h"

// Note: this is a low level class for ndTree use only
// unpredictable result will happen if you attempt to manipulate
// any member of this class
class ndRedBackNode
{
	public:
	enum REDBLACK_COLOR
	{
		RED = true,
		BLACK = false
	};

	ndRedBackNode()
	{
	}

	virtual ~ndRedBackNode () 
	{
	}

	D_CORE_API void RemoveAllLow ();
	D_CORE_API void RotateLeft(ndRedBackNode** const head); 
	D_CORE_API void RotateRight(ndRedBackNode** const head); 
	D_CORE_API void RemoveFixup (ndRedBackNode* const node, ndRedBackNode* * const head); 

	D_CORE_API ndRedBackNode (ndRedBackNode* const parent);
	D_CORE_API inline void Initdata (ndRedBackNode* const parent);
	D_CORE_API inline void SetColor (REDBLACK_COLOR color);
	D_CORE_API REDBLACK_COLOR GetColor () const;
	D_CORE_API ndUnsigned32 IsInTree () const;
	D_CORE_API inline void SetInTreeFlag (ndUnsigned32 flag);

	D_CORE_API void RemoveAll ();
	D_CORE_API ndRedBackNode* Prev() const;
	D_CORE_API ndRedBackNode* Next() const;
	D_CORE_API ndRedBackNode* Minimum() const;
	D_CORE_API ndRedBackNode* Maximum() const;
	D_CORE_API void Remove (ndRedBackNode** const head);
	D_CORE_API void Unlink (ndRedBackNode** const head);
	D_CORE_API void InsertFixup(ndRedBackNode** const head); 

	ndRedBackNode* m_left;
	ndRedBackNode* m_right;
	ndRedBackNode* m_parent;
	ndUnsigned32  m_color	: 1;
	ndUnsigned32  m_inTree	: 1;
};

template<class OBJECT, class KEY, class allocator = ndContainersAlloc<OBJECT> >
class ndTree: public ndClassAlloc
{
	public:
	class ndNode: public allocator, public ndRedBackNode
	{
		ndNode(const KEY &key, ndNode* parentNode)
			:allocator()
			,ndRedBackNode(parentNode), m_info(), m_key(key)
		{
		}

		ndNode (const OBJECT &info, const KEY &key, ndNode* parentNode)
			:allocator()
			,ndRedBackNode(parentNode), m_info (info), m_key (key)
		{
		}

		~ndNode () 
		{
		}

		ndNode* GetLeft () const
		{
			return (ndNode* )ndRedBackNode::m_left;
		}

		ndNode* GetRight () const
		{
			return (ndNode* )ndRedBackNode::m_right;
		}

		ndNode* GetParent ()
		{
			return (ndNode* )ndRedBackNode::m_parent;
		}

		void SetLeft (ndNode* const node)
		{
			ndRedBackNode::m_left = node;
		}

		void SetRight (ndNode* const node)
		{
			ndRedBackNode::m_right = node;
		}

		void SetParent (ndNode* const node)
		{
			ndRedBackNode::m_parent = node;
		}

		public:
		const KEY& GetKey() const
		{
			return m_key;
		}

		OBJECT& GetInfo()
		{
			return m_info;
		}

		const OBJECT& GetInfo() const
		{
			return m_info;
		}

		private:
		OBJECT m_info;
		KEY m_key; 
		friend class ndTree<OBJECT, KEY, allocator>;
	};

	class Iterator
	{
		public:
		Iterator(const ndTree<OBJECT, KEY, allocator> &me)
		{
			m_ptr = nullptr;
			m_tree = &me;
		}

		~Iterator()
		{
		}

		void Begin() 
		{
			m_ptr = m_tree->Minimum();
		}

		void End()  
		{
			m_ptr = m_tree->Maximum();
		}

		void Set (ndNode* const node)
		{
			m_ptr = node;
		}

		operator ndInt32() const 
		{
			return m_ptr != nullptr;
		}

		void operator++ ()
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Next();
		}

		void operator++ (ndInt32)
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Next();
		}

		void operator-- () 
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Prev();
		}

		void operator-- (ndInt32) 
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Prev();
		}

		OBJECT &operator* () const 
		{
			return ((ndNode*)m_ptr)->GetInfo();
		}

		ndNode* GetNode() const
		{
			return (ndNode*)m_ptr;
		}

		KEY GetKey () const
		{
			ndNode* const tmp = (ndNode*)m_ptr;
			//return tmp ? tmp->GetKey() : KEY(0);
			return tmp ? tmp->GetKey() : KEY();
		}

		private:
		ndRedBackNode* m_ptr;
		const ndTree* m_tree;

	};

	// ***********************************************************
	// member functions
	// ***********************************************************
	public:
	ndTree ();
	~ndTree (); 

	operator ndInt32() const;
	ndInt32 GetCount() const;

	ndNode* GetRoot () const;
	ndNode* Minimum () const;
	ndNode* Maximum () const;

	ndNode* Find (const KEY& key) const;
	ndNode* FindGreater (const KEY& key) const;
	ndNode* FindLessEqual(const KEY& key) const;
	ndNode* FindGreaterEqual (const KEY& key) const;
	ndNode* FindCreate(const KEY& key, bool& wasFound);

	ndNode* GetNodeFromInfo (OBJECT &info) const;

	ndNode* Insert(const KEY& key);
	ndNode* Insert(ndNode* const node, const KEY& key);
	ndNode* Insert(const OBJECT &element, const KEY& key);
	ndNode* Insert (const OBJECT &element, const KEY& key, bool& wasFound);

	ndNode* Replace (OBJECT &element, const KEY& key);
	ndNode* ReplaceKey (const KEY& oldKey, const KEY& newKey);
	ndNode* ReplaceKey (ndNode* const node, const KEY& key);

	void RemoveAll();
	void Remove (const KEY& key);
	void Remove (ndNode* const node);

	void Unlink (ndNode* const node);
	void SwapInfo (ndTree& tree);

	bool SanityCheck () const;

	static void FlushFreeList()
	{
		allocator::FlushFreeList(sizeof(ndNode));
	}

	// ***********************************************************
	// member variables
	// ***********************************************************
	private:
	ndNode* m_head;
	ndInt32 m_count;

	ndInt32 CompareKeys (const KEY &key0, const KEY &key1) const;
	bool SanityCheck (ndNode* const ptr, ndInt32 height) const;

	friend class ndNode;
};

inline ndRedBackNode::ndRedBackNode (ndRedBackNode* const parent)
{
	Initdata (parent);
}

inline void ndRedBackNode::Initdata (ndRedBackNode* const parent)
{
	SetColor (RED);
	SetInTreeFlag (true);
	m_left = nullptr;
	m_right = nullptr;
	m_parent = parent;
}

inline void ndRedBackNode::SetColor (ndRedBackNode::REDBLACK_COLOR color)
{
	m_color = color;
}

inline ndRedBackNode::REDBLACK_COLOR  ndRedBackNode::GetColor () const
{
	return REDBLACK_COLOR (m_color);
}

inline void ndRedBackNode::SetInTreeFlag (ndUnsigned32 flag)
{
	m_inTree = flag;
}

inline ndUnsigned32 ndRedBackNode::IsInTree () const
{
	return m_inTree;
}

template<class OBJECT, class KEY, class allocator>
ndTree<OBJECT, KEY, allocator>::ndTree ()
	:ndClassAlloc()
	,m_head(nullptr)
	,m_count(0)
{
}

template<class OBJECT, class KEY, class allocator>
ndTree<OBJECT, KEY, allocator>::~ndTree () 
{
	RemoveAll();
}

template<class OBJECT, class KEY, class allocator>
ndTree<OBJECT, KEY, allocator>::operator ndInt32() const
{
	return m_head != nullptr;
}

template<class OBJECT, class KEY, class allocator>
ndInt32 ndTree<OBJECT, KEY, allocator>::GetCount() const
{
	return m_count;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Minimum () const
{
	return m_head ? (ndNode* )m_head->Minimum() : nullptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Maximum () const
{
	return m_head ? (ndNode* )m_head->Maximum() : nullptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::GetRoot () const
{
	return m_head;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Find (const KEY& key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	ndNode* ptr = m_head;
	while (ptr != nullptr) 
	{
		if (key < ptr->m_key) 
		{
			dAssert (CompareKeys (ptr->m_key, key) == -1) ;
			ptr = ptr->GetLeft();
		} 
		else if (key > ptr->m_key) 
		{
			dAssert (CompareKeys (ptr->m_key, key) == 1) ;
			ptr = ptr->GetRight();
		} 
		else 
		{
			dAssert (CompareKeys (ptr->m_key, key) == 0) ;
			break;
		}
	}
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::GetNodeFromInfo (OBJECT &info) const
{
	ndNode* const node = (ndNode* ) &info;
	ndInt64 offset = ((char*) &node->m_info) - ((char *) node);
	ndNode* const retnode = (ndNode* ) (((char *) node) - offset);

	dAssert (retnode->IsInTree ());
	dAssert (&retnode->GetInfo () == &info);
	return (retnode->IsInTree ()) ? retnode : nullptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::FindGreater (const KEY& key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	ndNode* prev = nullptr;
	ndNode* ptr = m_head;

	while (ptr != nullptr) 
	{
		if (key < ptr->m_key) 
		{
			prev = ptr;
			ptr = ptr->GetLeft();
		} 
		else 
		{
			ptr = ptr->GetRight();
		}
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK
	if (prev) 
	{
		Iterator iter (*this);
		for (iter.Begin(); iter.GetNode() != prev; iter ++) 
		{
			KEY key1 = iter.GetKey(); 
			dAssert (key1 <= key);
		}
		for (; iter.GetNode(); iter ++) 
		{
			KEY key1 = iter.GetKey(); 
			dAssert (key1 > key);
		}
	}
#endif

	return (ndNode* )prev; 
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::FindGreaterEqual (const KEY& key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	ndNode* prev = nullptr;
	ndNode* ptr = m_head;
	
	while (ptr != nullptr) 
	{
		if (key == ptr->m_key) 
		{
			return ptr;
		}
		if (key < ptr->m_key) 
		{
			prev = ptr;
			ptr = ptr->GetLeft();
		} 
		else 
		{
			ptr = ptr->GetRight();
		}
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK
	if (prev) 
	{
		Iterator iter (*this);
		for (iter.Begin(); iter.GetNode() != prev; iter ++) 
		{
			KEY key1 = iter.GetKey(); 
			dAssert (key1 <= key);
		}
		for (; iter.GetNode(); iter ++) 
		{
			KEY key1 = iter.GetKey(); 
			dAssert (key1 >= key);
		}
	}
#endif

	return (ndNode* )prev; 
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::FindLessEqual (const KEY& key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	ndNode* prev = nullptr;
	ndNode* ptr = m_head;

	while (ptr != nullptr) 
	{
		if (key == ptr->m_key) 
		{
			return ptr;
		}

		if (key < ptr->m_key) 
		{
			ptr = ptr->GetLeft();
		} 
		else 
		{
			prev = ptr;
			ptr = ptr->GetRight();
		}

	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK
	if (prev) 
	{
		Iterator iter (*this);
		for (iter.End(); iter.GetNode() != prev; iter --) 
		{
			KEY key1 = iter.GetKey(); 
			dAssert (key1 >= key);
		}
		for (; iter.GetNode(); iter --) 
		{
			KEY key1 = iter.GetKey(); 
			dAssert (key1 < key);
		}
	}
#endif

	return (ndNode* )prev; 
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Insert (const OBJECT &element, const KEY& key, bool& wasFound)
{
	ndNode* parent = nullptr;
	ndNode* ptr = m_head;
	ndInt32 val = 0;
	while (ptr != nullptr) 
	{
		parent = ptr;

		if (key < ptr->m_key) 
		{
			dAssert (CompareKeys (ptr->m_key, key) == -1) ;
			val = -1;
			ptr = ptr->GetLeft();
		} 
		else if (key > ptr->m_key) 
		{
			dAssert (CompareKeys (ptr->m_key, key) == 1) ;
			val = 1;
			ptr = ptr->GetRight();
		} 
		else 
		{
			dAssert (CompareKeys (ptr->m_key, key) == 0) ;
			wasFound = true;
			return ptr;
		}
	}

	m_count	++;
	wasFound = false;
	ptr = new ndNode (element, key, parent);
	if (!parent) 
	{
		m_head = ptr;
	} 
	else 
	{
		if (val < 0) 
		{
			parent->m_left = ptr; 
		} 
		else 
		{
			parent->m_right = ptr;
		}
	}

	ndNode** const headPtr = (ndNode**) &m_head;
	ptr->InsertFixup ((ndRedBackNode**)headPtr);
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::FindCreate(const KEY& key, bool& wasFound)
{
	ndNode* parent = nullptr;
	ndNode* ptr = m_head;
	ndInt32 val = 0;
	while (ptr != nullptr)
	{
		parent = ptr;
		if (key < ptr->m_key)
		{
			dAssert(CompareKeys(ptr->m_key, key) == -1);
			val = -1;
			ptr = ptr->GetLeft();
		}
		else if (key > ptr->m_key)
		{
			dAssert(CompareKeys(ptr->m_key, key) == 1);
			val = 1;
			ptr = ptr->GetRight();
		}
		else
		{
			dAssert(CompareKeys(ptr->m_key, key) == 0);
			wasFound = true;
			return ptr;
		}
	}

	m_count++;
	wasFound = false;
	ptr = new ndNode(key, parent);
	if (!parent)
	{
		m_head = ptr;
	}
	else
	{
		if (val < 0)
		{
			parent->m_left = ptr;
		}
		else
		{
			parent->m_right = ptr;
		}
	}

	ndNode** const headPtr = (ndNode**)&m_head;
	ptr->InsertFixup((ndRedBackNode**)headPtr);
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Insert (const OBJECT &element, const KEY& key)
{
	bool foundState;

	ndNode* const node = Insert (element, key, foundState);
	if (foundState) 
	{
		return nullptr;
	}
	return node;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Insert(const KEY& key)
{
	OBJECT element;
	return Insert(element, key);
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Insert (typename ndTree<OBJECT, KEY, allocator>::ndNode* const node, const KEY& key)
{
	ndInt32 val = 0;
	ndNode* ptr = m_head;
	ndNode* parent = nullptr;
	while (ptr != nullptr) 
	{
		parent = ptr;

		if (key < ptr->m_key) 
		{
			dAssert (CompareKeys (ptr->m_key, key) == -1) ;
			val = -1;
			ptr = ptr->GetLeft();
		} 
		else if (key > ptr->m_key) 
		{
			dAssert (CompareKeys (ptr->m_key, key) == 1) ;
			val = 1;
			ptr = ptr->GetRight();
		} 
		else 
		{
			dAssert (CompareKeys (ptr->m_key, key) == 0) ;
			return nullptr;
		}
	}

	m_count	++;

	ptr = node;
	ptr->m_key = key;
	ptr->Initdata (parent);

	if (!parent) 
	{
		m_head = ptr;
	} 
	else 
	{
		if (val < 0) 
		{
			parent->m_left = ptr; 
		} 
		else 
		{
			parent->m_right = ptr;
		}
	}

	ndNode** const headPtr = (ndNode**) &m_head;
	ptr->InsertFixup ((ndRedBackNode**)headPtr);
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::Replace (OBJECT &element, const KEY& key)
{
	ndNode* parent = nullptr;
	ndNode* ptr = m_head;
	ndInt32 val = 0;

	while (ptr != nullptr) 
	{
		parent = ptr;

		dAssert (0);
		val = CompareKeys (ptr->m_key, key);
		if (val == 0) 
		{
			ptr->m_info = element;
			return ptr;
		}
		if (val < 0) 
		{
			ptr = ptr->GetLeft();
		} 
		else 
		{
			ptr = ptr->GetRight();
		}
	}

	ptr = new ndNode (element, key, parent);
	if (!parent) 
	{
		m_head = ptr;
	} 
	else 
	{
		if (val < 0) 
		{
			parent->m_left = ptr; 
		} 
		else 
		{
			parent->m_right = ptr;
		}
	}

	ndNode** const headPtr = (ndNode**) &m_head;
	ptr->InsertFixup ((ndRedBackNode**)headPtr );
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::ReplaceKey (typename ndTree<OBJECT, KEY, allocator>::ndNode* const node, const KEY& key)
{
	Unlink (node);
	ndNode* const ptr = Insert (node, key);
	dAssert (ptr);
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename ndTree<OBJECT, KEY, allocator>::ndNode* ndTree<OBJECT, KEY, allocator>::ReplaceKey (const KEY& oldKey, const KEY& newKey)
{
	ndNode* const node = Find (oldKey);
	return node ? ReplaceKey (node, newKey) : nullptr;
}

template<class OBJECT, class KEY, class allocator>
void ndTree<OBJECT, KEY, allocator>::Unlink (typename ndTree<OBJECT, KEY, allocator>::ndNode* const node)
{
	m_count	--;

	ndNode** const headPtr = (ndNode**) &m_head;
	node->Unlink ((ndRedBackNode**)headPtr);
	dAssert (!Find (node->GetKey()));
}

template<class OBJECT, class KEY, class allocator>
void ndTree<OBJECT, KEY, allocator>::Remove (typename ndTree<OBJECT, KEY, allocator>::ndNode* const node)
{
	m_count	--;
	ndNode** const headPtr = (ndNode**) &m_head;
	node->Remove ((ndRedBackNode**)headPtr);
}

template<class OBJECT, class KEY, class allocator>
void ndTree<OBJECT, KEY, allocator>::Remove (const KEY& key) 
{
	// find node in tree 
	ndNode* const node = Find (key);
	if (node) 
	{
		Remove(node);
	}
}

template<class OBJECT, class KEY, class allocator>
void ndTree<OBJECT, KEY, allocator>::RemoveAll () 
{
	if (m_head) 
	{
		m_count	 = 0;
		m_head->RemoveAll ();
		m_head = nullptr;
	}
}

template<class OBJECT, class KEY, class allocator>
bool ndTree<OBJECT, KEY, allocator>::SanityCheck () const
{
	return SanityCheck (m_head, 0);
}

template<class OBJECT, class KEY, class allocator>
bool ndTree<OBJECT, KEY, allocator>::SanityCheck (typename ndTree<OBJECT, KEY, allocator>::ndNode* const ptr, ndInt32 height) const
{
	if (!ptr) 
	{
		return true;
	}

	if (!ptr->IsInTree()) 
	{
		return false;
	}

	if (ptr->m_left) 
	{
		if (CompareKeys (ptr->m_key, ptr->GetLeft()->m_key) > 0) 
		{
			return false;
		}
	}

	if (ptr->m_right) 
	{
		if (CompareKeys (ptr->m_key, ptr->GetRight()->m_key) < 0) 
		{
			return false;
		}
	}

	if (ptr->GetColor() == ndNode::BLACK) 
	{
		height ++;
	} 
	else if (!((!ptr->m_left  || (ptr->m_left->GetColor() == ndNode::BLACK)) &&
		       (!ptr->m_right || (ptr->m_right->GetColor() == ndNode::BLACK)))) 
	{
	  	return false;
	}

	if (!ptr->m_left && !ptr->m_right) 
	{
		ndInt32 bh = 0;
		for (ndNode* x = ptr; x; x = x->GetParent()) 
		{
	 		if (x->GetColor() == ndNode::BLACK) 
			{
				bh ++;
			}
		}
		if (bh != height) 
		{
			return false;
		}
	}

	if (ptr->m_left && !SanityCheck (ptr->GetLeft(), height)) 
	{
		return false;
	}

	if (ptr->m_right && !SanityCheck (ptr->GetRight(), height)) 
	{
		return false;
	}
	return true;
}

template<class OBJECT, class KEY, class allocator>
ndInt32 ndTree<OBJECT, KEY, allocator>::CompareKeys (const KEY &key0, const KEY &key1) const
{
	if (key1 < key0) 
	{
		return - 1;
	}
	if (key1 > key0) 
	{
		return 1;
	}
	return 0;
}

template<class OBJECT, class KEY, class allocator>
void ndTree<OBJECT, KEY, allocator>::SwapInfo (ndTree<OBJECT, KEY, allocator>& tree)
{
	dSwap (m_head, tree.m_head);
	dSwap (m_count, tree.m_count);
}

//template<class OBJECT, class KEY, class allocator> ndInt32 ndTree<OBJECT, KEY, allocator>::m_size = 0;
//template<class OBJECT, class KEY, class allocator> dgMemoryAllocator* ndTree<OBJECT, KEY, allocator>::m_staticAllocator = nullptr;


#endif


