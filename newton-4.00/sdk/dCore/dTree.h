/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_TREE_H__
#define __D_TREE_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dMemory.h"
#include "dContainersAlloc.h"

// Note: this is a low level class for dTree use only
// unpredictable result will happen if you attempt to manipulate
// any member of this class
class dRedBackNode
{
	public:
	enum REDBLACK_COLOR
	{
		RED = true,
		BLACK = false
	};

	dRedBackNode()
	{
	}

	virtual ~dRedBackNode () 
	{
	}

	void RemoveAllLow ();
	void RotateLeft(dRedBackNode** const head); 
	void RotateRight(dRedBackNode** const head); 
	void RemoveFixup (dRedBackNode* const node, dRedBackNode* * const head); 

	dRedBackNode* GetLeft() const;
	dRedBackNode* GetRight() const;
	dRedBackNode* GetParent() const;

	dRedBackNode (dRedBackNode* const parent);
	inline void Initdata (dRedBackNode* const parent);
	inline void SetColor (REDBLACK_COLOR color);
	REDBLACK_COLOR GetColor () const;
	dUnsigned32 IsInTree () const;
	inline void SetInTreeFlag (dUnsigned32 flag);

	void RemoveAll ();
	dRedBackNode* Prev() const;
	dRedBackNode* Next() const;
	dRedBackNode* Minimum() const;
	dRedBackNode* Maximum() const;
	void Remove (dRedBackNode** const head);
	void Unlink (dRedBackNode** const head);
	void InsertFixup(dRedBackNode** const head); 

	dRedBackNode* m_left;
	dRedBackNode* m_right;
	dRedBackNode* m_parent;
	dUnsigned32  m_color	: 1;
	dUnsigned32  m_inTree	: 1;
};

template<class OBJECT, class KEY, class allocator = dContainersAlloc<OBJECT> >
class dTree 
{
	public:
	class dTreeNode: public allocator, public dRedBackNode
	{
		dTreeNode (
			const OBJECT &info, 
			const KEY &key, 
			dTreeNode* parentNode)
			:allocator()
			,dRedBackNode(parentNode), m_info (info), m_key (key)
		{
//			dAssert ((dUnsigned64 (&m_info) & 0x0f) == 0);
		}

		~dTreeNode () 
		{
		}

		dTreeNode* GetLeft () const
		{
			return (dTreeNode* )dRedBackNode::m_left;
		}

		dTreeNode* GetRight () const
		{
			return (dTreeNode* )dRedBackNode::m_right;
		}

		dTreeNode* GetParent ()
		{
			return (dTreeNode* )dRedBackNode::m_parent;
		}

		void SetLeft (dTreeNode* const node)
		{
			dRedBackNode::m_left = node;
		}

		void SetRight (dTreeNode* const node)
		{
			dRedBackNode::m_right = node;
		}

		void SetParent (dTreeNode* const node)
		{
			dRedBackNode::m_parent = node;
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

		private:
		OBJECT m_info;
		KEY m_key; 
		friend class dTree<OBJECT, KEY, allocator>;
	};

	class Iterator
	{
		public:
		Iterator(const dTree<OBJECT, KEY, allocator> &me)
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

		void Set (dTreeNode* const node)
		{
			m_ptr = node;
		}

		operator dInt32() const 
		{
			return m_ptr != nullptr;
		}

		void operator++ ()
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Next();
		}

		void operator++ (dInt32)
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Next();
		}

		void operator-- () 
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Prev();
		}

		void operator-- (dInt32) 
		{
			dAssert (m_ptr);
			m_ptr = m_ptr->Prev();
		}

		OBJECT &operator* () const 
		{
			return ((dTreeNode*)m_ptr)->GetInfo();
		}

		dTreeNode* GetNode() const
		{
			return (dTreeNode*)m_ptr;
		}

		KEY GetKey () const
		{
			dTreeNode* const tmp = (dTreeNode*)m_ptr;
			return tmp ? tmp->GetKey() : KEY(0);
		}

		private:
		dRedBackNode* m_ptr;
		const dTree* m_tree;

	};

	// ***********************************************************
	// member functions
	// ***********************************************************
	public:
	dTree ();
	~dTree (); 

	operator dInt32() const;
	dInt32 GetCount() const;

	dTreeNode* GetRoot () const;
	dTreeNode* Minimum () const;
	dTreeNode* Maximum () const;

	dTreeNode* Find (KEY key) const;
	dTreeNode* FindGreater (KEY key) const;
	dTreeNode* FindGreaterEqual (KEY key) const;
	dTreeNode* FindLessEqual (KEY key) const;

	dTreeNode* GetNodeFromInfo (OBJECT &info) const;

	dTreeNode* Insert (const OBJECT &element, KEY key, bool& elementWasInTree);
	dTreeNode* Insert (const OBJECT &element, KEY key);
	dTreeNode* Insert (dTreeNode* const node, KEY key);

	dTreeNode* Replace (OBJECT &element, KEY key);
	dTreeNode* ReplaceKey (KEY oldKey, KEY newKey);
	dTreeNode* ReplaceKey (dTreeNode* const node, KEY key);

	void Remove (KEY key);
	void Remove (dTreeNode* const node);
	void RemoveAll (); 

	void Unlink (dTreeNode* const node);
	void SwapInfo (dTree& tree);

	bool SanityCheck () const;

	static void FlushFreeList()
	{
		allocator::FlushFreeList();
	}


	// ***********************************************************
	// member variables
	// ***********************************************************
	private:
	dTreeNode* m_head;
	dInt32 m_count;

	dInt32 CompareKeys (const KEY &key0, const KEY &key1) const;
	bool SanityCheck (dTreeNode* const ptr, dInt32 height) const;

	friend class dTreeNode;
};

inline dRedBackNode::dRedBackNode (dRedBackNode* const parent)
{
	Initdata (parent);
}

inline void dRedBackNode::Initdata (dRedBackNode* const parent)
{
	SetColor (RED);
	SetInTreeFlag (true);
	m_left = nullptr;
	m_right = nullptr;
	m_parent = parent;
}

inline void dRedBackNode::SetColor (dRedBackNode::REDBLACK_COLOR color)
{
	m_color = color;
}

inline dRedBackNode::REDBLACK_COLOR  dRedBackNode::GetColor () const
{
	return REDBLACK_COLOR (m_color);
}

inline void dRedBackNode::SetInTreeFlag (dUnsigned32 flag)
{
	m_inTree = flag;
}

inline dUnsigned32 dRedBackNode::IsInTree () const
{
	return m_inTree;
}

template<class OBJECT, class KEY, class allocator>
dTree<OBJECT, KEY, allocator>::dTree ()
	:m_head(nullptr)
	,m_count(0)
{
}

template<class OBJECT, class KEY, class allocator>
dTree<OBJECT, KEY, allocator>::~dTree () 
{
	RemoveAll();
}

template<class OBJECT, class KEY, class allocator>
dTree<OBJECT, KEY, allocator>::operator dInt32() const
{
	return m_head != nullptr;
}

template<class OBJECT, class KEY, class allocator>
dInt32 dTree<OBJECT, KEY, allocator>::GetCount() const
{
	return m_count;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::Minimum () const
{
	return m_head ? (dTreeNode* )m_head->Minimum() : nullptr;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::Maximum () const
{
	return m_head ? (dTreeNode* )m_head->Maximum() : nullptr;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::GetRoot () const
{
	return m_head;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::Find (KEY key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	dTreeNode* ptr = m_head;
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
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::GetNodeFromInfo (OBJECT &info) const
{
	dTreeNode* const node = (dTreeNode* ) &info;
	dInt64 offset = ((char*) &node->m_info) - ((char *) node);
	dTreeNode* const retnode = (dTreeNode* ) (((char *) node) - offset);

	dAssert (retnode->IsInTree ());
	dAssert (&retnode->GetInfo () == &info);
	return (retnode->IsInTree ()) ? retnode : nullptr;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::FindGreater (KEY key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	dTreeNode* prev = nullptr;
	dTreeNode* ptr = m_head;

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

	return (dTreeNode* )prev; 
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::FindGreaterEqual (KEY key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	dTreeNode* prev = nullptr;
	dTreeNode* ptr = m_head;
	
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

	return (dTreeNode* )prev; 
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::FindLessEqual (KEY key) const
{
	if (m_head == nullptr) 
	{
		return nullptr;
	}

	dTreeNode* prev = nullptr;
	dTreeNode* ptr = m_head;

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

	return (dTreeNode* )prev; 
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::Insert (const OBJECT &element, KEY key, bool& elementWasInTree)
{
	dTreeNode* parent = nullptr;
	dTreeNode* ptr = m_head;
	dInt32 val = 0;
	elementWasInTree = false;
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
			elementWasInTree = true;
			return ptr;
		}
	}

	m_count	++;
	ptr = new dTreeNode (element, key, parent);
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

	dTreeNode** const headPtr = (dTreeNode**) &m_head;
	ptr->InsertFixup ((dRedBackNode**)headPtr);
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::Insert (const OBJECT &element, KEY key)
{
	bool foundState;

	dTreeNode* const node = Insert (element, key, foundState);
	if (foundState) 
	{
		return nullptr;
	}
	return node;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::Insert (typename dTree<OBJECT, KEY, allocator>::dTreeNode* const node, KEY key)
{
	dInt32 val = 0;
	dTreeNode* ptr = m_head;
	dTreeNode* parent = nullptr;
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

	dTreeNode** const headPtr = (dTreeNode**) &m_head;
	ptr->InsertFixup ((dRedBackNode**)headPtr);
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::Replace (OBJECT &element, KEY key)
{
	dTreeNode* parent = nullptr;
	dTreeNode* ptr = m_head;
	dInt32 val = 0;

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

	dAssert (m_allocator);
	ptr = new dTreeNode (element, key, parent);
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

	dTreeNode** const headPtr = (dTreeNode**) &m_head;
	ptr->InsertFixup ((dRedBackNode**)headPtr );
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::ReplaceKey (typename dTree<OBJECT, KEY, allocator>::dTreeNode* const node, KEY key)
{
	Unlink (node);
	dTreeNode* const ptr = Insert (node, key);
	dAssert (ptr);
	return ptr;
}

template<class OBJECT, class KEY, class allocator>
typename dTree<OBJECT, KEY, allocator>::dTreeNode* dTree<OBJECT, KEY, allocator>::ReplaceKey (KEY oldKey, KEY newKey)
{
	dTreeNode* const node = Find (oldKey);
	return node ? ReplaceKey (node, newKey) : nullptr;
}

template<class OBJECT, class KEY, class allocator>
void dTree<OBJECT, KEY, allocator>::Unlink (typename dTree<OBJECT, KEY, allocator>::dTreeNode* const node)
{
	m_count	--;

	dTreeNode** const headPtr = (dTreeNode**) &m_head;
	node->Unlink ((dRedBackNode**)headPtr);
	dAssert (!Find (node->GetKey()));
}

template<class OBJECT, class KEY, class allocator>
void dTree<OBJECT, KEY, allocator>::Remove (typename dTree<OBJECT, KEY, allocator>::dTreeNode* const node)
{
	m_count	--;
	dTreeNode** const headPtr = (dTreeNode**) &m_head;
	node->Remove ((dRedBackNode**)headPtr);
}

template<class OBJECT, class KEY, class allocator>
void dTree<OBJECT, KEY, allocator>::Remove (KEY key) 
{
	// find node in tree 
	dTreeNode* const node = Find (key);
	if (node) 
	{
		Remove(node);
	}
}

template<class OBJECT, class KEY, class allocator>
void dTree<OBJECT, KEY, allocator>::RemoveAll () 
{
	if (m_head) 
	{
		m_count	 = 0;
		m_head->RemoveAll ();
		m_head = nullptr;
	}
}

template<class OBJECT, class KEY, class allocator>
bool dTree<OBJECT, KEY, allocator>::SanityCheck () const
{
	return SanityCheck (m_head, 0);
}

template<class OBJECT, class KEY, class allocator>
bool dTree<OBJECT, KEY, allocator>::SanityCheck (typename dTree<OBJECT, KEY, allocator>::dTreeNode* const ptr, dInt32 height) const
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

	if (ptr->GetColor() == dTreeNode::BLACK) 
	{
		height ++;
	} 
	else if (!((!ptr->m_left  || (ptr->m_left->GetColor() == dTreeNode::BLACK)) &&
		       (!ptr->m_right || (ptr->m_right->GetColor() == dTreeNode::BLACK)))) 
	{
	  	return false;
	}

	if (!ptr->m_left && !ptr->m_right) 
	{
		dInt32 bh = 0;
		for (dTreeNode* x = ptr; x; x = x->GetParent()) 
		{
	 		if (x->GetColor() == dTreeNode::BLACK) 
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
dInt32 dTree<OBJECT, KEY, allocator>::CompareKeys (const KEY &key0, const KEY &key1) const
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
void dTree<OBJECT, KEY, allocator>::SwapInfo (dTree<OBJECT, KEY, allocator>& tree)
{
	dSwap (m_head, tree.m_head);
	dSwap (m_count, tree.m_count);
}

//template<class OBJECT, class KEY, class allocator> dInt32 dTree<OBJECT, KEY, allocator>::m_size = 0;
//template<class OBJECT, class KEY, class allocator> dgMemoryAllocator* dTree<OBJECT, KEY, allocator>::m_staticAllocator = nullptr;


#endif


