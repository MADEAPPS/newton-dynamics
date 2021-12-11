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

#include "ndCoreStdafx.h"
#include "ndTree.h"
#include "ndTypes.h"

ndRedBackNode *ndRedBackNode::Minimum () const
{
	ndRedBackNode* ptr = (ndRedBackNode *)this;
	for (; ptr->m_left; ptr = ptr->m_left)
	{
	}
	return ptr;
}

ndRedBackNode *ndRedBackNode::Maximum () const
{
	ndRedBackNode* ptr = (ndRedBackNode *)this;
	for (; ptr->m_right; ptr = ptr->m_right)
	{
	}
	return ptr;
}

ndRedBackNode *ndRedBackNode::Prev () const
{
	if (m_left) 
	{
		return m_left->Maximum ();
	}

	ndRedBackNode* node = (ndRedBackNode *)this;
	ndRedBackNode* ptr = m_parent;
	for (; ptr && node == ptr->m_left; ptr = ptr->m_parent) 
	{
		node = ptr;
	}
	return ptr;
}

ndRedBackNode *ndRedBackNode::Next () const
{
	if (m_right) 
	{
		return m_right->Minimum ();
	}

	ndRedBackNode* node = (ndRedBackNode *)this;
	ndRedBackNode* ptr = m_parent;
	for (; ptr && node == ptr->m_right; ptr = ptr->m_parent) 
	{
		node = ptr;
	}
	return ptr;
}

// rotate node me to left 
void ndRedBackNode::RotateLeft(ndRedBackNode** const head) 
{
	ndRedBackNode* const me = this;
	ndRedBackNode* const child = me->m_right;
	
	//establish me->m_right link 
	me->m_right = child->m_left;
	if (child->m_left != nullptr) 
	{
		child->m_left->m_parent = me;
	}
	
	// establish child->m_parent link 
	if (child != nullptr) 
	{
		child->m_parent = me->m_parent;
	}
	if (me->m_parent) 
	{
		if (me == me->m_parent->m_left) 
		{
			me->m_parent->m_left = child;
		} 
		else 
		{
			me->m_parent->m_right = child;
		}
	} 
	else 
	{
		*head = child;
	}
	
	// link child and me 
	dAssert (child);
	child->m_left = me;
	if (me != nullptr) 
	{
		me->m_parent = child;
	}
}

// rotate node me to right  *
void ndRedBackNode::RotateRight(ndRedBackNode ** const head) 
{
	ndRedBackNode* const me = this;
	ndRedBackNode* const child = me->m_left;

	// establish me->m_left link 
	me->m_left = child->m_right;
	if (child->m_right != nullptr) 
	{
		child->m_right->m_parent = me;
	}

	// establish child->m_parent link 
	if (child != nullptr) 
	{
		child->m_parent = me->m_parent;
	}
	if (me->m_parent) 
	{
		if (me == me->m_parent->m_right) 
		{
			me->m_parent->m_right = child;
		} 
		else 
		{
			me->m_parent->m_left = child;
		}
	} 
	else 
	{
		*head = child;
	}

	// link me and child 
	dAssert (child);
	child->m_right = me;
	if (me != nullptr) 
	{
		me->m_parent = child;
	}
}


// maintain Red-Black tree balance after inserting node ptr   
void ndRedBackNode::InsertFixup(ndRedBackNode ** const head) 
{
	ndRedBackNode* ptr = this;
	// check Red-Black properties 
	while ((ptr != *head) && (ptr->m_parent->GetColor() == RED)) 
	{
		// we have a violation 
		dAssert (ptr->m_parent);
		dAssert (ptr->m_parent->m_parent);
		if (ptr->m_parent == ptr->m_parent->m_parent->m_left) 
		{
			ndRedBackNode* const tmp = ptr->m_parent->m_parent->m_right;
			if (tmp && (tmp->GetColor() == RED)) 
			{
				// uncle is RED 
				ptr->m_parent->SetColor(BLACK);
				tmp->SetColor(BLACK) ;
				ptr->m_parent->m_parent->SetColor(RED) ;
				ptr = ptr->m_parent->m_parent;
			} 
			else 
			{
				// uncle is BLACK 
				if (ptr == ptr->m_parent->m_right) 
				{
					// make ptr a left child 
					ptr = ptr->m_parent;
					ptr->RotateLeft(head);
				}

				ptr->m_parent->SetColor(BLACK);
				if (ptr->m_parent->m_parent) 
				{
					ptr->m_parent->m_parent->SetColor(RED);
					ptr->m_parent->m_parent->RotateRight(head);
				}
			}
		} 
		else 
		{
			dAssert (ptr->m_parent == ptr->m_parent->m_parent->m_right);
			// mirror image of above code 
			ndRedBackNode* const tmp = ptr->m_parent->m_parent->m_left;
			if (tmp && (tmp->GetColor() == RED)) 
			{
				//uncle is RED 
				ptr->m_parent->SetColor(BLACK);
				tmp->SetColor(BLACK) ;
				ptr->m_parent->m_parent->SetColor(RED) ;
				ptr = ptr->m_parent->m_parent;
			} 
			else 
			{
				// uncle is BLACK 
				if (ptr == ptr->m_parent->m_left) 
				{
					ptr = ptr->m_parent;
					ptr->RotateRight(head);
				}
				ptr->m_parent->SetColor(BLACK);
				if (ptr->m_parent->m_parent->GetColor() == BLACK) 
				{
					ptr->m_parent->m_parent->SetColor(RED) ;
					ptr->m_parent->m_parent->RotateLeft (head); 
				}
			}
		}
	}
	(*head)->SetColor(BLACK);
}

//maintain Red-Black tree balance after deleting node x 
void ndRedBackNode::RemoveFixup (ndRedBackNode* const thisNode, ndRedBackNode ** const head) 
{
	ndRedBackNode* ptr = this;
	ndRedBackNode* node = thisNode;
	while ((node != *head) && (!node || node->GetColor() == BLACK)) 
	{
		if (node == ptr->m_left) 
		{
			if (!ptr) 
			{
				return;
			}
			ndRedBackNode* tmp = ptr->m_right;
			if (!tmp) 
			{
				return;
			}
			if (tmp->GetColor() == RED) 
			{
				tmp->SetColor(BLACK) ;
				ptr->SetColor(RED) ;
				ptr->RotateLeft (head);
				tmp = ptr->m_right;
				if (!tmp) 
				{
					return;
				}
			}
			if ((!tmp->m_left  || (tmp->m_left->GetColor() == BLACK)) && 
				 (!tmp->m_right || (tmp->m_right->GetColor() == BLACK))) 
			{
				tmp->SetColor(RED);
				node = ptr;
				ptr = ptr->m_parent;
				continue;
			} 
			else if (!tmp->m_right || (tmp->m_right->GetColor() == BLACK)) 
			{
				tmp->m_left->SetColor(BLACK);
				tmp->SetColor(RED);
				tmp->RotateRight (head);
				tmp = ptr->m_right;
				if (!tmp) 
				{
					return;
				}
			}
			tmp->SetColor (ptr->GetColor());
			if (tmp->m_right) 
			{
				tmp->m_right->SetColor(BLACK) ;
			}
			if (ptr) 
			{
				ptr->SetColor(BLACK) ;
				ptr->RotateLeft (head);
			}
			node = *head;

		} 
		else 
		{
		  	if (!ptr) 
			{
				return;
		  	}
			ndRedBackNode* tmp = ptr->m_left;
			if (!tmp) 
			{
				return;
			}
			if (tmp->GetColor() == RED) 
			{
				tmp->SetColor(BLACK) ;
				ptr->SetColor(RED) ;
				ptr->RotateRight (head);
				tmp = ptr->m_left;
				if (!tmp) 
				{
					return;
				}
			}

			if ((!tmp->m_right || (tmp->m_right->GetColor() == BLACK)) && 
				 (!tmp->m_left  || (tmp->m_left->GetColor() == BLACK))) 
			{
				tmp->SetColor(RED) ;
				node = ptr;
				ptr = ptr->m_parent;
				continue;
			} 
			else if (!tmp->m_left || (tmp->m_left->GetColor() == BLACK)) 
			{
				tmp->m_right->SetColor(BLACK) ;
				tmp->SetColor(RED) ;
				tmp->RotateLeft (head);
				tmp = ptr->m_left;
				if (!tmp) 
				{
					return;
				}
			}
			tmp->SetColor (ptr->GetColor());
			if (tmp->m_left) 
			{
				tmp->m_left->SetColor(BLACK);
			}
			if (ptr) 
			{
				ptr->SetColor(BLACK) ;
				ptr->RotateRight (head);
			}
			node = *head;
		}
	}
	if (node) 
	{
		node->SetColor(BLACK);
	}
}

void ndRedBackNode::Unlink (ndRedBackNode ** const head) 
{
	ndRedBackNode* const node = this;
	node->SetInTreeFlag(false);

	if (!node->m_left || !node->m_right) 
	{
		// y has a nullptr node as a child 
		ndRedBackNode* const endNode = node;

		// x is y's only child 
		ndRedBackNode* child = endNode->m_right;
		if (endNode->m_left) 
		{
			child = endNode->m_left;
		}

		// remove y from the parent chain 
		if (child) 
		{
			child->m_parent = endNode->m_parent;
		}

		if (endNode->m_parent)
		{
			if (endNode == endNode->m_parent->m_left) 
			{
				endNode->m_parent->m_left = child;
			} 
			else 
			{
				endNode->m_parent->m_right = child;
			}
		} 
		else 
		{
			*head = child;
		}

		if (endNode->GetColor() == BLACK) 
		{
			endNode->m_parent->RemoveFixup (child, head);
		}
	} 
	else 
	{
		// find tree successor with a nullptr node as a child 
		ndRedBackNode* endNode = node->m_right;
		while (endNode->m_left != nullptr) {
			endNode = endNode->m_left;
		}

		dAssert (endNode);
		dAssert (endNode->m_parent);
		dAssert (!endNode->m_left);

		// x is y's only child 
		ndRedBackNode* const child = endNode->m_right;

		dAssert ((endNode != node->m_right) || !child || (child->m_parent == endNode));

		endNode->m_left = node->m_left;
		node->m_left->m_parent = endNode;

		ndRedBackNode* endNodeParent = endNode;
		if (endNode != node->m_right) 
		{
			if (child) 
			{
				child->m_parent = endNode->m_parent;
			}
			endNode->m_parent->m_left = child;
			endNode->m_right = node->m_right;
			node->m_right->m_parent = endNode;
			endNodeParent = endNode->m_parent;
		}

		if (node == *head) 
		{
			*head = endNode;
		} 
		else if (node == node->m_parent->m_left) 
		{
			node->m_parent->m_left = endNode;
		} 
		else 
		{
			node->m_parent->m_right = endNode;
		}
		endNode->m_parent = node->m_parent;

		ndRedBackNode::REDBLACK_COLOR oldColor = endNode->GetColor();
		endNode->SetColor (node->GetColor());
		node->SetColor	(oldColor);

		if (oldColor == BLACK) 
		{
			endNodeParent->RemoveFixup (child, head);
		}
	}
}

void ndRedBackNode::Remove (ndRedBackNode ** const head) 
{
	Unlink (head);
	delete this;	
}

void ndRedBackNode::RemoveAllLow ()
{
	if (m_left) 
	{
		m_left->RemoveAllLow();
	}
	if (m_right) {
		m_right->RemoveAllLow ();
	}
	SetInTreeFlag(false);
	delete this;
}

void ndRedBackNode::RemoveAll ()
{
	ndRedBackNode* root = this;
	for (; root->m_parent; root = root->m_parent);
	root->RemoveAllLow();
}
