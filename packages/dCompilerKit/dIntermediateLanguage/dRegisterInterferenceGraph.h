/* Copyright (c) <2009> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _dRegisterInterferenceGraph_H_
#define _dRegisterInterferenceGraph_H_


#include "dCILstdafx.h"
#include "dTreeAdressStmt.h"


class dDataFlowGraph;
class dRegisterInterferenceNode;

class dRegisterInterferenceNodeEdge
{
	public:
	dRegisterInterferenceNodeEdge (dTree<dRegisterInterferenceNode, dString>::dTreeNode* const m_incidentNode)
		:m_twin(NULL)
		,m_incidentNode(m_incidentNode)
		,m_mark (false)
	{
	}
	
	dList<dRegisterInterferenceNodeEdge>::dListNode* m_twin;
	dTree<dRegisterInterferenceNode, dString>::dTreeNode* m_incidentNode;
	bool m_mark;
};

class dRegisterInterferenceNode
{
	public: 
	dRegisterInterferenceNode()
		:m_name()
		,m_interferanceEdge()
		,m_coalescedParent(NULL)
		,m_registerIndex (-1)
		,m_inSet (false)
		,m_isMove(false)
	{
	}

	dString m_name;
	dList<dRegisterInterferenceNodeEdge> m_interferanceEdge;
	dTree<dRegisterInterferenceNode, dString>::dTreeNode* m_coalescedParent;
	int m_registerIndex;
	bool m_inSet;
	bool m_isMove;
};


class dRegisterInterferenceGraph: public dTree<dRegisterInterferenceNode, dString>
{
	public: 
	class dCoalescedNodePair
	{
		public:	
		dCoalescedNodePair (dTreeNode* const nodeA, dTreeNode* const nodeB)
			:m_nodeA (nodeA)
			,m_nodeB (nodeB)
		{
		}
		dTreeNode* m_nodeA;
		dTreeNode* m_nodeB;
	};


	class dVariableSpillPriority
	{
		public:
		dVariableSpillPriority()
			:m_useCount(0)
			,m_loopUseCount(0)
		{

		}

		int m_useCount;
		int m_loopUseCount;
	};

	dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount);
	private:

	void Build();
	int ColorGraph ();
	dTreeNode* GetBestNode();
	void AllocateRegisters ();
	void CoalesceNodes();
	bool CoalesceNodesRule1 (dTreeNode* const nodeA, dTreeNode* const nodeB);
	bool CoalesceNodesRule2 (dTreeNode* const nodeA, dTreeNode* const nodeB);
	void SelectSpillVariableAndReWriteFunction();


	void SortRegisters(int totalRegisters);
	void SortRegistersByFrequency (int totalRegisters);
	void AllocatedSpilledRegister(int totalRegisters);

	void MakeWorkingRegisters(int totalRegisters);
	void RemapRegister(dTreeAdressStmt::dArg& arg, int totalRegisters);

	void SaveRegisterToTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument);
	void LoadRegisterFromTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument);

	void SaveRegisterToTemp(dCIL::dListNode* const node, const dString& reg, const dString& local);
	void LoadRegisterFromTemp(dCIL::dListNode* const node, const dString& reg, const dString& local);

	void LoadSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument);
	void SaveSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument);
		
	int GetRegisterIndex (const dString& varName) const;
	dString GetRegisterName (const dString& varName) const;
	dString MakeSpilledTemporary ();
		
	int FindRegister (int regIndex, int totalRegisters) const;
	static int Compare (const void* p1, const void* p2);


	dList<dCoalescedNodePair> m_coalescedNodes;
    dString m_reg0;
    dString m_reg1;

    dString m_local0;
    dString m_local1;

	dDataFlowGraph* m_flowGraph;
	int m_spillCount;
	int m_registerCount;
	int m_registenFrequency[1024][2];
};


#endif