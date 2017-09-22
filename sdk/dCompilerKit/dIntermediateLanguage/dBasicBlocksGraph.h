/* Copyright (c) <2003-2016> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _dBasicBlockList_H_
#define _dBasicBlockList_H_

#include "dCILstdafx.h"
#include "dCIL.h"
#include "dCILInstr.h"

inline int RegisterToIndex(const dString& reg)
{
	return dString(reg.GetStr() + strlen(D_REGISTER_SYMBOL)).ToInteger();
}

inline dString IndexToRegister(int index)
{
	char regName[256];
	sprintf(regName, "%s%d", D_REGISTER_SYMBOL, index);
	return regName;
}


template <class SET_TYPE>
class dVariableSet: public dTree<int, SET_TYPE>
{
	public:
	dVariableSet()
		:dTree<int, SET_TYPE>()
	{
	}

	dVariableSet(const dVariableSet& copy)
		:dTree<int, SET_TYPE>()
	{
		Iterator iter(copy);
		for (iter.Begin(); iter; iter++) {
			Insert(0, iter.GetKey());
		}
	}

	void Difference(const dVariableSet& b, const dVariableSet& c)
	{
		Iterator iter0(b);
		for (iter0.Begin(); iter0; iter0++) {
			Insert(0, iter0.GetKey());
		}

		Iterator iter1(c);
		for (iter1.Begin(); iter1; iter1++) {
			dTreeNode* const node = Find(iter1.GetKey());
			if (node) {
				Remove(node);
			}
		}
	}

	void Union(const dVariableSet& or)
	{
		Iterator iter(or);
		for (iter.Begin(); iter; iter++) {
			Insert(0, iter.GetKey());
		}
	}

	void Union(const dList<dCILInstr::dArg*>& or)
	{
		for (dList<dCILInstr::dArg*>::dListNode* node = or.GetFirst(); node; node = node->GetNext()) {
			Insert(0, node->GetInfo()->m_label);
		}
	}

	bool Compare(const dVariableSet& cmp) const
	{
		if (GetCount() == cmp.GetCount()) {
			Iterator iter0(cmp);
			Iterator iter1(*this);
			for (iter0.Begin(), iter1.Begin(); iter0 && iter1; iter0++, iter1++) {
				if (iter0.GetKey() != iter1.GetKey()) {
					return false;
				}
			}
			return true;
		}
		return false;
	}

	void Trace() const
	{
		Iterator iter(*this);
		for (iter.Begin(); iter; iter++) {
			dTrace(("%s ", iter.GetKey().GetStr()));
		}
		dTrace(("\n"));
	}
};


class dBasicBlock
{
	public:
	dBasicBlock (dCIL::dListNode* const begin);
	dBasicBlock (const dBasicBlock& src);
	void Trace() const;

	void ReplaceDominator(const dTree<int, const dBasicBlock*>& dominators);
	bool ComparedDominator(const dTree<int, const dBasicBlock*>& dominators) const;
	
	dCIL::dListNode* m_begin;
	dCIL::dListNode* m_end;
	const dBasicBlock* m_idom;
	mutable dList<const dBasicBlock*> m_children;
	mutable dTree<int, const dBasicBlock*> m_dominators; 
	//mutable dList<const dBasicBlock*> m_dominanceFrontier; 
	dList<const dBasicBlock*> m_successors;
	dList<const dBasicBlock*> m_predecessors;
	mutable int m_mark;
};


class dStatementBlockBucket: public dTree <const dBasicBlock*, const dCIL::dListNode*>
{
	public:
	dStatementBlockBucket()
		:dTree <const dBasicBlock*, const dCIL::dListNode*>()
	{
	}
};

class dStatementBlockDictionary: public dTree <dStatementBlockBucket, dString>
{
	public:
	dStatementBlockDictionary()
		:dTree <dStatementBlockBucket, dString>()
	{
	}

	void BuildUsedVariableWorklist(dBasicBlocksGraph& list);
};

class dWorkList: public dTree <dCIL::dListNode*, int>
{
	public:
	dWorkList()
		:dTree <dCIL::dListNode*, int>()
	{
	}

	void Insert(dCILInstr* const instruction)
	{
		dTree <dCIL::dListNode*, int>::Insert(instruction->GetNode(), instruction->GetUniqueID());
	}
};


class dBasicBlocksList: public dList<dBasicBlock> 
{
};

/*
class dDataFlowPoint
{
	public:

	void Init(dCIL::dListNode* const statementNode, dBasicBlocksList::dListNode* const basicBlock)
	{
		//m_mark = 0;
		//m_generateStmt = false;
		m_basicBlockNode = basicBlock;
		m_statement = statementNode;
	}

	dList<dDataFlowPoint*> m_successors;
	dList<dDataFlowPoint*> m_predecessors;

	dCIL::dListNode* m_statement;
	dBasicBlocksList::dListNode* m_basicBlockNode;

	dString m_generatedVariable;
	dVariableSet<dString> m_liveInputSet;
	dVariableSet<dString> m_liveOutputSet;
	dVariableSet<dString> m_usedVariableSet;

	int m_mark;
	bool m_generateStmt;

	dVariableSet<dCIL::dListNode*> m_killStmtSet;
	dVariableSet<dCIL::dListNode*> m_reachStmtInputSet;
	dVariableSet<dCIL::dListNode*> m_reachStmtOutputSet;
};
*/

class dLiveInLiveOut
{
	public:
	dCILInstr* m_instruction;
	dVariableSet<dString> m_liveInputSet;
	dVariableSet<dString> m_liveOutputSet;
	dList<dLiveInLiveOut*> m_successors;
//	dList<dLiveInLiveOut*> m_predecessors;
};

class dLiveInLiveOutSolver: public dList<dLiveInLiveOut>
{
	public:
	dLiveInLiveOutSolver(dBasicBlocksGraph* const graph);
	void BuildReverseOrdeBlockList(dList<const dBasicBlock*>& reverseOrderList, const dBasicBlock* const block) const;

	void Trace();
	dBasicBlocksGraph* m_graph;
};

class dBasicBlocksGraph: public dBasicBlocksList
{
	public:
	dBasicBlocksGraph();
	void Trace() const;

	void Build (dCIL::dListNode* const functionNode);
	void OptimizeSSA ();
	void ConvertToSSA ();

	void RegistersAllocations ();

	private:
	void GetStatementsWorklist (dWorkList& workList) const;

	void RemovePhyFunctions ();
	void BuildDominatorTree ();
	void DeleteUnreachedBlocks();
	void CalculateSuccessorsAndPredecessors ();

	// Single Static transformation assignments passes
	bool ApplyCopyPropagationSSA();
	//bool ApplyConstantPropagationSSA();
	bool ApplyDeadCodeEliminationSSA();
	bool ApplyConstantConditionalSSA();
	bool ApplyConstantPropagationSSA();

	// Non Single Static transformation assignments passes

	dCIL::dListNode* m_begin;
	dCIL::dListNode* m_end;
	dBasicBlock* m_dominatorTree;
	int m_savedRegistersMask;
	mutable int m_mark;

	friend class dBasicBlocksList;
	friend class dConvertToSSASolver;
	friend class dLiveInLiveOutSolver;
	friend class dRegisterInterferenceGraph;
	friend class dConstantPropagationSolver;
};


#endif