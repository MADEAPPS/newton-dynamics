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


class dBasicBlock
{
	public:
	dBasicBlock (dCIL::dListNode* const begin);
	dBasicBlock (const dBasicBlock& src);
	void Trace() const;

	void ReplaceDominator(const dTree<int, const dBasicBlock*>& dominators);
	bool ComparedDominator(const dTree<int, const dBasicBlock*>& dominators) const;
	
	int m_mark;
	dCIL::dListNode* m_begin;
	dCIL::dListNode* m_end;
	const dBasicBlock* m_idom;
	mutable dList<const dBasicBlock*> m_children;
	mutable dTree<int, const dBasicBlock*> m_dominators; 
	//mutable dList<const dBasicBlock*> m_dominanceFrontier; 

	dList<const dBasicBlock*> m_successors;
	dList<const dBasicBlock*> m_predecessors;
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

class dWorkList : public dTree <dCIL::dListNode*, int>
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


class dBasicBlocksGraph: public dList<dBasicBlock> 
{
	public:
	dBasicBlocksGraph();
	void Trace() const;

	void Build (dCIL::dListNode* const functionNode);
	void OptimizeSSA ();
	void ConvertToSSA ();
	void RemovePhyFunctions ();

	private:
	void GetStatementsWorklist (dWorkList& workList) const;

	void BuildDominatorTree ();
	void DeleteUnreachedBlocks();
	void CalculateSuccessorsAndPredecessors ();

	bool ApplyCopyPropagationSSA();
	//bool ApplyConstantPropagationSSA();
	bool ApplyDeadCodeEliminationSSA();
	bool ApplyConstantConditionalSSA();
	bool ApplyConstantPropagationSSA();

	dCIL::dListNode* m_begin;
	dCIL::dListNode* m_end;
	dBasicBlock* m_dominatorTree;
	int m_mark;

	friend class dConvertToSSASolver;
	friend class dConstantPropagationSolver;
};


#endif