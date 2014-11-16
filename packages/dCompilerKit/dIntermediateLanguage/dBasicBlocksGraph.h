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
	mutable dList<const dBasicBlock*> m_dominanceFrontier; 

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


class dBasicBlocksGraph: public dList<dBasicBlock> 
{
	class dStatementBucket: public dTree <const dCILInstr*, const dBasicBlock*>
	{
		public:
		dStatementBucket()
			:dTree <const dCILInstr*, const dBasicBlock*>()
			,m_index (0)
		{
		}

		int m_index;
		dList<int> m_stack;
		dCILInstr::dArg m_variable;
	};

	public:
	dBasicBlocksGraph();

	void Trace() const;

	void Build (dCIL::dListNode* const functionNode);

	void OptimizeSSA ();
	void ConvertToSSA ();

	private:
	void GetStatementsWorklist (dTree <int, dCIL::dListNode*>& list) const;
	void BuildDomicanceFrontier (const dBasicBlock* const root) const;
	void RenameVariables (const dBasicBlock* const root, dTree <dStatementBucket, dString>& stack) const;

	void BuildDominatorTree ();
	void DeleteUnreachedBlocks();
	void CalculateSuccessorsAndPredecessors ();

	bool ApplyCopyPropagationSSA();
	bool ApplyConstantPropagationSSA();
	bool ApplyDeadCodeEliminationSSA();
	bool ApplyConstantConditionalSSA();
	bool ApplyConditionalConstantPropagationSSA();

	dCIL::dListNode* m_begin;
	dCIL::dListNode* m_end;
	dBasicBlock* m_dominatorTree;
	int m_mark;

	friend class dConstantPropagationSolver;
};


#endif