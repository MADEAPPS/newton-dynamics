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

#ifndef _dDataFlowGraph_H_
#define _dDataFlowGraph_H_


#include "dCILstdafx.h"
#include "dCILInstr.h"
#include "dBasicBlocksGraph.h"

inline dString IndexToRegister(int index)
{
	char regName[256];
	sprintf (regName, "%s%d", D_REGISTER_SYMBOL, index);
	return regName;			
}

inline int RegisterToIndex (const dString& reg)
{
	return dString(reg.GetStr() + strlen(D_REGISTER_SYMBOL)).ToInteger();
}

inline dString IndexToLocal(int index)
{
	char regName[256];
	sprintf (regName, "%s%d", D_SPILL_REGISTER_SYMBOL, index);
	return regName;			
}


class dRegisterInterferenceGraph;

/*
class dDataFlowPoint
{
	public:
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
			Iterator iter (copy);
			for (iter.Begin(); iter; iter ++) {
				Insert(0, iter.GetKey());
			}
		}

		void Difference (const dVariableSet& b, const dVariableSet& c)
		{
			Iterator iter0 (b);
			for (iter0.Begin(); iter0; iter0 ++) {
				Insert(0, iter0.GetKey());
			}

			Iterator iter1 (c);
			for (iter1.Begin(); iter1; iter1 ++) {
				dTreeNode* const node = Find (iter1.GetKey());
				if (node) {
					Remove (node);
				}
			}
		}

		void Union (const dVariableSet& or)
		{
			Iterator iter (or);
			for (iter.Begin(); iter; iter ++) {
				Insert(0, iter.GetKey());
			}
		}


		bool Compare (const dVariableSet& cmp) const
		{
			if (GetCount() == cmp.GetCount()) {
				Iterator iter0 (cmp);
				Iterator iter1 (*this);
				for (iter0.Begin(), iter1.Begin(); iter0 && iter1 ; iter0 ++, iter1 ++) {
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
			Iterator iter (*this);
			for (iter.Begin(); iter; iter ++) {
				dTrace (("%s ", iter.GetKey().GetStr()));
			}
			dTrace (("\n"));
		}
	};

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


class dInstructionVariableDictionary: public dTree<dList<dCIL::dListNode*>, dString>
{
	public:
	dInstructionVariableDictionary ()
		:dTree<dList<dCIL::dListNode*>, dString>()
	{
	}
};
*/



class dWorkList: public dTree <int, dCIL::dListNode*>
{
	public:
	dWorkList ()
		:dTree <int, dCIL::dListNode*>()
	{
	}
};

class dDataFlowGraph 
{
	public:
	class dTransformation
	{
		public: 
		dTransformation ()
		{
		}

		virtual ~dTransformation ()
		{
		}
	};

	dDataFlowGraph (dCIL* const cil, dCIL::dListNode* const function);
	virtual ~dDataFlowGraph(void);
/*
	void ConvertToSSA ();
	void ApplyLocalOptimizations();
	void RegistersAllocation (int registerCount);
	private:
	void CalculateReachingDefinitions();
	void CalculateLiveInputLiveOutput();
	void UpdateLiveInputLiveOutput();
	void BuildGeneratedAndUsedVariableSets();
	void BuildDefinedAndKilledStatementSets();
	void UpdateReachingDefinitions();
	bool ApplyRemoveDeadCode();
	bool ApplyCopyPropagation();
	bool ApplyConstantFolding();
	bool ApplyCommonSubExpresion();
	bool ApplyConstantPropagation();
	bool ApplyIfStatementsSimplification();
	bool ApplySubExpresionToCopyPropagation();
	bool ApplyLoopOptimization(dLoop& loop);

	bool RemoveNop ();
	bool RemoveDeadInstructions();
	bool RemoveRedundantJumps ();
	void FindNodesInPathway(dCIL::dListNode* const source, dCIL::dListNode* const destination, dTree<int, dCIL::dListNode*>& pathOut) const;

	//bool DoStatementAreachesStatementB(dCIL::dListNode* const stmtNodeB, dCIL::dListNode* const stmtNodeA) const;
	bool DoMoveReachInstruction(dCIL::dListNode* const stmtNodeB, dCILInstrMove* const move) const;
	int EvaluateBinaryExpression (const dString& arg1, dCILThreeArgInstr::dOperator operation, const dString& arg2) const;

	void GetLoops (dList<dLoop>& loops) const;
	bool CheckBackEdgePreReachInLoop(dCIL::dListNode* const stmt, const dLoop& loop) const;
	bool IsStatementInReachList(dCIL::dListNode* const node, dList<dCIL::dListNode*>& definitionList, dCIL::dListNode* const me) const;

	void TraceReachIn (dCIL::dListNode* const node) const;
	void TraceReachOutput (dCIL::dListNode* const node) const;

	dInstructionVariableDictionary m_variableUsed;
	dInstructionVariableDictionary m_variableDefinitions;


	friend dCILInstrMove;
	friend dCILSingleArgInstr;
	friend dRegisterInterferenceGraph;
*/


	private:
	//void BuildBasicBlockGraph();
	//void GetStatementsWorklist (dTree <int, dCIL::dListNode*>& list) const;
	//void GeneratedVariableWorklist (dTree <int, dCIL::dListNode*>& list) const;

	dCIL* m_cil;
	dCIL::dListNode* m_function;
	//dBasicBlocksGraph m_basicBlocks; 
	//dTree<dDataFlowPoint, dCIL::dListNode*> m_dataFlowGraph;
	//dList<dBasicBlock*> m_traversalBlocksOrder; 

	mutable int m_mark;
	friend class dCIL;
	friend class dBasicBlock;
	friend class dBasicBlocksGraph;
};


#endif