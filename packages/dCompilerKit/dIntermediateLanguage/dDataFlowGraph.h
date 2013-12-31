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

#ifndef _dDataFlowGraph_H_
#define _dDataFlowGraph_H_


#include "dCILstdafx.h"
#include "dTreeAdressStmt.h"

inline dString MakeRegister(int index)
{
	char regName[256];
	sprintf (regName, "%s%d", D_REGISTER_SYMBOL, index);
	return regName;			
}


inline int GetRegisterIndex (const dString& reg)
{
	static int base = int (strlen (D_REGISTER_SYMBOL));
	return dString(reg.GetStr() + base).ToInteger();
}


class dDataFlowGraph 
{
	public:

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
				#ifdef TRACE_INTERMEDIATE_CODE
					Iterator iter (*this);
					for (iter.Begin(); iter; iter ++) {
						dTrace (("%s ", iter.GetKey().GetStr()));
					}
					dTrace (("\n"));
				#endif
			}
		};


		void Init(dCIL::dListNode* const statementNode)
		{
			m_mark = 0;
			m_generateStmt = false;
			m_statement = statementNode;
		}
		
		dList<dDataFlowPoint*> m_successors;
		dList<dDataFlowPoint*> m_predecessors; 
	
		dString m_killVariable;
		dVariableSet<dString> m_liveInputSet;
		dVariableSet<dString> m_liveOutputSet;
		dVariableSet<dString> m_generatedVariableSet;

		int m_mark;
		bool m_generateStmt;
		
		dCIL::dListNode* m_statement;
		dVariableSet<dCIL::dListNode*> m_killStmtSet;
		dVariableSet<dCIL::dListNode*> m_reachStmtInputSet;
		dVariableSet<dCIL::dListNode*> m_reachStmtOutputSet;
	};



	class dBasicBlock
	{
		public:
		dBasicBlock (dCIL::dListNode* const begin)
			:m_mark (0)
			,m_begin (begin)
			,m_end(NULL)
		{
		}
		void Trace() const;

		int m_mark;
		dCIL::dListNode* m_begin;
		dCIL::dListNode* m_end;
	};

	class dBasicBlocks: public dList<dBasicBlock> 
	{
		public:
		dBasicBlocks()
			:dList<dBasicBlock> ()
		{
		}

		void Trace() const
		{
			#ifdef TRACE_INTERMEDIATE_CODE
				for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
					node->GetInfo().Trace();
				}
			#endif
		}
	};

	class dRegisterInterferenceNode;
	class dRegisterInterferenceNodeEdge
	{
		public:
		dRegisterInterferenceNodeEdge (dTree<dRegisterInterferenceNode, dString>::dTreeNode* const targetNode)
			:m_isMov(false)
			,m_mark (0)
			,m_twin(NULL)
			,m_targetNode(targetNode)
		{
		}

		bool m_isMov;
		int m_mark;
		dRegisterInterferenceNodeEdge* m_twin;
		dTree<dRegisterInterferenceNode, dString>::dTreeNode* m_targetNode;
	};


	class dRegisterInterferenceNode
	{
		public: 
		dRegisterInterferenceNode()
			:m_inSet (false)
			,m_registerIndex (-1)
		{
		}

		bool m_inSet;
		int m_registerIndex;
		dString m_name;
		dList<dRegisterInterferenceNodeEdge> m_interferanceEdge;
	};


	class dRegisterInterferenceGraph: public dTree<dRegisterInterferenceNode, dString>
	{
		public: 
		dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount);

		private:
		int ColorGraph (int registerCount);
		void SortRegistersByFrequency (int totalRegisters, int registerCount);
		void AllocatedSpillsRegister(int totalRegisters, int registerCount);

		void RemapRegister(dTreeAdressStmt::dArg& arg, int totalRegisters);
		void SortRegisters(int totalRegisters);
		int FindRegister (int regIndex, int totalRegisters) const;


		void SaveSpillRegister(dCIL::dListNode* const node, const dString& reg, const dString& local);
		static int Compare (const void* p1, const void* p2);


		dTreeNode* GetBestNode();
		dDataFlowGraph* m_flowGraph;
		int m_registerCount;
		int m_registenFrequency[1024][2];
	};

	class dLoop 
	{
		public:
		dCIL::dListNode* m_head;
		dCIL::dListNode* m_tail;
	};

	class dDominator: public dTree<int, dCIL::dListNode*> 
	{
		public:
		dDominator()
			:dTree<int, dCIL::dListNode*>()
			,m_isLoopInvariant(false)
		{
		}

		dDominator(const dDominator& copy)
			:dTree<int, dCIL::dListNode*>()
			,m_isLoopInvariant(false)
		{
			Iterator iter (copy);
			for (iter.Begin(); iter; iter ++) {
				Insert(0, iter.GetKey());
			}
		}

		void Intersection (const dDominator& parent)
		{
			Iterator iter (*this);
			for (iter.Begin(); iter; ) {
				dTreeNode* const node = iter.GetNode();
				iter ++;
				if (!parent.Find (node->GetKey())){
					Remove (node);
				}
			}
		}

		bool Compare (const dDominator& cmp) const
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
		bool m_isLoopInvariant;	
	};



	dDataFlowGraph(dCIL* const cil, dCIL::dListNode* const function, dCIL::dReturnType returnType);
	virtual ~dDataFlowGraph(void);

	void ApplyLocalOptimizations();
	void RegistersAllocation (int registerCount);

	private:
	void CalculateReachingDefinitions();
	void CalculateLiveInputLiveOutput();
	void UpdateLiveInputLiveOutput();

	void BuildBasicBlockGraph();
	void BuildGeneratedAndKillVariableSets();
	void BuildGeneratedAndKillStatementSets();
	void UpdateReachingDefinitions();

	bool ApplyRemoveDeadCode();
	bool ApplyCopyPropagation();
	bool ApplyConstantFolding();
	bool ApplyCommonSubExpresion();
	bool ApplyConstantPropagation();
	bool ApplyInstructionSematicOrdering();
	bool ApplyIfStatementsSimplification();
	bool ApplySubExpresionToCopyPropagation();
	
	bool ApplyLoopOptimization(dLoop& loop);
	void AllocateRegisters (dRegisterInterferenceGraph& interferenceGraph);
	
	dString GetRegisterName (dRegisterInterferenceGraph& interferenceGraph, const dString& varName) const;
	void FindNodesInPathway(dCIL::dListNode* const source, dCIL::dListNode* const destination, dTree<int, dCIL::dListNode*>& pathOut) const;

	bool DoStatementAreachesStatementB(dCIL::dListNode* const stmtNodeB, dCIL::dListNode* const stmtNodeA) const;
	int EvaluateBinaryExpression (const dString& arg1, dTreeAdressStmt::dOperator operation, const dString& arg2) const;

	void GetLoops (dList<dLoop>& loops) const;
	bool CheckBackEdgePreReachInLoop(dCIL::dListNode* const stmt, const dLoop& loop) const;
	bool IsStatementInReachList(dCIL::dListNode* const node, dList<dCIL::dListNode*>& definitionList, dCIL::dListNode* const me) const;

	mutable int m_mark;
	int m_registersUsedMask;
	dCIL* m_cil;
	dCIL::dReturnType m_returnType;
	dString m_returnVariableName;
	dCIL::dListNode* m_function;
	dBasicBlocks m_basicBlocks; 
	dList<dBasicBlock*> m_traversalBlocksOrder; 
	dTree<dDataFlowPoint, dCIL::dListNode*> m_dataFlowGraph;
	dTree<dList<dCIL::dListNode*>, dString> m_variableDefinitions;
};


#endif