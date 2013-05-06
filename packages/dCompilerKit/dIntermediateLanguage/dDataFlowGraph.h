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
				Iterator iter (*this);
				for (iter.Begin(); iter; iter ++) {
					dTrace (("%s ", iter.GetKey().GetStr()));
				}
				dTrace (("\n"));
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

	class dRegisterInterferenceWorkingSet: public dList<dTree<dRegisterInterferenceNode, dString>::dTreeNode*>
	{
		public:
		dRegisterInterferenceWorkingSet(dTree<dRegisterInterferenceNode, dString>* const interferenceGraph, int registerCount)
			:dList<dTree<dRegisterInterferenceNode, dString>::dTreeNode*>()
			,m_registerCount(registerCount)
			,m_graph(interferenceGraph)
		{
		}

		void GetSet ()
		{
			RemoveAll();
			dTree<dRegisterInterferenceNode, dString>::Iterator iter (*m_graph);
			for (iter.Begin(); iter; iter ++) {
				dTree<dRegisterInterferenceNode, dString>::dTreeNode* const node = iter.GetNode();
				dRegisterInterferenceNode& info = node->GetInfo();
				if (!info.m_inSet) {

					int count = 0;
					for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = info.m_interferanceEdge.GetFirst(); edgeNode && (count < m_registerCount); edgeNode = edgeNode->GetNext()) {
						dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
						if (!edge.m_mark) {
							count += (edge.m_isMov) ? 0 : 1;
						}
					}

					if (count < m_registerCount) {
						info.m_inSet = true;
						Append(node);
					}
				}
			}

			if (!GetCount()) {
				_ASSERTE (GetCount());
				// the function cannot be generate with m_registerCount, select a node candidate for spill here
			}
		}

		

		int m_registerCount;
		dTree<dRegisterInterferenceNode, dString>* m_graph;
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
	void RegistersAllocation (int classRegisterIndex, int returnRegisterIndex, int registerCount);

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
	bool ApplySubExpresionToCopyPropagation();
	bool ApplyLoopOptimization(dLoop& loop);
	
	void AllocateRegisters (dTree<dRegisterInterferenceNode, dString>& interferenceGraph);
	void BuildInterferenceGraph (dTree<dRegisterInterferenceNode, dString>& interference);
	void ColorInterferenceGraph (dTree<dRegisterInterferenceNode, dString>& interference, int registerCount);
	
	dString GetRegisterName (dTree<dRegisterInterferenceNode, dString>& interferenceGraph, const dString& varName) const;
	void FindNodesInPathway(dCIL::dListNode* const source, dCIL::dListNode* const destination, dTree<int, dCIL::dListNode*>& pathOut) const;

	bool DoStatementAreachesStatementB(dCIL::dListNode* const stmtNodeB, dCIL::dListNode* const stmtNodeA) const;
	int EvaluateBinaryExpression (const dString& arg1, dTreeAdressStmt::dOperator operation, const dString& arg2) const;

	void GetLoops (dList<dLoop>& loops) const;
	bool CheckBackEdgePreReachInLoop(dCIL::dListNode* const stmt, const dLoop& loop) const;
	bool IsStatementInReachList(dCIL::dListNode* const node, dList<dCIL::dListNode*>& definitionList, dCIL::dListNode* const me) const;

	mutable int m_mark;
	int m_registersUsed;
	dCIL* m_cil;
	dCIL::dReturnType m_returnType;
	dString m_returnVariableName;
	dCIL::dListNode* m_function;
	dList<dBasicBlock> m_basicBlocks; 
	dList<dBasicBlock*> m_traversalBlocksOrder; 
	dTree<dDataFlowPoint, dCIL::dListNode*> m_dataFlowGraph;
	dTree<dList<dCIL::dListNode*>, dString> m_variableDefinitions;
};


#endif