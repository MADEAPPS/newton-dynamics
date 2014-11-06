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


#include "dCILstdafx.h"
#include "dDataFlowGraph.h"
#include "dCILInstrBranch.h"
#include "dBasicBlockList.h"
#include "dCILInstrLoadStore.h"


dBasicBlock::dBasicBlock (dCIL::dListNode* const begin)
	:m_mark (0)
	,m_begin (begin)
	,m_end(NULL)
	,m_idom(NULL)
{
}

void dBasicBlock::Trace() const
{
	bool terminate = false;
	for (dCIL::dListNode* node = m_begin; !terminate; node = node->GetNext()) {
		terminate = (node == m_end);
		node->GetInfo()->Trace();
	}

	dCILInstrLabel* const label = m_begin->GetInfo()->GetAsLabel();
	dTrace ((" block-> %s\n", label->GetArg0().m_label.GetStr()));

	if (m_idom) {
		dTrace(("   immediateDominator-> %s\n", m_idom->m_begin->GetInfo()->GetAsLabel()->GetArg0().m_label.GetStr()));
	} else {
		dTrace(("   immediateDominator->\n"));
	}

	dTrace(("   dominatorTreeChildren-> "));
	for (dList<const dBasicBlock*>::dListNode* childNode = m_children.GetFirst(); childNode; childNode = childNode->GetNext()) {
		const dBasicBlock* const childBlock = childNode->GetInfo();
		dCILInstrLabel* const label = childBlock->m_begin->GetInfo()->GetAsLabel();
		dTrace(("%s ", label->GetArg0().m_label.GetStr()));
	}
	dTrace(("\n"));


	dTrace (("   dominators-> "));
	dTree<int, const dBasicBlock*>::Iterator iter (m_dominators);
	for (iter.Begin(); iter; iter ++) {
		const dBasicBlock& domBlock = *iter.GetKey();
		dCILInstrLabel* const label = domBlock.m_begin->GetInfo()->GetAsLabel();
		dTrace (("%s ", label->GetArg0().m_label.GetStr()));
	}
	dTrace (("\n"));

	dTrace(("   dominanceFrontier-> "));
	for (dList<const dBasicBlock*>::dListNode* childFrontierNode = m_dominanceFrontier.GetFirst(); childFrontierNode; childFrontierNode = childFrontierNode->GetNext()) {
		const dBasicBlock* const childBlock = childFrontierNode->GetInfo();
		dCILInstrLabel* const label = childBlock->m_begin->GetInfo()->GetAsLabel();
		dTrace(("%s ", label->GetArg0().m_label.GetStr()));
	}
	dTrace (("\n"));

	dTrace (("\n"));
}


bool dBasicBlock::ComparedDominator(const dTree<int, const dBasicBlock*>& newdominators) const
{
	if (m_dominators.GetCount() != newdominators.GetCount()) {
		return true;
	}

	dTree<int, const dBasicBlock*>::Iterator iter0 (m_dominators);
	dTree<int, const dBasicBlock*>::Iterator iter1 (newdominators);
	for (iter0.Begin(), iter1.Begin(); iter0 && iter1; iter0++, iter1++) {
		if (iter0.GetKey() != iter1.GetKey()) {
			return true;
		}
	}
	return false;
}

void dBasicBlock::ReplaceDominator (const dTree<int, const dBasicBlock*>& newdominators)
{
	m_dominators.RemoveAll();
	dTree<int, const dBasicBlock*>::Iterator iter1(newdominators);
	for (iter1.Begin(); iter1; iter1++) {
		m_dominators.Insert(0, iter1.GetKey());
	}
}



void dBasicBlocksList::dVariablesDictionary::Build(const dBasicBlocksList& list)
{
	RemoveAll();
	for (dListNode* node = list.GetFirst(); node; node = node->GetNext()) {
		dBasicBlock& block = node->GetInfo();

		for (dCIL::dListNode* node = block.m_end; node != block.m_begin; node = node->GetPrev()) {
			dCILInstr* const instruction = node->GetInfo();
			//instruction->Trace();
			const dCILInstr::dArg* const variable = instruction->GetGeneratedVariable();
			if (variable) {
				dVariablesDictionary::dTreeNode* entry = Find(variable->m_label);
				if (!entry) {
					entry = Insert(variable->m_label);
					entry->GetInfo().m_variable = dCILInstr::dArg(variable->m_label, variable->GetType());
				}
				dStatementBucket& buckect = entry->GetInfo();
				buckect.Insert(instruction, &block);
			}
		}
	}
}

dBasicBlocksList::dBasicBlocksList()
	:dList<dBasicBlock> ()
	,m_dominatorTree(NULL)
{
}


void dBasicBlocksList::Trace() const
{
	for (dList<dBasicBlock>::dListNode* blockNode = GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		block.Trace();
	}
}

void dBasicBlocksList::BuildBegin (dCIL::dListNode* const functionNode)
{
	RemoveAll();
	m_begin = functionNode->GetNext();
	dAssert (m_begin->GetInfo()->GetAsLabel());
	for (m_end = functionNode->GetNext(); !m_end->GetInfo()->GetAsFunctionEnd(); m_end = m_end->GetNext());

	// find the root of all basic blocks leaders
	for (dCIL::dListNode* node = m_begin; node != m_end; node = node->GetNext()) {
		if (node->GetInfo()->IsBasicBlockBegin()) {
			dAssert (node->GetInfo()->GetAsLabel());
			Append(dBasicBlock(node));
		}
	}

	for (dList<dBasicBlock>::dListNode* blockNode = GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();
		for (dCIL::dListNode* node = block.m_begin; node; node = node->GetNext()) {
//node->GetInfo()->Trace();
			if (node->GetInfo()->IsBasicBlockEnd()) {
				block.m_end = node;
				break;
			}
		} 
//block.Trace();
	}

//cil.Trace();
//Trace();
}

void dBasicBlocksList::CalculateSuccessorsAndPredecessors(dDataFlowGraph* const dataFlow)
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dBasicBlock& block =  node->GetInfo();
		block.m_successors.RemoveAll();
		block.m_predecessors.RemoveAll();

		const dDataFlowPoint& blockEndDatapoint = dataFlow->m_dataFlowGraph.Find(block.m_end)->GetInfo();
		for (dList<dDataFlowPoint*>::dListNode* succNode = blockEndDatapoint.m_successors.GetFirst(); succNode; succNode = succNode->GetNext()) {
			block.m_successors.Append(&succNode->GetInfo()->m_basicBlockNode->GetInfo());
		}

		const dDataFlowPoint& blockBeginDatapoint = dataFlow->m_dataFlowGraph.Find(block.m_begin)->GetInfo();
		for (dList<dDataFlowPoint*>::dListNode* preceNode = blockBeginDatapoint.m_predecessors.GetFirst(); preceNode; preceNode = preceNode->GetNext()) {
			block.m_predecessors.Append(&preceNode->GetInfo()->m_basicBlockNode->GetInfo());
		}
	}
}

void dBasicBlocksList::BuildDominatorTree (dDataFlowGraph* const dataFlow)
{
	// dominator of the start node is the start itself
	//Dom(n0) = { n0 }
	dBasicBlock& firstBlock = GetFirst()->GetInfo();
	firstBlock.m_dominators.Insert(0, &firstBlock);

	// for all other nodes, set all nodes as the dominators
	for (dListNode* node = GetFirst()->GetNext(); node; node = node->GetNext()) {
		dBasicBlock& block = node->GetInfo();
		for (dListNode* node1 = GetFirst(); node1; node1 = node1->GetNext()) {
			block.m_dominators.Insert(0, &node1->GetInfo());
		}
	}

	bool change = true;
	while (change) {
		change = false;
		for (dListNode* node = GetFirst()->GetNext(); node; node = node->GetNext()) {
			dBasicBlock& block = node->GetInfo();

//block.Trace();
			dTree<int, const dBasicBlock*> predIntersection;
			const dBasicBlock& predBlock = *block.m_predecessors.GetFirst()->GetInfo();

			dTree<int, const dBasicBlock*>::Iterator domIter (predBlock.m_dominators);
			for (domIter.Begin(); domIter; domIter ++) {
				const dBasicBlock* const block = domIter.GetKey();
				predIntersection.Insert (0, block);
			}

			for (dList<const dBasicBlock*>::dListNode* predNode = block.m_predecessors.GetFirst()->GetNext(); predNode; predNode = predNode->GetNext()) {
				const dBasicBlock& predBlock = *predNode->GetInfo();
				dTree<int, const dBasicBlock*>::Iterator predIter (predIntersection);
				for (predIter.Begin(); predIter; ) {
					const dBasicBlock* block = predIter.GetKey();
					predIter ++;
					if (!predBlock.m_dominators.Find(block)) {
						predIntersection.Remove(block);
					}
				}
			}

			dAssert (!predIntersection.Find(&block));
			predIntersection.Insert(&block);

			bool dominatorChanged = block.ComparedDominator (predIntersection);
			if (dominatorChanged) {
				block.ReplaceDominator (predIntersection);
				//block.Trace();
			}
			change |= dominatorChanged;
		}
	}

	// find the immediate dominator of each block
	for (dListNode* node = GetFirst()->GetNext(); node; node = node->GetNext()) {
		dBasicBlock& block = node->GetInfo();
//block.Trace();

		dAssert (block.m_dominators.GetCount() >= 2);
		dTree<int, const dBasicBlock*>::Iterator iter(block.m_dominators);

		for (iter.Begin(); iter; iter, iter++) {
			const dBasicBlock* const sblock = iter.GetKey();
			if (sblock != &block) {
				if (sblock->m_dominators.GetCount() == (block.m_dominators.GetCount() - 1)) {
					bool identical = true;
					dTree<int, const dBasicBlock*>::Iterator iter0(block.m_dominators);
					dTree<int, const dBasicBlock*>::Iterator iter1(sblock->m_dominators);
					for (iter0.Begin(), iter1.Begin(); identical && iter0 && iter1; iter0++, iter1++) {
						if (iter0.GetKey() == &block) {
							iter0 ++;
						}
						identical &= (iter0.GetKey() == iter1.GetKey());
					}
					if (identical) {
						if (sblock->m_dominators.GetCount() == (block.m_dominators.GetCount() - 1)) {
							block.m_idom = sblock;
							break;
						}
					}
				}
			}
		}
		dAssert (block.m_idom);
		//block.m_idom->Trace();
	}

	// build dominator tree
	m_dominatorTree = &GetFirst()->GetInfo();
	for (dListNode* node = GetFirst()->GetNext(); node; node = node->GetNext()) {
		const dBasicBlock& block = node->GetInfo();
		block.m_idom->m_children.Append(&block);
	}
	
//Trace();
}


void dBasicBlocksList::BuildDomicanceFrontier (const dBasicBlock* const root, dDataFlowGraph* const dataFlow) const
{
	dTree<int, const dBasicBlock*> frontier;
	for (dList<const dBasicBlock*>::dListNode* succNode = root->m_successors.GetFirst(); succNode; succNode = succNode->GetNext()) {
		const dBasicBlock& succBlock = *succNode->GetInfo();
		if (succBlock.m_idom != root) {
			frontier.Insert(0, &succBlock);
		}
//succBlock.Trace();
	}

	for (dList<const dBasicBlock*>::dListNode* node = root->m_children.GetFirst(); node; node = node->GetNext()) {
		const dBasicBlock* const childBlock = node->GetInfo();
		BuildDomicanceFrontier (childBlock, dataFlow);
		for (dList<const dBasicBlock*>::dListNode* childFrontierNode = childBlock->m_dominanceFrontier.GetFirst(); childFrontierNode; childFrontierNode = childFrontierNode->GetNext()) {
			const dBasicBlock* const childBlock = childFrontierNode->GetInfo();
			if (childBlock->m_idom != root) {
				frontier.Insert(0, childBlock);
			}
		}
	}

	dTree<int, const dBasicBlock*>::Iterator iter (frontier);
	for (iter.Begin(); iter; iter ++) {
		root->m_dominanceFrontier.Append(iter.GetKey());
	}
}

void dBasicBlocksList::ConvertToSSA (dDataFlowGraph* const dataFlow)
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dBasicBlock& block = node->GetInfo();
		block.m_dominanceFrontier.RemoveAll();
	}
	BuildDomicanceFrontier (m_dominatorTree, dataFlow);

//Trace();

	dVariablesDictionary variableList;
	variableList.Build (*this);

	dVariablesDictionary phyVariables;
	dVariablesDictionary::Iterator iter (variableList);
	for (iter.Begin(); iter; iter ++) {
		dStatementBucket w;
		dStatementBucket::Iterator bucketIter (iter.GetNode()->GetInfo());
		for (bucketIter.Begin(); bucketIter; bucketIter ++) {
			w.Insert(bucketIter.GetNode()->GetInfo(), bucketIter.GetKey());
		}

		//const dString& name = iter.GetKey();
		dCILInstr::dArg name (iter.GetNode()->GetInfo().m_variable);
		dStatementBucket& wPhy = phyVariables.Insert(name.m_label)->GetInfo();
		while (w.GetCount()) {
			const dBasicBlock* const block = w.GetRoot()->GetKey();
			const dCILInstr* const instruction = w.GetRoot()->GetInfo();
			w.Remove(w.GetRoot());

//instruction->Trace();
//block->Trace();
			for (dList<const dBasicBlock*>::dListNode* node = block->m_dominanceFrontier.GetFirst(); node; node = node->GetNext()) {
				const dBasicBlock* const frontier = node->GetInfo();
				if (!wPhy.Find(frontier)) {
					dList<dCILInstr*> sources;

					for (dList<const dBasicBlock*>::dListNode* predNode = frontier->m_predecessors.GetFirst(); predNode; predNode = predNode->GetNext()) {
						const dBasicBlock& predBlock = *predNode->GetInfo();
						for (dCIL::dListNode* node = predBlock.m_end; node != predBlock.m_begin; node = node->GetPrev()) {
							dCILInstr* const instruction = node->GetInfo();
							const dCILInstr::dArg* const variable = instruction->GetGeneratedVariable();
							if (variable && variable->m_label == name.m_label) {
								sources.Append(instruction);
								break;
							}
						}
					}
					if (sources.GetCount() > 1) {
						dCILInstrPhy* const phyInstruction = new dCILInstrPhy (*dataFlow->m_cil, name.m_label, name.GetType(), sources);
						dataFlow->m_cil->InsertAfter(frontier->m_begin, phyInstruction->GetNode());
	//frontier->Trace();

						wPhy.Insert(instruction, frontier);
						if (!iter.GetNode()->GetInfo().Find(frontier)) {
							w.Insert(instruction, frontier);
						}
					}
				}
			}
		}
	}

	dataFlow->CalculateSuccessorsAndPredecessors();
	variableList.Build (*this);

//Trace ();
	dTree<int, dString> stack;
	RenameVariables (&GetFirst()->GetInfo(), dataFlow, variableList);
}


void dBasicBlocksList::RenameVariables (const dBasicBlock* const root, dDataFlowGraph* const dataFlow, dVariablesDictionary& stack) const
{
//root->Trace();
	bool terminate = false;
	for (dCIL::dListNode* node = root->m_begin; !terminate; node = node->GetNext()) {
		terminate = (node == root->m_end);
		dCILInstr* const instruction = node->GetInfo();
//instruction->Trace();
		if (!instruction->GetAsPhy()) {
			dList<dCILInstr::dArg*> variablesList;
			instruction->GetUsedVariables (variablesList);
			for (dList<dCILInstr::dArg*>::dListNode* argNode = variablesList.GetFirst(); argNode; argNode = argNode->GetNext()) {
				dCILInstr::dArg* const variable = argNode->GetInfo();
				dString name (instruction->RemoveSSAPostfix(variable->m_label));
				dAssert(stack.Find(name));
				dStatementBucket& topStack = stack.Find(name)->GetInfo();
				if (topStack.m_stack.GetCount()) {
					int stackLevel = topStack.m_stack.GetLast()->GetInfo();
					variable->m_label = instruction->MakeSSAName(name, stackLevel);
				}
			}
		}
		
		dCILInstr::dArg* const variable = instruction->GetGeneratedVariable();
		if (variable) {
			dString name(instruction->RemoveSSAPostfix(variable->m_label));
			dAssert(stack.Find(name));
			dStatementBucket& topStack = stack.Find(name)->GetInfo();

			int i = topStack.m_index;
			variable->m_label = instruction->MakeSSAName(name, i);
			topStack.m_stack.Append(i);
			topStack.m_index = i + 1;
		}
		//instruction->Trace();
	}

	for (dList<const dBasicBlock*>::dListNode* node = root->m_children.GetFirst(); node; node = node->GetNext()) {
		RenameVariables (node->GetInfo(), dataFlow, stack);
	}

	for (dCIL::dListNode* node = root->m_end; node != root->m_begin; node = node->GetPrev()) {
		dCILInstr* const instruction = node->GetInfo();
		const dCILInstr::dArg* const variable = instruction->GetGeneratedVariable();
		if (variable) {
			dString name (instruction->RemoveSSAPostfix(variable->m_label));
			dStatementBucket& topStack = stack.Find(name)->GetInfo();
			dAssert (topStack.m_stack.GetLast());
			topStack.m_stack.Remove(topStack.m_stack.GetLast());
		}
	}

//root->Trace();
//dataFlow->m_cil->Trace();
}