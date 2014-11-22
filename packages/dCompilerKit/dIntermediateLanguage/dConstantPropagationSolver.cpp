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
#include "dBasicBlocksGraph.h"
#include "dCILInstrLoadStore.h"
#include "dConstantPropagationSolver.h"


dConstantPropagationSolver::dInstructionMap::dInstructionMap():dTree<int, dCILInstr*>()
{
}

dConstantPropagationSolver::dVariable::dVariable(dCILInstr* const instruction, dCILInstr::dArg* const variable)
	:m_type(m_undefined)
	,m_constValue("")
	,m_instruction(instruction)
	,m_variable(variable)
{
}

dConstantPropagationSolver::dVariable& dConstantPropagationSolver::dVariable::operator=(const dVariable& copy)
{
	dAssert(0);
	return *this;
}



dConstantPropagationSolver::dBlockEdgeKey::dBlockEdgeKey(const dBasicBlock* const blockHigh, const dBasicBlock* const blockLow)
	:m_blockHigh(blockHigh)
	,m_blockLow(blockLow)
{
}



bool dConstantPropagationSolver::dBlockEdgeKey::operator<(const dBlockEdgeKey& src) const
{
	if (m_blockHigh < src.m_blockHigh) {
		return true;
	}
	return m_blockLow < src.m_blockLow;
}

bool dConstantPropagationSolver::dBlockEdgeKey::operator>(const dBlockEdgeKey& src) const
{
	if (m_blockHigh > src.m_blockHigh) {
		return true;
	}
	return m_blockLow > src.m_blockLow;
}




dConstantPropagationSolver::dConstantPropagationSolver (dBasicBlocksGraph* const graph)
	:m_graph(graph)
{
	for (dCIL::dListNode* node = m_graph->m_begin; node != m_graph->m_end; node = node->GetNext()) {
		dCILInstr* const instruction = node->GetInfo();
//instruction->Trace();
		dCILInstr::dArg* const variable = instruction->GetGeneratedVariable();
		if (variable) {
			m_variablesList.Insert(dVariable(instruction, variable), variable->m_label);
		}

		dList<dCILInstr::dArg*> variablesList;
		instruction->GetUsedVariables(variablesList);
		for (dList<dCILInstr::dArg*>::dListNode* varNode = variablesList.GetFirst(); varNode; varNode = varNode->GetNext()) {
			dCILInstr::dArg* const arg = varNode->GetInfo();
			dTree<dInstructionMap, dString>::dTreeNode* bucketNode = m_uses.Find(arg->m_label);
			if (!bucketNode) {
				bucketNode =  m_uses.Insert(arg->m_label);
			}
			bucketNode->GetInfo().Insert (instruction);
		}
	}
/*
	for (dCIL::dListNode* node = m_begin; node != m_end; node = node->GetNext()) {
		dCILInstr* const instruction = node->GetInfo();
		if (instruction->GetAsArgument()) {
			dAssert (0);
	//instruction->Trace();
			dList<dCILInstr::dArg*> variables;
			instruction->GetUsedVariables(variables);
			for (dList<dCILInstr::dArg*>::dListNode* varNode = variables.GetFirst(); varNode; varNode = varNode->GetNext()) {
				dCILInstr::dArg* const variable = varNode->GetInfo();
				dTree<dConstantPropagationVariable, dString>::dTreeNode* const varNodeEntry = variablesList.Insert(dConstantPropagationVariable(instruction, variable), variable->m_label);
				dAssert (varNodeEntry);
				varNodeEntry->GetInfo().m_value = dConstantPropagationVariable::m_varyngValue;
			}
		}
	}


	dTree<int, dBlockEdgeKey> blockEdges;
	for (dBasicBlocksGraph::dListNode* node = GetFirst(); node; node = node->GetNext()) {
		const dBasicBlock* const block = &node->GetInfo();
//block->Trace();
		for (dList<const dBasicBlock*>::dListNode* succNode = block->m_successors.GetFirst(); succNode; succNode = succNode->GetNext()) {
			const dBasicBlock* const succ = succNode->GetInfo();
//succ->Trace();
			blockEdges.Insert(0, dBlockEdgeKey(block, succ));
		}
	}
*/
}



void dConstantPropagationSolver::Trace()
{
	for (dCIL::dListNode* node = m_graph->m_begin; node != m_graph->m_end; node = node->GetNext()) {
		dCILInstr* const instruction = node->GetInfo();
		dList<dCILInstr::dArg*> variablesList;
		instruction->GetUsedVariables(variablesList);
		if (variablesList.GetCount()) {
			char text[4096];
			instruction->Serialize(text);
			dString str (text);
			for (dList<dCILInstr::dArg*>::dListNode* varNode = variablesList.GetFirst(); varNode; varNode = varNode->GetNext()) {
				dCILInstr::dArg* const arg = varNode->GetInfo();
				dAssert (m_variablesList.Find(arg->m_label));
				dVariable& variable = m_variablesList.Find(arg->m_label)->GetInfo();
				if (variable.m_type == dVariable::m_constant) {
					int index = str.Find (arg->m_label);
					if (index > 0) {
						str.Replace(index, arg->m_label.Size(), variable.m_constValue);
					}
				}
			}

			dTrace ((str.GetStr()));
		} else {
			instruction->Trace();
		}
	}

	dTrace (("\n"));
}

void dConstantPropagationSolver::UpdateLatice (const dCILInstr::dArg& arg, const dString& newValue, dVariable::dValueTypes type)
{
	dAssert (m_variablesList.Find(arg.m_label));
	dVariable& variable = m_variablesList.Find(arg.m_label)->GetInfo();
	if (type != variable.m_type) {
		variable.m_type = type;
		variable.m_constValue = newValue;
		
		dTree<dInstructionMap, dString>::dTreeNode* const usesNode = m_uses.Find(arg.m_label);
		if (usesNode) {
			dInstructionMap::Iterator iter(usesNode->GetInfo());
			for (iter.Begin(); iter; iter++) {
				dCILInstr* const instruction = iter.GetKey();
//		instruction->Trace();
				m_instructionsWorklist.Append (instruction);
			}
		}
		Trace();
	}
}


bool dConstantPropagationSolver::Solve ()
{
m_graph->Trace();

	m_blockWorklist.Append(&m_graph->GetFirst()->GetInfo());

	while (m_blockWorklist.GetCount() || m_instructionsWorklist.GetCount()) {
Trace();
		while (m_instructionsWorklist.GetCount()) {
			dList<dCILInstr*>::dListNode* const node = m_instructionsWorklist.GetFirst();
			dCILInstr* const instruction = node->GetInfo();
			m_instructionsWorklist.Remove(node);
instruction->Trace();
			instruction->ApplyConstantPropagationSSA(*this);
		}

		while (m_blockWorklist.GetCount()) {
			dList<dBasicBlock*>::dListNode* const node = m_blockWorklist.GetFirst();
			dBasicBlock* const block = node->GetInfo();
			m_blockWorklist.Remove(node);
block->Trace();

			bool done = false;
			for (dCIL::dListNode* node = block->m_begin; !done; node = node->GetNext()) {
				done = (node == block->m_end);
				dCILInstrPhy* const phy = node->GetInfo()->GetAsPhy();
				if (phy) {
//phy->Trace();
					phy->ApplyConstantPropagationSSA (*this);
				}
			}

			if (!m_visited.Find(block)) {
				m_visited.Insert(block);
				bool done = false;
				for (dCIL::dListNode* node = block->m_begin; !done; node = node->GetNext()) {
					done = (node == block->m_end);
					dCILInstr* const instruction = node->GetInfo();
					
instruction->Trace();
					instruction->ApplyConstantPropagationSSA (*this);
				}
			}
		}
	}
	return false;
}





