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
#include "dCIL.h"
#include "dDataFlowGraph.h"
#include "dRegisterInterferenceGraph.h"


dRegisterInterferenceGraph::dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount)
	:dTree<dRegisterInterferenceNode, dString>()
	,m_flowGraph(flowGraph)
	,m_spillCount(0)
	,m_localLayer(0)
	,m_registerCount(registerCount)
{
	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
	while (m_flowGraph->ApplyRemoveDeadCode());
/*
	for (int registersUsed = 100000; registersUsed > registerCount; ) {
		Build();
		registersUsed = ColorGraph (registerCount);
m_flowGraph->m_cil->Trace();
		if (registersUsed > registerCount) {
			EmitSpillStatements();
m_flowGraph->m_cil->Trace();
			
			m_flowGraph->BuildBasicBlockGraph();
			m_flowGraph->CalculateLiveInputLiveOutput ();
			RemoveAll();
m_flowGraph->m_cil->Trace();
		}
	}

	AllocateRegisters ();
	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
m_flowGraph->m_cil->Trace();
*/

	Build();
	int registersUsed = ColorGraph (registerCount);
	AllocateRegisters();
	if (registersUsed > registerCount) {
		// we have some spill fix the program.
//		AllocatedSpilledRegister(registersUsed, registerCount);
		EmitSpillStatements(registersUsed);
	}

	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
m_flowGraph->m_cil->Trace();

	for (bool optimized = true; optimized;) {
		optimized = false;
		m_flowGraph->UpdateReachingDefinitions();
		optimized |= m_flowGraph->ApplyCopyPropagation();
		m_flowGraph->m_cil->Trace();
		optimized |= m_flowGraph->ApplyRemoveDeadCode();
		m_flowGraph->m_cil->Trace();
	}


	for (bool optimized = true; optimized;) {
		optimized = false;
		m_flowGraph->UpdateReachingDefinitions();
		optimized |= m_flowGraph->ApplyCopyPropagation();
m_flowGraph->m_cil->Trace();
		optimized |= m_flowGraph->ApplyRemoveDeadCode();
m_flowGraph->m_cil->Trace();
	}
}


int dRegisterInterferenceGraph::GetRegisterIndex (const dString& varName) const
{
	int index = -1;
	if (varName[0] != '_') {
		dTreeNode* const node = Find (varName);
		dRegisterInterferenceNode& var = node->GetInfo();
		index = var.m_registerIndex;
	}
	return index;
}

dString dRegisterInterferenceGraph::GetRegisterName (const dString& varName) const
{
	int index = GetRegisterIndex (varName);
	if (index == -1) {
		return varName;
	} else {
		return IndexToRegister(index);
	}
}


void dRegisterInterferenceGraph::Build()
{
	dTree<dDataFlowGraph::dDataFlowPoint, dCIL::dListNode*>::Iterator iter (m_flowGraph->m_dataFlowGraph);
	for (iter.Begin(); iter; iter ++) {
		dDataFlowGraph::dDataFlowPoint& point = iter.GetNode()->GetInfo();

		dDataFlowGraph::dDataFlowPoint::dVariableSet<dString>::Iterator definedIter (point.m_generatedVariableSet);
		for (definedIter.Begin(); definedIter; definedIter ++) {
			const dString& variable = definedIter.GetKey();
			if (variable[0] != '_' ) {
				dTreeNode* interferanceGraphNode = Find(variable);
				if (!interferanceGraphNode) {
					interferanceGraphNode = Insert(variable);
					interferanceGraphNode->GetInfo().m_name = variable;
			dTrace (("%s\n", variable.GetStr()));
				}
			}
		}
	}

	// pre-color some special nodes
	dTreeNode* const returnRegNode = Find(GetReturnVariableName());
	if (returnRegNode) {
		dRegisterInterferenceNode& returnReginster = returnRegNode->GetInfo();
		returnReginster.m_registerIndex = D_RETURN_REGISTER_INDEX;
		m_flowGraph->m_returnVariableName = GetRegisterName (returnReginster.m_name);
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowGraph::dDataFlowPoint& info = iter.GetNode()->GetInfo();
		if (info.m_killVariable.Size()) {
			const dString& variableA = info.m_killVariable;

			if ((variableA[0] != '_')) {
				dTreeNode* const nodeA = Find(variableA);
				dAssert (nodeA);
				dRegisterInterferenceNode& inteferanceNodeA = nodeA->GetInfo();
				dDataFlowGraph::dDataFlowPoint::dVariableSet<dString>::Iterator aliveIter (info.m_liveOutputSet);

				for (aliveIter.Begin(); aliveIter; aliveIter ++) {
					const dString& variableB = aliveIter.GetKey();
					if ((variableB[0] != '_') && (variableA != variableB)) {
						dTreeNode* const nodeB = Find(variableB);
						dAssert (nodeB);
						dRegisterInterferenceNode& inteferanceNodeB = nodeB->GetInfo();

						bool hasEdge = false;
						for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = inteferanceNodeA.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
							if (ptr->GetInfo().m_targetNode == nodeB) {
								hasEdge = true;
								break;
							}
						}
						if (!hasEdge) {
							dList<dRegisterInterferenceNodeEdge>::dListNode* const entryA = inteferanceNodeA.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeB));
							dList<dRegisterInterferenceNodeEdge>::dListNode* const entryB = inteferanceNodeB.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeA));
							entryA->GetInfo().m_twin = &entryB->GetInfo();
							entryB->GetInfo().m_twin = &entryA->GetInfo();
						}
					}
				}
			}
		}
	}

	for (iter.Begin(); iter; iter ++) {
		dDataFlowGraph::dDataFlowPoint& info = iter.GetNode()->GetInfo();
		dTreeAdressStmt& stmt = info.m_statement->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_assigment) && (stmt.m_operator == dTreeAdressStmt::m_nothing)) {
			const dString& variableA = stmt.m_arg0.m_label;
			const dString& variableB = stmt.m_arg1.m_label;
			dTreeNode* const nodeA = Find(variableA);
			dTreeNode* const nodeB = Find(variableB);
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = nodeA->GetInfo().m_interferanceEdge.GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				if (edge.m_targetNode == nodeB) {
					edge.m_isMov = true;
					edge.m_twin->m_isMov = true;
					break;
				}
			}
		}
	}
}


dRegisterInterferenceGraph::dTreeNode* dRegisterInterferenceGraph::GetBestNode()
{
	int minCount = 0x7fffff;
	dTreeNode* bestNode = NULL;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dRegisterInterferenceNode& info = node->GetInfo();
		if (!info.m_inSet) {
			int count = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = info.m_interferanceEdge.GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				if (!edge.m_mark) {
					count ++;
				}
			}
			if (count < minCount) {
				minCount = count;
				bestNode = node;
			}
		}
	}

	dRegisterInterferenceNode& info = bestNode->GetInfo();
	info.m_inSet = true;
	return bestNode;
}


int dRegisterInterferenceGraph::ColorGraph (int registerCount)
{
	dList<dTreeNode*> registerOrder;
	for (int i = 0; i < GetCount(); i ++) {
		dTreeNode* const bestNode = GetBestNode();
		registerOrder.Addtop(bestNode);
		dRegisterInterferenceNode& node = bestNode->GetInfo();
		dList<dRegisterInterferenceNodeEdge>& edgeListInfo = node.m_interferanceEdge; 
		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfo.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			if (edge.m_mark == 0) {
				edge.m_mark = 1;
				edge.m_twin->m_mark = 1;
			}
		}
	} 

	int registersUsed = -1;
	for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
		dTreeNode* const varNode = node->GetInfo();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (variable.m_registerIndex == -1) {
			int regMask = 0;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = variable.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
				dRegisterInterferenceNode& otherVariable = edge.m_targetNode->GetInfo();
				if (otherVariable.m_registerIndex != -1) {
					regMask |= (1 << otherVariable.m_registerIndex);
				}
			}

			int index = 0;
			for (int mask = regMask; mask & 1 ; mask >>= 1) {
				index ++;
			}
			variable.m_registerIndex = index;
		}
		m_flowGraph->m_registersUsedMask |= (1 << variable.m_registerIndex);
		registersUsed = dMax (registersUsed, variable.m_registerIndex);
	}
	return registersUsed + 1;
}


void dRegisterInterferenceGraph::AllocateRegisters ()
{
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();	
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_assigment:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						stmt.m_arg2.m_label = GetRegisterName (stmt.m_arg2.m_label);
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_free:
			case dTreeAdressStmt::m_loadBase:
			case dTreeAdressStmt::m_storeBase:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				break;
			}


			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_store:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg2.m_label = GetRegisterName (stmt.m_arg2.m_label);
				} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}
				break;
			}


			case dTreeAdressStmt::m_alloc:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				break;
			}


			case dTreeAdressStmt::m_if:
			{
				stmt.m_arg0.m_label = GetRegisterName (stmt.m_arg0.m_label);
				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					stmt.m_arg1.m_label = GetRegisterName (stmt.m_arg1.m_label);
				}
				break;
			}

			case dTreeAdressStmt::m_enter:
			{
				int returnRegisterMask = ~((m_flowGraph->m_returnType == 1) ? (1 << D_RETURN_REGISTER_INDEX) : 0);
				int regMask = m_flowGraph->m_registersUsedMask & returnRegisterMask; 
				stmt.m_extraInformation = regMask;
				break;
			}

			case dTreeAdressStmt::m_leave:
			{
				int returnRegisterMask = ~((m_flowGraph->m_returnType == 1) ? (1 << D_RETURN_REGISTER_INDEX) : 0);
				int regMask = m_flowGraph->m_registersUsedMask & returnRegisterMask; 
				stmt.m_extraInformation = regMask;
				break;
			}

			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_argument:
			case dTreeAdressStmt::m_function:
				break;

			default:
				dAssert (0);
		}
	}
}

int dRegisterInterferenceGraph::Compare (const void* p1, const void* p2)
{
	int* const reg0 = (int*) p1;
	int* const reg1 = (int*) p2;

	if (reg0[0] > reg1[0]) {
		return -1;
	} else if (reg0[0] < reg1[0]) {
		return 1; 
	} 
	return 0;
}


int dRegisterInterferenceGraph::FindRegister(int regIndex, int totalRegisters) const
{
	for (int i = 0; i < totalRegisters; i ++) {
		if (m_registenFrequency[i][1] == regIndex) {
			return i;
		}
	}
	dAssert (0);
	return -1;
}


void dRegisterInterferenceGraph::SortRegisters(int totalRegisters)
{
	qsort (m_registenFrequency, totalRegisters, 2 * sizeof (int), Compare);
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		dAssert (0);
		int index = FindRegister (D_RETURN_REGISTER_INDEX, totalRegisters);
	}
}


void dRegisterInterferenceGraph::EmitSpillStatements(int totalRegisters)
{
	SortRegistersByFrequency(totalRegisters);
m_flowGraph->m_cil->Trace();

/*
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dTreeAdressStmt& stmt = node->GetInfo();
stmt.Trace();
		switch (stmt.m_instruction)
		{

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_loadBase:
			{
				SaveRegisterToTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				LoadRegisterFromTemp (node, stmt.m_arg1);
				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
//					LoadRegisterFromTemp (node, stmt.m_arg2);
				}
				SaveRegisterToTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				LoadRegisterFromTemp (node, stmt.m_arg2);
				break;
			}

			case dTreeAdressStmt::m_load:
			{
				LoadRegisterFromTemp (node, stmt.m_arg1);
				LoadRegisterFromTemp (node, stmt.m_arg2);
				SaveRegisterToTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_storeBase:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				break;
			}

			case dTreeAdressStmt::m_alloc:
			{
				dAssert (0);
				//point.m_killVariable = stmt.m_arg0.m_label;
				//point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}

			case dTreeAdressStmt::m_free:
			{
				dAssert (0);
				//point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}

			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
				break;

			default:
				dAssert (0);
		}
	}

	m_localLayer ++;
*/
}

dString dRegisterInterferenceGraph::MakeLocalVariable (int index)
{
	return  dString("_local") + dString(m_localLayer) + dString ('_') + dString(index);
}

dString dRegisterInterferenceGraph::MakeSpilledTemporary ()
{
	m_spillCount ++;
	return dString("spilled") + dString(m_spillCount - 1);
}


void dRegisterInterferenceGraph::SaveRegisterToTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument)
{
	//	int index = RegisterToIndex (argument.m_label);
	//	argument.m_label.Replace(0, 3, dString ("tmp"));
	int index = GetRegisterIndex (argument.m_label);
	if (index >= m_registerCount) {

		dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
		m_flowGraph->m_cil->InsertAfter (node, newNode);

		dString local (MakeLocalVariable (index));
		//dString spill (MakeSpilledTemporary());
		argument.m_label = MakeSpilledTemporary();

		dTreeAdressStmt& stmt = newNode->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_storeBase;
		stmt.m_operator = dTreeAdressStmt::m_nothing;
		stmt.m_arg0.m_label = argument.m_label;
		stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
		stmt.m_arg2.m_label = local;
		stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;

node->GetInfo().Trace();
stmt.Trace();
	}
}


void dRegisterInterferenceGraph::LoadRegisterFromTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument)
{
	//	int index = RegisterToIndex (argument.m_label);
	//	argument.m_label.Replace(0, 3, dString ("tmp"));
	int index = GetRegisterIndex (argument.m_label);
	if (index >= m_registerCount) {

		dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
		m_flowGraph->m_cil->InsertAfter (node->GetPrev(), newNode);

		dString local (MakeLocalVariable (index));
		argument.m_label = MakeSpilledTemporary();

		dTreeAdressStmt& stmt = newNode->GetInfo();
		stmt.m_instruction = dTreeAdressStmt::m_loadBase;
		stmt.m_arg0.m_label = argument.m_label;
		stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
		stmt.m_arg2.m_label = local;
		stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;

stmt.Trace();
node->GetInfo().Trace();

	}
}

void dRegisterInterferenceGraph::RemapRegister(dTreeAdressStmt::dArg& arg, int totalRegisters)
{
	int index = RegisterToIndex (arg.m_label);
	int remapIndex = FindRegister (index, totalRegisters);
	arg.m_label = IndexToRegister(remapIndex);
}



void dRegisterInterferenceGraph::SortRegistersByFrequency(int totalRegisters)
{
	memset (m_registenFrequency, 0, sizeof (m_registenFrequency)) ;

	for (int i = 0; i < totalRegisters; i ++) {
		m_registenFrequency[i][0] = 0;
		m_registenFrequency[i][1] = i;
	}

	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = node->GetNext()) {
		const dTreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_assigment:
			{
				m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					m_registenFrequency[RegisterToIndex (stmt.m_arg1.m_label)][0] += 1;
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						m_registenFrequency[RegisterToIndex (stmt.m_arg2.m_label)][0] += 1;
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_store:
			{
				m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;
				m_registenFrequency[RegisterToIndex (stmt.m_arg1.m_label)][0] += 1;
				m_registenFrequency[RegisterToIndex (stmt.m_arg2.m_label)][0] += 1;
				break;
			}


			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_loadBase:
			case dTreeAdressStmt::m_storeBase:
			{
				m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				if (stmt.m_arg0.m_type == dTreeAdressStmt::m_intVar) {
					m_registenFrequency[RegisterToIndex (stmt.m_arg0.m_label)][0] += 1;
				}

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					m_registenFrequency[RegisterToIndex (stmt.m_arg1.m_label)][0] += 1;
				}
				break;
			}


			case dTreeAdressStmt::m_alloc:
			{
				dAssert (0);
//				point.m_killVariable = stmt.m_arg0.m_label;
//				point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}
			case dTreeAdressStmt::m_free:
			{
				dAssert (0);
//				point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}

			
			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}

	SortRegisters(totalRegisters);


	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = node->GetNext()) {
		dTreeAdressStmt& stmt = node->GetInfo();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_loadBase:
			case dTreeAdressStmt::m_storeBase:
			{
				RemapRegister(stmt.m_arg0, totalRegisters);
				break;
			}

			case dTreeAdressStmt::m_load:
			case dTreeAdressStmt::m_store:
			{
				RemapRegister(stmt.m_arg0, totalRegisters);
				RemapRegister(stmt.m_arg1, totalRegisters);
				RemapRegister(stmt.m_arg2, totalRegisters);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				RemapRegister(stmt.m_arg0, totalRegisters);

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					RemapRegister(stmt.m_arg1, totalRegisters);
				} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
					dAssert (0);
				}

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
						RemapRegister(stmt.m_arg2, totalRegisters);
					} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}
				}
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				if (stmt.m_arg0.m_type == dTreeAdressStmt::m_intVar) {
					RemapRegister(stmt.m_arg0, totalRegisters);
				}

				if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
					RemapRegister(stmt.m_arg1, totalRegisters);
				}
				break;
			}

			case dTreeAdressStmt::m_alloc:
			{
				dAssert (0);
				//point.m_killVariable = stmt.m_arg0.m_label;
				//point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}
			case dTreeAdressStmt::m_free:
			{
				dAssert (0);
				//point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}

			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}
}

/*
void dRegisterInterferenceGraph::AllocatedSpilledRegister(int totalRegisters, int registerCount)
{
	SortRegistersByFrequency(totalRegisters, registerCount);
m_flowGraph->m_cil->Trace();

	int index0 = registerCount - 1;
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		index0 = (index0 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
	}
	int index1 = index0 - 1;
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		index1 = (index1 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
	}
	int index2 = index1 - 1;
	if (m_flowGraph->m_returnType != dCIL::m_void) {
		index2 = (index2 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
	}

	m_reg0 = dString(IndexToRegister(index0));
	m_reg1 = dString (IndexToRegister(index1));
	m_reg2 = dString (IndexToRegister(index2));

	m_local0 = dString (dString("_local") + dString(index0));
	m_local1 = dString (dString("_local") + dString(index1));
	m_local2 = dString (dString("_local") + dString(index2));

	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dTreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_loadBase:
			{
				SaveRegisterToTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				SaveRegisterToTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);

				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					LoadRegisterFromTemp (node, stmt.m_arg2);
				}
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				LoadRegisterFromTemp (node, stmt.m_arg2);
				break;
			}

			case dTreeAdressStmt::m_load:
			{
				SaveRegisterToTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				LoadRegisterFromTemp (node, stmt.m_arg2);
				break;
			}

			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_storeBase:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				LoadRegisterFromTemp (node, stmt.m_arg0);
				LoadRegisterFromTemp (node, stmt.m_arg1);
				break;
			}

			case dTreeAdressStmt::m_alloc:
			{
				dAssert (0);
				//point.m_killVariable = stmt.m_arg0.m_label;
				//point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}
			case dTreeAdressStmt::m_free:
			{
				dAssert (0);
				//point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}

			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_enter:
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}

m_flowGraph->m_cil->Trace();


	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dTreeAdressStmt& stmt = node->GetInfo();
//stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_loadBase:
			{
				SaveSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				break;
			}

			case dTreeAdressStmt::m_assigment:
			{
				SaveSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					LoadSpillRegister (node, m_reg2, stmt.m_arg2, registerCount);
				}
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				LoadSpillRegister (node, m_reg2, stmt.m_arg2, registerCount);
				break;
			}

			case dTreeAdressStmt::m_load:
			{
				SaveSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				LoadSpillRegister (node, m_reg2, stmt.m_arg2, registerCount);
				break;
			}

			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_storeBase:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0, registerCount);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1, registerCount);
				break;
			}

			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_enter:
			{
				stmt.m_extraInformation = m_flowGraph->m_registersUsedMask & ((1 << registerCount) - 1);
				break;
			}


			case dTreeAdressStmt::m_alloc:
			{
				dAssert (0);
				//point.m_killVariable = stmt.m_arg0.m_label;
				//point.m_generatedVariableSet.Insert(stmt.m_arg1.m_label);
				break;
			}
			case dTreeAdressStmt::m_free:
			{
				dAssert (0);
				//point.m_generatedVariableSet.Insert(stmt.m_arg0.m_label);
				break;
			}

			case dTreeAdressStmt::m_nop:
			case dTreeAdressStmt::m_ret:
			case dTreeAdressStmt::m_goto:
			case dTreeAdressStmt::m_call:
			case dTreeAdressStmt::m_label:
			case dTreeAdressStmt::m_function:
			case dTreeAdressStmt::m_argument:
			break;
	
			default:
				dAssert (0);
		}
	}

m_flowGraph->m_cil->Trace();
}


void dRegisterInterferenceGraph::SaveSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument, int registerCount)
{
	int index = RegisterToIndex (argument.m_label);
	if (index >= registerCount) {
		argument.m_label = alliasRegister;
		SaveRegisterToTemp (node, alliasRegister, IndexToLocal(index));
	}
}

void dRegisterInterferenceGraph::LoadSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument, int registerCount)
{
	int index = RegisterToIndex (argument.m_label);
	if (index >= registerCount) {
		LoadRegisterFromTemp (node, alliasRegister, IndexToLocal(index));
		argument.m_label = alliasRegister;
	}
}


void dRegisterInterferenceGraph::SaveRegisterToTemp(dCIL::dListNode* const node, const dString& reg, const dString& local)
{
	dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
	m_flowGraph->m_cil->InsertAfter (node, newNode);

	dTreeAdressStmt& stmt = newNode->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_storeBase;
	stmt.m_operator = dTreeAdressStmt::m_nothing;
	stmt.m_arg0.m_label = local;
	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
	stmt.m_arg2.m_label = reg;
	stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;

//	dTreeAdressStmt& stmt = newNode->GetInfo();
//	stmt.m_instruction = dTreeAdressStmt::m_assigment;
//	stmt.m_operator = dTreeAdressStmt::m_nothing;
//	stmt.m_arg0.m_label = local;
//	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
//	stmt.m_arg1.m_label = reg;
//	stmt.m_arg1.m_type = dTreeAdressStmt::m_intVar;
}



void dRegisterInterferenceGraph::LoadRegisterFromTemp(dCIL::dListNode* const node, const dString& reg, const dString& local)
{
	dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
	m_flowGraph->m_cil->InsertAfter (node->GetPrev(), newNode);

	dTreeAdressStmt& stmt = newNode->GetInfo();
	stmt.m_instruction = dTreeAdressStmt::m_loadBase;
	stmt.m_arg0.m_label = reg;
	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
	stmt.m_arg2.m_label = local;
	stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;


//	dTreeAdressStmt& stmt = newNode->GetInfo();
//	stmt.m_instruction = dTreeAdressStmt::m_assigment;
//	stmt.m_operator = dTreeAdressStmt::m_nothing;
//	stmt.m_arg0.m_label = reg;
//	stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
//	stmt.m_arg1.m_label = local;
//	stmt.m_arg1.m_type = dTreeAdressStmt::m_intVar;
}













*/

