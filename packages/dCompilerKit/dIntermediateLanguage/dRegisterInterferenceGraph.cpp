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


#define D_SPILL_WEIGHT_FACTOR	10
#define D_MOVE_WEIGHT_FACTOR	20

dRegisterInterferenceGraph::dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount)
	:dTree<dRegisterInterferenceNode, dString>()
	,m_flowGraph(flowGraph)
	,m_spillCount(0)
	,m_registerCount(registerCount)
{
	m_flowGraph->BuildBasicBlockGraph();
	m_flowGraph->CalculateLiveInputLiveOutput ();
	while (m_flowGraph->ApplyRemoveDeadCode());

	Build();
	while (ColorGraph () > m_registerCount) {
		// we have a spill, find a good spill node and try graph coloring again
		SelectSpillVariableAndReWriteFunction();
	}
/*
	int registersUsed = ColorGraph (0x7fffffff);
	if (registersUsed > registerCount) {
		SpillAndColorGraph ();
	}

	AllocateRegisters();


	
m_flowGraph->m_cil->Trace();
	if (registersUsed > registerCount) {
		// we have some spill fix the program.
		//EmitSpillStatements(registersUsed);
		SortRegistersByFrequency(registersUsed);
m_flowGraph->m_cil->Trace();

		MakeWorkingRegisters (registersUsed);
		AllocatedSpilledRegister(registersUsed);
	}
*/

	AllocateRegisters();
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
}

/*








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


void dRegisterInterferenceGraph::MakeWorkingRegisters(int totalRegisters)
{
    int index0 = m_registerCount - 1;
    if (m_flowGraph->m_returnType != dCIL::m_void) {
        index0 = (index0 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
    }
    int index1 = index0 - 1;
    if (m_flowGraph->m_returnType != dCIL::m_void) {
        index1 = (index1 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
    }
    //  int index2 = index1 - 1;
    //  if (m_flowGraph->m_returnType != dCIL::m_void) {
    //    index2 = (index2 == D_RETURN_REGISTER_INDEX) ? 1 : 0;
    //  }

    m_reg0 = dString(IndexToRegister(index0));
    m_reg1 = dString (IndexToRegister(index1));
    //  m_reg2 = dString (IndexToRegister(index2));

    m_local0 = IndexToLocal(index0);
    m_local1 = IndexToLocal(index1);
    //  m_local2 = IndexToLocal(index2);


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
                LoadRegisterFromTemp (node, stmt.m_arg1);
                if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
                    LoadRegisterFromTemp (node, stmt.m_arg2);
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
}


dString dRegisterInterferenceGraph::MakeSpilledTemporary ()
{
	m_spillCount ++;
	return dString(D_SPILL_REGISTER_SYMBOL) + dString(m_spillCount - 1);
}


void dRegisterInterferenceGraph::SaveRegisterToTemp(dCIL::dListNode* const node, const dString& reg, const dString& local)
{
    dCIL::dListNode* const newNode = m_flowGraph->m_cil->NewStatement();
    m_flowGraph->m_cil->InsertAfter (node, newNode);

    dTreeAdressStmt& stmt = newNode->GetInfo();
    stmt.m_instruction = dTreeAdressStmt::m_storeBase;
    stmt.m_operator = dTreeAdressStmt::m_nothing;
    stmt.m_arg0.m_label = reg;
    stmt.m_arg0.m_type = dTreeAdressStmt::m_intVar;
    stmt.m_arg2.m_label = local;
    stmt.m_arg2.m_type = dTreeAdressStmt::m_intVar;
}


void dRegisterInterferenceGraph::SaveRegisterToTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument)
{
    if (argument.m_label == m_reg0) {
        SaveRegisterToTemp(node, m_reg0, m_local0);
    } else if (argument.m_label == m_reg1) {
        SaveRegisterToTemp(node, m_reg1, m_local1);
//  } else if (argument.m_label == m_reg2) {
//        SaveRegisterToTemp(node, m_reg2, m_local2);
    }
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
}


void dRegisterInterferenceGraph::LoadRegisterFromTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument)
{
    if (argument.m_label == m_reg0) {
        LoadRegisterFromTemp(node, m_reg0, m_local0);
    } else if (argument.m_label == m_reg1) {
        LoadRegisterFromTemp(node, m_reg1, m_local1);
//    } else if (argument.m_label == m_reg2) {
//        LoadRegisterFromTemp(node, m_reg2, m_local2);
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

void dRegisterInterferenceGraph::SaveSpillRegister(dCIL::dListNode* const node, const dString& workingRegister, dTreeAdressStmt::dArg& argument)
{
	int index = RegisterToIndex (argument.m_label);
	if (index >= (m_registerCount - 2)) {
		argument.m_label = workingRegister;
		SaveRegisterToTemp (node, workingRegister, IndexToLocal(index));

node->GetInfo().Trace();
node->GetNext()->GetInfo().Trace();

	}
}

void dRegisterInterferenceGraph::LoadSpillRegister(dCIL::dListNode* const node, const dString& workingRegister, dTreeAdressStmt::dArg& argument)
{
	int index = RegisterToIndex (argument.m_label);
	if (index >= (m_registerCount - 2)) {
		LoadRegisterFromTemp (node, workingRegister, IndexToLocal(index));
		argument.m_label = workingRegister;

node->GetPrev()->GetInfo().Trace();
node->GetInfo().Trace();

	}
}


void dRegisterInterferenceGraph::AllocatedSpilledRegister(int totalRegisters)
{
m_flowGraph->m_cil->Trace();

    dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = m_flowGraph->m_function; node; node = nextNode) {
		nextNode = node->GetNext();
		dTreeAdressStmt& stmt = node->GetInfo();
stmt.Trace();
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_leave:
			case dTreeAdressStmt::m_enter:
			{
				stmt.m_extraInformation = m_flowGraph->m_registersUsedMask & ((1 << m_registerCount) - 1);
				break;
			}

			case dTreeAdressStmt::m_pop:
			case dTreeAdressStmt::m_loadBase:
			{
				SaveSpillRegister (node, m_reg0, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_push:
			case dTreeAdressStmt::m_storeBase:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0);
				break;
			}


			case dTreeAdressStmt::m_assigment:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg1);
				if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
					LoadSpillRegister (node, m_reg1, stmt.m_arg2);
				}
				SaveSpillRegister (node, m_reg0, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_load:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg1);
				LoadSpillRegister (node, m_reg1, stmt.m_arg2);
				SaveSpillRegister (node, m_reg0, stmt.m_arg0);
				break;
			}

			case dTreeAdressStmt::m_if:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1);
				break;
			}

			case dTreeAdressStmt::m_store:
			{
				LoadSpillRegister (node, m_reg0, stmt.m_arg0);
				LoadSpillRegister (node, m_reg1, stmt.m_arg1);
				LoadSpillRegister (node, m_reg0, stmt.m_arg2);
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
}
*/

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

		dDataFlowGraph::dDataFlowPoint::dVariableSet<dString>::Iterator definedIter (point.m_usedVariableSet);
		for (definedIter.Begin(); definedIter; definedIter ++) {
			const dString& variable = definedIter.GetKey();
			if (variable[0] != '_' ) {
				dTreeNode* interferanceGraphNode = Find(variable);
				if (!interferanceGraphNode) {
					interferanceGraphNode = Insert(variable);
					interferanceGraphNode->GetInfo().m_name = variable;
//dTrace (("%s\n", variable.GetStr()));
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
		if (info.m_generatedVariable.Size()) {
			const dString& variableA = info.m_generatedVariable;

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
							//dRegisterInterferenceNodeEdge* const twin = ptr->GetInfo().m_twin;
							dList<dRegisterInterferenceNodeEdge>::dListNode* const twinEdgeNode = ptr->GetInfo().m_twin;
							if (twinEdgeNode && (twinEdgeNode->GetInfo().m_incidentNode == nodeB)) {
								hasEdge = true;
								break;
							}
						}
						if (!hasEdge) {
							dList<dRegisterInterferenceNodeEdge>::dListNode* const entryA = inteferanceNodeA.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeA));
							dList<dRegisterInterferenceNodeEdge>::dListNode* const entryB = inteferanceNodeB.m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeB));
							entryA->GetInfo().m_twin = entryB;
							entryB->GetInfo().m_twin = entryA;
						}
					}
				}
			}
		}
	}

//m_flowGraph->m_cil->Trace();

	dList<dTreeNode*> moveNodes;
	for (iter.Begin(); iter; iter ++) {
		dDataFlowGraph::dDataFlowPoint& info = iter.GetNode()->GetInfo();
		dTreeAdressStmt& stmt = info.m_statement->GetInfo();
		if ((stmt.m_instruction == dTreeAdressStmt::m_assigment) && (stmt.m_operator == dTreeAdressStmt::m_nothing) && (stmt.m_arg1.m_type != dTreeAdressStmt::m_intConst) &&  (stmt.m_arg1.m_type != dTreeAdressStmt::m_floatConst)) {

//stmt.Trace();
			const dString& variableA = stmt.m_arg0.m_label;
			const dString& variableB = stmt.m_arg1.m_label;
			dTreeNode* const nodeA = Find(variableA);
			dTreeNode* const nodeB = Find(variableB);

			bool canCoalese = true;
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* edgeNode = nodeA->GetInfo().m_interferanceEdge.GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = edgeNode->GetInfo();
				dList<dRegisterInterferenceNodeEdge>::dListNode* const twinEdgeNode = edge.m_twin;
				if (twinEdgeNode->GetInfo().m_incidentNode == nodeB) {
					canCoalese = false;
					break;
				}
			}
			if (canCoalese) {
				nodeA->GetInfo().m_isMove = true;
				nodeB->GetInfo().m_isMove = true;
				if (nodeB == returnRegNode) {
					m_coalescedNodes.Append(dCoalescedNodePair (nodeB, nodeA));
				} else {
					m_coalescedNodes.Append(dCoalescedNodePair (nodeA, nodeB));
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
			int count = info.m_isMove ? 4096 : 0;
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

bool dRegisterInterferenceGraph::CoalesceNodesRule1(dTreeNode* const nodePairA, dTreeNode* const nodePairB)
{
	dTree<dTreeNode*, dTreeNode*> edgeUnion;
	dRegisterInterferenceNode& nodeA = nodePairA->GetInfo();
	dList<dRegisterInterferenceNodeEdge>& edgeListInfoA = nodeA.m_interferanceEdge; 
	for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoA.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
		dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
		if (!otherNode->GetInfo().m_inSet) {
			edgeUnion.Insert(otherNode, otherNode);
		}
	}

	dRegisterInterferenceNode& nodeB = nodePairB->GetInfo();
	dList<dRegisterInterferenceNodeEdge>& edgeListInfoB = nodeB.m_interferanceEdge; 
	for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoB.GetFirst(); ptr; ptr = ptr->GetNext()) {
		dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
		dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
		if (!otherNode->GetInfo().m_inSet) {
			edgeUnion.Insert(otherNode, otherNode);
		}
	}

	if (edgeUnion.GetCount() < m_registerCount) {
		nodeB.m_registerIndex = nodeA.m_registerIndex;
		nodeB.m_coalescedParent = nodePairA;

		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoB.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			edge.m_mark = true;
			edge.m_twin->GetInfo().m_mark = true;
		}

		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoA.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
			edgeUnion.Remove(otherNode);
		}

		dTree<dTreeNode*, dTreeNode*>::Iterator iter (edgeUnion);
		for (iter.Begin(); iter; iter ++) {
			dTreeNode* const nodeA = nodePairA;
			dTreeNode* const nodeB = iter.GetNode()->GetInfo();

			dList<dRegisterInterferenceNodeEdge>::dListNode* const entryA = nodeA->GetInfo().m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeA));
			dList<dRegisterInterferenceNodeEdge>::dListNode* const entryB = nodeB->GetInfo().m_interferanceEdge.Append(dRegisterInterferenceNodeEdge (nodeB));
			entryA->GetInfo().m_twin = entryB;
			entryB->GetInfo().m_twin = entryA;
		}
		return true;
	}
	return false;

}

bool dRegisterInterferenceGraph::CoalesceNodesRule2(dTreeNode* const nodeA, dTreeNode* const nodeB)
{
	return false;
}


void dRegisterInterferenceGraph::CoalesceNodes()
{
	for (bool hasCoalesced = true; hasCoalesced; ) {
		hasCoalesced = false;
		dList<dCoalescedNodePair>::dListNode* nextNode;
		for (dList<dCoalescedNodePair>::dListNode* coalsescedNodePair = m_coalescedNodes.GetFirst(); coalsescedNodePair; coalsescedNodePair = nextNode) {
			const dCoalescedNodePair& pair = coalsescedNodePair->GetInfo();
			nextNode = coalsescedNodePair->GetNext();

			bool coalesced = false;
			dRegisterInterferenceNode& nodeA = pair.m_nodeA->GetInfo();
			dList<dRegisterInterferenceNodeEdge>& edgeListInfoA = nodeA.m_interferanceEdge; 
			for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfoA.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
				dTreeNode* const otherNode = edge.m_twin->GetInfo().m_incidentNode;
				if (pair.m_nodeB == otherNode) {
					coalesced = true;
					break;
				}
			}

			coalesced = coalesced || CoalesceNodesRule1 (pair.m_nodeA, pair.m_nodeB) || CoalesceNodesRule2 (pair.m_nodeA, pair.m_nodeB);
			if (coalesced) {
				m_coalescedNodes.Remove(coalsescedNodePair);
			}
			hasCoalesced |= coalesced;
		}
	}
}

int dRegisterInterferenceGraph::ColorGraph ()
{
	dList<dTreeNode*> registerOrder;
	for (int i = 0; i < GetCount(); i ++) {
		CoalesceNodes();

		dTreeNode* const bestNode = GetBestNode();
		registerOrder.Addtop(bestNode);
		dRegisterInterferenceNode& node = bestNode->GetInfo();
		dList<dRegisterInterferenceNodeEdge>& edgeListInfo = node.m_interferanceEdge; 
		for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = edgeListInfo.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
			edge.m_mark = true;
			edge.m_twin->GetInfo().m_mark = true;
		}

		if (node.m_isMove) {
			dList<dCoalescedNodePair>::dListNode* nextNode;
			for (dList<dCoalescedNodePair>::dListNode* coalsescedNodePair = m_coalescedNodes.GetFirst(); coalsescedNodePair; coalsescedNodePair = nextNode) {
				const dCoalescedNodePair& pair = coalsescedNodePair->GetInfo();
				nextNode = coalsescedNodePair->GetNext();
				if ((bestNode == pair.m_nodeA) || (bestNode == pair.m_nodeB)) {
					m_coalescedNodes.Remove(coalsescedNodePair);
				}
			}
		}
	} 

	int registersUsed = -1;
	for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
		dTreeNode* const varNode = node->GetInfo();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (!variable.m_coalescedParent) {
			if (variable.m_registerIndex == -1) {
				int regMask = 0;
				for (dList<dRegisterInterferenceNodeEdge>::dListNode* ptr = variable.m_interferanceEdge.GetFirst(); ptr; ptr = ptr->GetNext()) {
					dRegisterInterferenceNodeEdge& edge = ptr->GetInfo();
					dRegisterInterferenceNode& otherVariable = edge.m_twin->GetInfo().m_incidentNode->GetInfo();
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
	}

	for (dList<dTreeNode*>::dListNode* node = registerOrder.GetFirst(); node; node = node->GetNext()) {
		dTreeNode* const varNode = node->GetInfo();
		dRegisterInterferenceNode& variable = varNode->GetInfo();
		if (variable.m_coalescedParent) {
			dTreeNode* const parentNode = (dTreeNode*)variable.m_coalescedParent;
			dRegisterInterferenceNode& parentVariable = parentNode->GetInfo();
			variable.m_registerIndex = parentVariable.m_registerIndex;
		}
	}
	return registersUsed + 1;
}


void dRegisterInterferenceGraph::SelectSpillVariableAndReWriteFunction()
{
	dTree<dVariableSpillPriority,dString> spillPriority;
	dTree<dDataFlowGraph::dDataFlowPoint, dCIL::dListNode*>::Iterator stmtIter (m_flowGraph->m_dataFlowGraph);

	for (stmtIter.Begin(); stmtIter; stmtIter ++) {
		dDataFlowGraph::dDataFlowPoint& point = stmtIter.GetNode()->GetInfo();
		dTreeAdressStmt& stmt = point.m_statement->GetInfo();	
		switch (stmt.m_instruction)
		{
			case dTreeAdressStmt::m_loadBase:
			case dTreeAdressStmt::m_storeBase:
			{
				dAssert (point.m_generatedVariable.GetStr());
				dTree<dVariableSpillPriority,dString>::dTreeNode* node = spillPriority.Find(point.m_generatedVariable);
				if (!node) {
					node = spillPriority.Insert(point.m_generatedVariable);
				}
				node->GetInfo().m_useCount += 1;
				break;
			}

			default:
			{
				if (point.m_generatedVariable.GetStr()) {
					dTree<dVariableSpillPriority,dString>::dTreeNode* node = spillPriority.Find(point.m_generatedVariable);
					if (!node) {
						node = spillPriority.Insert(point.m_generatedVariable);
					}
					node->GetInfo().m_useCount += 1;
				}
				
				dDataFlowGraph::dDataFlowPoint::dVariableSet<dString>::Iterator usedIter (point.m_usedVariableSet);
				for (usedIter.Begin(); usedIter; usedIter ++) {
					const dString& key = usedIter.GetKey();
					dTree<dVariableSpillPriority,dString>::dTreeNode* node = spillPriority.Find(key);
					if (!node) {
						node = spillPriority.Insert(key);
					}
					node->GetInfo().m_useCount += 1;
				}
			}
		}
	}



	dList<dDataFlowGraph::dLoop> loops;
	m_flowGraph->GetLoops (loops);
	for (dList<dDataFlowGraph::dLoop>::dListNode* loopNode = loops.GetFirst(); loopNode; loopNode = loopNode->GetNext()) {
		dDataFlowGraph::dLoop& loop = loopNode->GetInfo();

		dCIL::dListNode* node = loop.m_head;
		do {

			dTreeAdressStmt& stmt = node->GetInfo();	
stmt.Trace();
			switch (stmt.m_instruction)
			{

				case dTreeAdressStmt::m_assigment:
				{
					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg0.m_label);
					if (priorityNode) {
						priorityNode->GetInfo().m_useCount --;
						priorityNode->GetInfo().m_loopUseCount ++;
					}

					if (stmt.m_arg1.m_type == dTreeAdressStmt::m_intVar) {
						dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg1.m_label);
						if (priorityNode) {
							priorityNode->GetInfo().m_useCount --;
							priorityNode->GetInfo().m_loopUseCount ++;
						}
					} else if (stmt.m_arg1.m_type == dTreeAdressStmt::m_floatVar) {
						dAssert (0);
					}

					if (stmt.m_operator != dTreeAdressStmt::m_nothing) {
						if (stmt.m_arg2.m_type == dTreeAdressStmt::m_intVar) {
							dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(stmt.m_arg2.m_label);
							if (priorityNode) {
								priorityNode->GetInfo().m_useCount --;
								priorityNode->GetInfo().m_loopUseCount ++;
							}
						} else if (stmt.m_arg2.m_type == dTreeAdressStmt::m_floatVar) {
							dAssert (0);
						}
					}
					break;
				}

				case dTreeAdressStmt::m_if:
				{
					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode0 = spillPriority.Find(stmt.m_arg0.m_label);
					if (priorityNode0) {
						priorityNode0->GetInfo().m_useCount --;
						priorityNode0->GetInfo().m_loopUseCount ++;
					}

					dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode1 = spillPriority.Find(stmt.m_arg1.m_label);
					if (priorityNode1) {
						priorityNode1->GetInfo().m_useCount --;
						priorityNode1->GetInfo().m_loopUseCount ++;
					}
					break;
				}


/*
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


*/
				case dTreeAdressStmt::m_call:
				case dTreeAdressStmt::m_goto:
				case dTreeAdressStmt::m_label:
				case dTreeAdressStmt::m_nop:
				case dTreeAdressStmt::m_enter:
				case dTreeAdressStmt::m_leave:
				case dTreeAdressStmt::m_function:
				case dTreeAdressStmt::m_argument:
					break;

				default:
				dAssert (0);
			}
			node = node->GetNext();
		} while (node!= loop.m_tail);
	}

m_flowGraph->m_cil->Trace();
	dFloat lowestUsage = 1.0e10f;
	dTree<dVariableSpillPriority,dString>::dTreeNode* spillCandidate = NULL;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTree<dVariableSpillPriority,dString>::dTreeNode* const priorityNode = spillPriority.Find(iter.GetNode()->GetKey());
		dAssert (priorityNode);

		dRegisterInterferenceNode& iterferanceInfo = iter.GetNode()->GetInfo();
		dList<dRegisterInterferenceNodeEdge>& edgeListInfo = iterferanceInfo.m_interferanceEdge; 
		dFloat priority = dFloat ((iterferanceInfo.m_isMove ? D_MOVE_WEIGHT_FACTOR : 0) + priorityNode->GetInfo().m_useCount + D_SPILL_WEIGHT_FACTOR * priorityNode->GetInfo().m_loopUseCount) / (edgeListInfo.GetCount() + 1);
		if (priority < lowestUsage) {
			lowestUsage = priority;
			spillCandidate = priorityNode;
		}
	}

}