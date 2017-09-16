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

#include "dLSCstdafx.h"
#include "dDAG.h"
#include "dDAGTypeNode.h"
#include "dDAGClassNode.h"
#include "dDAGFunctionNode.h"
#include "dDAGParameterNode.h"
#include "dDAGScopeBlockNode.h"
#include "dDAGFunctionModifier.h"
#include "dDAGFunctionStatementReturn.h"

dInitRtti(dDAGFunctionNode);



dDAGFunctionNode::dDAGFunctionNode(dList<dDAG*>& allNodes, dDAGTypeNode* const type, const char* const name, const char* const visivility)
	:dDAG(allNodes)
	,m_isStatic(false)
	,m_isPublic(true)
	,m_isConstructor(false)
	,m_loopLayer(0)
	,m_opertatorThis()
	,m_returnType (type)
	,m_body(NULL)
	,m_modifier(NULL)
	,m_functionStart(NULL)
	,m_parameters() 
{
	m_name = name;

	m_isStatic = strstr (visivility, "static") ? true : false;
	m_isPublic = strstr (visivility, "public") ? true : false;

	if (!m_isStatic) {
		dAssert (0);
		dDAGParameterNode* const operatorThis = new dDAGParameterNode (allNodes, "this", "");
		operatorThis->SetType(new dDAGTypeNode (allNodes, "this"));
		AddParameter(operatorThis);
	}

}


dDAGFunctionNode::~dDAGFunctionNode(void)
{
	dAssert (m_returnType);
}



void dDAGFunctionNode::AddParameter(dDAGParameterNode* const parameter)
{
	dAssert (parameter->IsType(dDAGParameterNode::GetRttiType()));

	dDAGTypeNode* const type = parameter->GetType();
	m_name += m_prototypeSeparator + type->GetArgType().GetTypeName();
	m_parameters.Append(parameter);
}

void dDAGFunctionNode::SetBody(dDAGScopeBlockNode* const body)
{
	m_body = body;
}

void dDAGFunctionNode::SetModifier(dDAGFunctionModifier* const modifier)
{
	m_modifier = modifier;
	dAssert (0);
//	m_modifier->AddRef();
}


dDAGParameterNode* dDAGFunctionNode::FindArgumentVariable(const char* const name) const
{
	for (dList<dDAGParameterNode*>::dListNode* node = m_parameters.GetFirst(); node; node = node->GetNext()) {
		dDAGParameterNode* const variable = node->GetInfo();
		if (variable->m_name == name) {
			return variable;
		}
	}
	return NULL;
}

void dDAGFunctionNode::ConnectParent(dDAG* const parent)
{
	m_parent = parent;

	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		m_body->AddVariable (arg->m_name, arg->m_type->GetArgType());
		arg->ConnectParent(this);
	}

	m_body->ConnectParent(this);
	m_returnType->ConnectParent(this);

	if (m_modifier) {
		m_modifier->ConnectParent(this);
	}
}


void dDAGFunctionNode::CompileCIL(dCIL& cil)  
{
	dAssert (m_body);
	dDAGClassNode* const myClass = GetClass();

	cil.ResetTemporaries();
	dString returnVariable (cil.NewTemp());

	dString functionName (myClass->GetFunctionName (m_name, m_parameters));
	dCILInstrFunction* const function = new dCILInstrFunction (cil, functionName, m_returnType->GetArgType());
	m_functionStart = function->GetNode();

	if (!m_isStatic) {
		dAssert (0);
//		dList<dDAGParameterNode*>::dListNode* const argNode = m_parameters.GetFirst();
//		dDAGParameterNode* const arg = argNode->GetInfo();
//		m_opertatorThis = arg->m_result.m_label;
	}

	// emit the function arguments
	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		arg->m_result = function->AddParameter (arg->m_name, arg->m_type->GetArgType())->GetInfo();
	}
	function->Trace();

	dCILInstrLabel* const entryPoint = new dCILInstrLabel(cil, cil.NewLabel());
	entryPoint->Trace();

	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();

		dTree<dCILInstr::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert (varNameNode);
		dCILInstrArgument* const localVariable = new dCILInstrArgument(cil, varNameNode->GetInfo().m_label, varNameNode->GetInfo().GetType());
		localVariable->Trace();
	}


	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTree<dCILInstr::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert (varNameNode);
		//dCILInstrStore* const store = new dCILInstrStore(cil, varNameNode->GetInfo().m_label, varNameNode->GetInfo().GetType(), arg->m_name, arg->GetType()->GetArgType());
		dString localVariableAliasName(cil.NewTemp());
		dCILInstrMove* const store = new dCILInstrMove(cil, localVariableAliasName, arg->GetType()->GetArgType(), varNameNode->GetInfo().m_label, varNameNode->GetInfo().GetType());
		//arg->m_result =  store->GetArg0();
		varNameNode->GetInfo().m_label = localVariableAliasName;
		store->Trace();
	}

//cil.Trace();
	m_body->CompileCIL(cil);
//cil.Trace();
	if (!m_returnType->GetArgType().m_isPointer && (m_returnType->GetArgType().m_intrinsicType == dCILInstr::m_void)) {
		 if (!cil.GetLast()->GetInfo()->GetAsReturn()) {
			dCILInstrReturn* const ret = new dCILInstrReturn(cil, "", dCILInstr::dArgType (dCILInstr::m_void));
			ret->Trace();
		}
	}

	dCILInstrFunctionEnd* const end = new dCILInstrFunctionEnd(function);

	dCILInstrReturn* const ret = end->GetNode()->GetPrev()->GetInfo()->GetAsReturn();
	dAssert (ret);
	dCILInstr::dArg returnArg (ret->GetArg0());
	
	dString exitLabel (cil.NewLabel());
	dCILInstrLabel* const returnLabel = new dCILInstrLabel (cil, exitLabel);
	cil.InsertAfter(ret->GetNode()->GetPrev(), returnLabel->GetNode());

	dCILInstrGoto* const jumpToReturn = new dCILInstrGoto(cil, exitLabel);
	cil.InsertAfter (returnLabel->GetNode()->GetPrev(), jumpToReturn->GetNode());
	jumpToReturn->SetTarget(returnLabel);

	dCIL::dListNode* prev;
	for (dCIL::dListNode* node = returnLabel->GetNode(); node && !node->GetInfo()->GetAsFunction(); node = prev) {
		dCILInstrReturn* const ret = node->GetInfo()->GetAsReturn();
		prev = node->GetPrev();
		if (ret) {
			dCILInstrLabel* const dommyLabel = new dCILInstrLabel (cil, cil.NewLabel());
			cil.InsertAfter(ret->GetNode()->GetPrev(), dommyLabel->GetNode());
	
			dCILInstrGoto* const jumpToReturn = new dCILInstrGoto (cil, exitLabel);
			cil.InsertAfter(dommyLabel->GetNode()->GetPrev(), jumpToReturn->GetNode());
			jumpToReturn->SetTarget(returnLabel);

			if (returnArg.m_isPointer || returnArg.GetType().m_intrinsicType != dCILInstr::m_void) {
				dCILInstrMove* const move = new dCILInstrMove (cil, returnArg.m_label, returnArg.GetType(), ret->GetArg0().m_label, returnArg.GetType());
				cil.InsertAfter (jumpToReturn->GetNode()->GetPrev(), move->GetNode());
			}
			cil.Remove (node);
		}
	}

	m_basicBlocks.Build (m_functionStart);
	m_basicBlocks.ConvertToSSA ();
//cil.Trace();
}


void dDAGFunctionNode::Optimize (dCIL& cil)
{
	m_basicBlocks.OptimizeSSA ();
}

bool dDAGFunctionNode::RemoveRedundantJumps (dCIL& cil)
{
	bool ret = false;
dAssert (0);
#if 0
	dTree<int, dCIL::dListNode*> jumpMap;

	// create jump and label map;
	for (dCIL::dListNode* stmtNode = m_functionStart; stmtNode; stmtNode = stmtNode->GetNext()) {
		dCILInstr* const instr = stmtNode->GetInfo();
		if (instr->GetAsIF() || instr->GetAsLabel() || instr->GetAsGoto()) {
			jumpMap.Insert(0, stmtNode);
		}
	}

	dTree<int, dCIL::dListNode*>::Iterator iter (jumpMap);

#if 0
	// remove redundant adjacent labels
	for (iter.Begin(); iter; iter ++) {
		dCIL::dListNode* const node = iter.GetKey();
		const dThreeAdressStmt& stmt = node->GetInfo();

		if (stmt.m_instruction == dThreeAdressStmt::m_label) {
			dCIL::dListNode* const labelNode = node->GetNext();
			if (labelNode && (labelNode->GetInfo().m_instruction == dThreeAdressStmt::m_label)) {
				dTree<int, dCIL::dListNode*>::Iterator iter1 (jumpMap);
				for (iter1.Begin(); iter1; iter1 ++) {
					dCIL::dListNode* const node1 = iter1.GetKey();
					dThreeAdressStmt& stmt1 = node1->GetInfo();
					if (stmt1.m_instruction == dThreeAdressStmt::m_goto) {
						dAssert (0);
//						if (stmt1.m_jmpTarget == labelNode)	{
//							stmt1.m_jmpTarget = node;
//							stmt1.m_arg0.m_label = stmt.m_arg0.m_label;
//						}
					} else if (stmt1.m_instruction == dThreeAdressStmt::m_if) { 
						dAssert (0);
//						if (stmt1.m_jmpTarget == labelNode)	{
//							stmt1.m_jmpTarget = node;	
//							stmt1.m_arg2.m_label = stmt.m_arg0.m_label;
//						}
					}
				}
				ret = true;
				dAssert (0);
				//m_cil->Remove(labelNode);
				//jumpMap.Remove(labelNode);
			}
		}
	}


	// redirect double indirect goto
	for (iter.Begin(); iter; iter ++) {
		dCIL::dListNode* const node = iter.GetKey();
		dThreeAdressStmt& stmt = node->GetInfo();
DTRACE_INTRUCTION (&stmt);
		if (stmt.m_instruction == dThreeAdressStmt::m_goto) {
			dAssert (jumpMap.Find (stmt.m_trueTargetJump));
			dCIL::dListNode* const targetNode = jumpMap.Find (stmt.m_trueTargetJump)->GetKey();
			dThreeAdressStmt& stmt1 = targetNode->GetInfo();
			dCIL::dListNode* nextGotoNode = targetNode->GetNext();
			while (nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_nop) {
				nextGotoNode = nextGotoNode->GetNext();
			}
			if ((stmt1.m_instruction == dThreeAdressStmt::m_label) && (nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_goto)) {
				const dThreeAdressStmt& stmt2 = nextGotoNode->GetInfo();
				stmt.m_arg0.m_label = stmt2.m_arg0.m_label;
				stmt.m_trueTargetJump = stmt2.m_trueTargetJump;
				ret = true;
			}

		} else if (stmt.m_instruction == dThreeAdressStmt::m_if) {
			if (stmt.m_falseTargetJump) {
				dAssert (jumpMap.Find (stmt.m_falseTargetJump));
				dCIL::dListNode* const falseTargetJump = jumpMap.Find (stmt.m_falseTargetJump)->GetKey();
				dThreeAdressStmt& stmt1 = falseTargetJump->GetInfo();

				dCIL::dListNode* nextGotoNode = falseTargetJump->GetNext();
				while (nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_nop) {
					nextGotoNode = nextGotoNode->GetNext();
				}

				dThreeAdressStmt& nextStmt = nextGotoNode->GetInfo();
				dAssert (nextStmt.m_instruction != dThreeAdressStmt::m_label);
				stmt.m_falseTargetJump = NULL;
				stmt.m_arg2.m_label = "";
			}
#if 0
			dAssert (jumpMap.Find (stmt.m_trueTargetJump));
			dCIL::dListNode* const trueTargetNode = jumpMap.Find (stmt.m_trueTargetJump)->GetKey();
			dThreeAdressStmt& stmt1 = trueTargetNode->GetInfo();

			dCIL::dListNode* nextGotoNode = trueTargetNode->GetNext();
			while (nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_nop) {
				nextGotoNode = nextGotoNode->GetNext();
			}
			if ((stmt1.m_instruction == dThreeAdressStmt::m_label) && (nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_goto)) {
				dAssert (0);

				const dThreeAdressStmt& stmt2 = nextGotoNode->GetInfo();
				stmt.m_arg2.m_label = stmt2.m_arg0.m_label;
				stmt.m_jmpTarget = stmt2.m_jmpTarget;
				ret = true;
			}
#endif
		}
	}
	

	// remove goto to immediate labels
	for (iter.Begin(); iter; iter ++) {
		dCIL::dListNode* const node = iter.GetKey();
		dThreeAdressStmt& stmt = node->GetInfo();
		dCIL::dListNode* const nextNode = node->GetNext();
		dAssert (0);

		//if (((stmt.m_instruction == dThreeAdressStmt::m_if) || (stmt.m_instruction == dThreeAdressStmt::m_goto)) && (stmt.m_jmpTarget == nextNode)) {
			dAssert (0);
			ret = true;
			//Remove(node);
			//jumpMap.Remove(node);
		//}
	}
#endif

	// redirect double indirect if
	for (iter.Begin(); iter; iter ++) {
		dCIL::dListNode* const node = iter.GetKey();
		dCILInstr* const instr = node->GetInfo();
		if (instr->GetAsIF()) {
			dCILInstrConditional* const ifInstr = instr->GetAsIF();
			//dAssert(jumpMap.Find(stmt.m_trueTargetJump));
			dCIL::dListNode* const trueTarget = ifInstr->GetTrueTarget();
			dAssert(jumpMap.Find(trueTarget));

			dAssert(jumpMap.Find(trueTarget)->GetKey() == trueTarget);
			//dCIL::dListNode* const trueTargetNode = jumpMap.Find(trueTarget)->GetKey();
			dCIL::dListNode* nextGotoNode = trueTarget;
			//while ((nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_nop) || (nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_label)) {
			while (nextGotoNode->GetInfo()->GetAsNop() || nextGotoNode->GetInfo()->GetAsLabel()) {
				nextGotoNode = nextGotoNode->GetNext();
			}
				
			//	dThreeAdressStmt& stmt1 = nextGotoNode->GetInfo();
			//if (stmt1.m_instruction == dThreeAdressStmt::m_goto) {
			if (nextGotoNode->GetInfo()->GetAsGoto()) {
				dAssert(0);
				//const dThreeAdressStmt& stmt2 = nextGotoNode->GetInfo();
				//stmt.m_arg1.m_label = stmt1.m_arg0.m_label;
				//stmt.m_trueTargetJump = stmt1.m_trueTargetJump;
				//ret = true;
			}
		}
	}

cil.Trace();
	// delete goto to immediate label
	for (iter.Begin(); iter; iter++) {
		dCIL::dListNode* const node = iter.GetKey();
		dCILInstr* const instr = node->GetInfo();
		if (instr->GetAsGoto()) {
//instr->Trace();

			dCILInstrGoto* const gotoInstr = instr->GetAsGoto();

			dCIL::dListNode* const target = gotoInstr->GetTarget();
			dAssert(jumpMap.Find(target));
			dCIL::dListNode* const destTarget = jumpMap.Find(target)->GetKey();

			dCILInstr* const instr1 = destTarget->GetInfo();
			dCIL::dListNode* nextGotoNode = node->GetNext();
			while (nextGotoNode->GetInfo()->GetAsNop()) {
				nextGotoNode = nextGotoNode->GetNext();
			}

			dCILInstrLabel* const label = destTarget->GetInfo()->GetAsLabel();
			dAssert(label);
			if (nextGotoNode == nextGotoNode) {
				//ifInstr->SetTargets(ifInstr->GetTrueTarget()->GetInfo()->GetAsLabel(), NULL);
				gotoInstr->Nullify();
cil.Trace();
			}
		}
	}
cil.Trace();

	
	// delete unreachable goto
	for (iter.Begin(); iter; iter ++) {
		dCIL::dListNode* const node = iter.GetKey();
		dCILInstr* const instr = node->GetInfo();

		//dThreeAdressStmt& stmt = node->GetInfo();
		//if (stmt.m_instruction == dThreeAdressStmt::m_goto) {
		if (instr->GetAsGoto()) {
			dAssert(0);
			/*
			//dCIL::dListNode* const trueTargetNode = jumpMap.Find (stmt.m_trueTargetJump)->GetKey();
			dCIL::dListNode* prevNode = node->GetPrev();
			while (prevNode->GetInfo().m_instruction == dThreeAdressStmt::m_nop) {
				prevNode = prevNode->GetPrev();
			}
			dThreeAdressStmt& stmt1 = prevNode->GetInfo();
			if ((stmt1.m_instruction == dThreeAdressStmt::m_ret) || (stmt1.m_instruction == dThreeAdressStmt::m_goto)){
				ret = true;
				stmt.m_instruction = dThreeAdressStmt::m_nop;
				jumpMap.Remove(node);
			}
*/
		}
	}


	// redirect if to immediate label
	for (iter.Begin(); iter; iter++) {
		dCIL::dListNode* const node = iter.GetKey();
		//dThreeAdressStmt& stmt = node->GetInfo();
		//iter ++;
		//if (stmt.m_instruction == dThreeAdressStmt::m_if) {
		dCILInstr* const instr = node->GetInfo();
		if (instr->GetAsIF()) {
			dCILInstrConditional* const ifInstr = instr->GetAsIF();
			//dAssert (jumpMap.Find (stmt.m_trueTargetJump));
			//dCIL::dListNode* const trueTargetNode = jumpMap.Find (stmt.m_trueTargetJump)->GetKey();

			dCIL::dListNode* const trueTarget = ifInstr->GetTrueTarget();
			dAssert(jumpMap.Find(trueTarget));
			dAssert(jumpMap.Find(trueTarget)->GetKey() == trueTarget);

			
			dCIL::dListNode* nextGotoNode = trueTarget;
			//while ((nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_nop) || (nextGotoNode->GetInfo().m_instruction == dThreeAdressStmt::m_label)) {
			while (nextGotoNode->GetInfo()->GetAsNop() || nextGotoNode->GetInfo()->GetAsLabel()) {
				nextGotoNode = nextGotoNode->GetNext();
			}

			bool islive = false;
			for (dCIL::dListNode* node1 = node->GetNext(); node1 != nextGotoNode; node1 = node1->GetNext()) {
				//dThreeAdressStmt& stmt1 = node1->GetInfo();
				dCILInstr* const instr1 = node1->GetInfo();
				//if (!((stmt1.m_instruction == dThreeAdressStmt::m_nop) || (stmt1.m_instruction == dThreeAdressStmt::m_label))) {
				if (!(instr1->GetAsNop() || nextGotoNode->GetInfo()->GetAsLabel())) {
					islive = true;
					break;
				}
			}
			if (!islive) {
				dAssert(0);
				//stmt.m_instruction = dThreeAdressStmt::m_nop;
				//jumpMap.Remove(node);
				//ret = true;
			}
		}
	}

//m_cil->Trace();

	// delete unreferenced labels
	for (iter.Begin(); iter; ) {
		dCIL::dListNode* const node = iter.GetKey();
		//dThreeAdressStmt& stmt = node->GetInfo();
		dCILInstr* const instr = node->GetInfo();
		iter++;

		//if (stmt.m_instruction == dThreeAdressStmt::m_label) {		
		if (instr->GetAsLabel()) {
			//instr->Trace();

			dTree<int, dCIL::dListNode*>::Iterator iter1 (jumpMap);
			bool isReferenced = false;
			for (iter1.Begin(); iter1; iter1 ++) {
				dCIL::dListNode* const node1 = iter1.GetKey();
				//dThreeAdressStmt& stmt1 = node1->GetInfo();
				dCILInstr* const instr1 = node1->GetInfo();
				//if (stmt1.m_instruction == dThreeAdressStmt::m_goto){
				if (instr1->GetAsGoto()){
					//if (stmt1.m_trueTargetJump == node) {
					dCILInstrGoto* const gotoInstru = instr1->GetAsGoto();
					if (gotoInstru->GetTarget() == node) {
						isReferenced = true;
						break;
					}
				//} else if (stmt1.m_instruction == dThreeAdressStmt::m_if) {
				} else if (instr1->GetAsIF()) {
					dCILInstrConditional* const ifInstru = instr1->GetAsIF();
					//if ((stmt1.m_trueTargetJump == node) || (stmt1.m_falseTargetJump == node)) {
					if ((ifInstru->GetTrueTarget() == node) || (ifInstru->GetFalseTarget()  == node)) {
						isReferenced = true;
						break;
					}
				}
			}
			if (!isReferenced) {
				//instr->Trace();
				ret = true;
				//stmt.m_instruction = dThreeAdressStmt::m_nop;
				jumpMap.Remove(node);
				instr->Nullify();
			}
		}
	}

cil.Trace();
#endif
	return ret;
}


/*
void dDAGFunctionNode::ClearBasicBlocks ()
{
	m_basicBlocks.RemoveAll();
}

void dDAGFunctionNode::BuildBasicBlocks(dCIL& cil, dCIL::dListNode* const functionNode)
{
	// build leading block map table
	void ClearBasicBlocks ();

	// remove redundant jumps
	dCIL::dListNode* nextNode;
	for (dCIL::dListNode* node = functionNode; node; node = nextNode) {
		nextNode = node->GetNext(); 
		const dThreeAdressStmt& stmt = node->GetInfo();
		if (stmt.m_instruction == dThreeAdressStmt::m_goto) {
			dCIL::dListNode* const prevNode = node->GetPrev();
			const dThreeAdressStmt& prevStmt = prevNode->GetInfo();
			if (prevStmt.m_instruction == dThreeAdressStmt::m_ret) {
				cil.Remove(node);
			}
		}
	}

	// find the root of all basic blocks leaders
	for (dCIL::dListNode* node = functionNode; node; node = node->GetNext()) {
		const dThreeAdressStmt& stmt = node->GetInfo();

		if (stmt.m_instruction == dThreeAdressStmt::m_label) {
			m_basicBlocks.Append(dBasicBlock(node));
		}
	}

	for (dList<dBasicBlock>::dListNode* blockNode = m_basicBlocks.GetFirst(); blockNode; blockNode = blockNode->GetNext()) {
		dBasicBlock& block = blockNode->GetInfo();

		for (dCIL::dListNode* stmtNode = block.m_begin; !block.m_end && stmtNode; stmtNode = stmtNode->GetNext()) {
			const dThreeAdressStmt& stmt = stmtNode->GetInfo();
			switch (stmt.m_instruction)
			{
				case dThreeAdressStmt::m_if:
				case dThreeAdressStmt::m_goto:
				case dThreeAdressStmt::m_ret:
					block.m_end = stmtNode;
					break;
			}
		} 
	}
}
*/





void dDAGFunctionNode::ConvertToTarget (dCIL& cil)
{
	dAssert (0);
//	cil.RegisterAllocation (m_functionStart);
}


