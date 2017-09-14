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


#include "dNewtonLuaStdafx.h"
#include "dNewtonLuaLex.h"
#include "dNewtonLuaParcer.h"
#include "dNewtonLuaCompiler.h"


dNewtonLuaCompiler::dLuaClosure::dLuaClosure()
	:dCIL()
	,m_parent(NULL)
	,m_returnLabel(NewLabel())
	,m_returnVariable(NewTemp())
{
}

dNewtonLuaCompiler::dLuaClosure::~dLuaClosure()
{
}

void dNewtonLuaCompiler::dLuaClosure::RemoveAll()
{
	dCIL::RemoveAll();
	m_children.RemoveAll();
}

dNewtonLuaCompiler::dLuaClosure* dNewtonLuaCompiler::dLuaClosure::AddClosure(dLuaClosure* const parent)
{
	dNewtonLuaCompiler::dLuaClosure* const closureNode = &m_children.Append()->GetInfo();
	closureNode->m_parent = this;
	return closureNode;
}


dNewtonLuaCompiler::dNewtonLuaCompiler()
	:dNewtonLuaParcer()
	,m_closures()
	,m_currentClosure(&m_closures)
{
}

dNewtonLuaCompiler::~dNewtonLuaCompiler()
{
	dAssert (0);
//	for(dList<dDAG*>::dListNode* nodePtr = m_allNodes.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext() ) {
//		dDAG* const node = nodePtr->GetInfo();
//		delete node;
//	}
/*
	if (m_currentPackage) {
		dAssert (0);
		m_currentPackage->Save (m_packageFileName.GetStr());
		delete m_currentPackage;
	}
*/
}

int dNewtonLuaCompiler::CompileSource (const char* const source)
{
	m_closures.RemoveAll();
	m_currentClosure = &m_closures;
	dNewtonLuaLex scanner(source);
	bool status = Parse(scanner);



#if 0
	if (status) {
		for (dList<dDAGClassNode*>::dListNode* node = m_classList.GetFirst(); node; node = node->GetNext()) {
			dDAGClassNode* const scripClass =  node->GetInfo();
			scripClass->ConnectParent (NULL);
		}

		dCIL cil;
		for (dList<dDAGClassNode*>::dListNode* node = m_classList.GetFirst(); node; node = node->GetNext()) {
			dDAGClassNode* const scripClass = node->GetInfo();
			scripClass->CompileCIL (cil);
		}

		for (dList<dDAGClassNode*>::dListNode* node = m_classList.GetFirst(); node; node = node->GetNext()) {
			dDAGClassNode* const scripClass = node->GetInfo();
			scripClass->Optimize(cil);
		}

/*
		for (dList<dDAGClassNode*>::dListNode* node = m_classList.GetFirst(); node; node = node->GetNext()) {
			dDAGClassNode* const scripClass = node->GetInfo();
			scripClass->ConvertToTarget (cil);
		}

		dVirtualMachine* const program = cil.BuilExecutable();
		delete program;
*/
	}
#endif
	return 0;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitFunctionDeclaration(const dUserVariable& functionName)
{
	m_currentClosure = m_closures.AddClosure(m_currentClosure);

//	dString returnVariable(m_cil.NewTemp());
//	dString functionName(myClass->GetFunctionName(m_name, m_parameters));
//	dCILInstrFunction* const function = new dCILInstrFunction(m_cil, functionName.GetString(), m_returnType->GetArgType());

/*
	m_functionStart = function->GetNode();

	if (!m_isStatic) {
		dAssert(0);
		//		dList<dDAGParameterNode*>::dListNode* const argNode = m_parameters.GetFirst();
		//		dDAGParameterNode* const arg = argNode->GetInfo();
		//		m_opertatorThis = arg->m_result.m_label;
	}

	// emit the function arguments
	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		arg->m_result = function->AddParameter(arg->m_name, arg->m_type->GetArgType())->GetInfo();
	}
	function->Trace();

	dCILInstrLabel* const entryPoint = new dCILInstrLabel(cil, cil.NewLabel());
	entryPoint->Trace();

	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();

		dTree<dCILInstr::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert(varNameNode);
		dCILInstrArgument* const localVariable = new dCILInstrArgument(cil, varNameNode->GetInfo().m_label, varNameNode->GetInfo().GetType());
		localVariable->Trace();
	}


	for (dList<dDAGParameterNode*>::dListNode* argNode = m_parameters.GetFirst(); argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
		dTree<dCILInstr::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
		dAssert(varNameNode);
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
			dCILInstrReturn* const ret = new dCILInstrReturn(cil, "", dCILInstr::dArgType(dCILInstr::m_void));
			ret->Trace();
		}
	}

	dCILInstrFunctionEnd* const end = new dCILInstrFunctionEnd(function);

	dCILInstrReturn* const ret = end->GetNode()->GetPrev()->GetInfo()->GetAsReturn();
	dAssert(ret);
	dCILInstr::dArg returnArg(ret->GetArg0());

	dString exitLabel(cil.NewLabel());
	dCILInstrLabel* const returnLabel = new dCILInstrLabel(cil, exitLabel);
	cil.InsertAfter(ret->GetNode()->GetPrev(), returnLabel->GetNode());

	dCILInstrGoto* const jumpToReturn = new dCILInstrGoto(cil, exitLabel);
	cil.InsertAfter(returnLabel->GetNode()->GetPrev(), jumpToReturn->GetNode());
	jumpToReturn->SetTarget(returnLabel);

	dCIL::dListNode* prev;
	for (dCIL::dListNode* node = returnLabel->GetNode(); node && !node->GetInfo()->GetAsFunction(); node = prev) {
		dCILInstrReturn* const ret = node->GetInfo()->GetAsReturn();
		prev = node->GetPrev();
		if (ret) {
			dCILInstrLabel* const dommyLabel = new dCILInstrLabel(cil, cil.NewLabel());
			cil.InsertAfter(ret->GetNode()->GetPrev(), dommyLabel->GetNode());

			dCILInstrGoto* const jumpToReturn = new dCILInstrGoto(cil, exitLabel);
			cil.InsertAfter(dommyLabel->GetNode()->GetPrev(), jumpToReturn->GetNode());
			jumpToReturn->SetTarget(returnLabel);

			if (returnArg.m_isPointer || returnArg.GetType().m_intrinsicType != dCILInstr::m_void) {
				dCILInstrMove* const move = new dCILInstrMove(cil, returnArg.m_label, returnArg.GetType(), ret->GetArg0().m_label, returnArg.GetType());
				cil.InsertAfter(jumpToReturn->GetNode()->GetPrev(), move->GetNode());
			}
			cil.Remove(node);
		}
	}

	m_basicBlocks.Build(m_functionStart);
	m_basicBlocks.ConvertToSSA();
*/

	dCILInstrFunction* const function = new dCILInstrFunction(*m_currentClosure, functionName.GetString(), dCILInstr::dArgType(dCILInstr::m_luaType));


	dString label(m_currentClosure->NewLabel());
	dCILInstrLabel* const startBlock = new dCILInstrLabel(*m_currentClosure, label);
	


	dUserVariable variable(functionName);
	variable.m_node = function->GetNode();
	function->Trace();
	startBlock->Trace();
	return variable;
}

void dNewtonLuaCompiler::CloseFunctionDeclaration()
{
	dCILInstr::dArgType type(dCILInstr::m_luaType);
	dCILInstrLabel* const label = new dCILInstrLabel(*m_currentClosure, m_currentClosure->m_returnLabel);
	for (dLuaClosure::dListNode* node = m_currentClosure->GetFirst(); node; node = node->GetNext()) {
		dCILInstrGoto* const gotoJump = node->GetInfo()->GetAsGoto();
		if (gotoJump) {
			if (gotoJump->GetArg0().m_label == m_currentClosure->m_returnLabel) {
				gotoJump->SetTarget(label);
			}
		}
	}
	dCILInstrReturn* const ret = new dCILInstrReturn(*m_currentClosure, m_currentClosure->m_returnVariable, type);
	dCILInstrFunctionEnd* const functionEnd = new dCILInstrFunctionEnd (m_currentClosure->GetFirst()->GetInfo()->GetAsFunction());

	label->Trace();
	ret->Trace();
	functionEnd->Trace();
	m_currentClosure->Trace();

	dBasicBlocksGraph basicBlocks;
	basicBlocks.Build(m_currentClosure->GetFirst());
	basicBlocks.Trace();

	m_currentClosure = m_currentClosure->m_parent;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitFunctionParameter(const dUserVariable& prevParameter, const dUserVariable& parameter)
{
	dCILInstrArgument* const localVariable = new dCILInstrArgument(*m_currentClosure, parameter.GetString(), dCILInstr::dArgType(dCILInstr::m_luaType));
	localVariable->LinkPrevius(prevParameter.m_node ? prevParameter.m_node->GetInfo() : NULL);
	dUserVariable variable(parameter);
	variable.m_node = localVariable->GetNode();
	localVariable->Trace();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::LinkExpresion(const dUserVariable& expression0, const dUserVariable& expression1)
{
	expression1.m_node->GetInfo()->LinkPrevius(expression0.m_node->GetInfo());
	return expression1;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitParametersToLocalVariables(const dUserVariable& parameterList)
{
	dCILInstr::dArgType type(dCILInstr::m_luaType);
	for (dCILInstr* instruction = parameterList.m_node->GetInfo(); instruction; instruction = instruction->GetPrevius()) {
		dCILInstrArgument* const argumnet = instruction->GetAsArgument();
		dAssert(argumnet);
		dString local(m_currentClosure->NewTemp());
		dCILInstrMove* const move = new dCILInstrMove(*m_currentClosure, local, type, argumnet->GetArg0().m_label, type);
		m_currentClosure->m_argumnets.Append(move);
		move->Trace();
	}

	return parameterList;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitFunctionCall(const dUserVariable& functionName, const dUserVariable& argumentsList)
{
	dCILInstr::dArgType type(dCILInstr::m_luaType);
	dString result(m_currentClosure->NewTemp());
	dCILInstrCall* const functionCall = new dCILInstrCall(*m_currentClosure, result, type, functionName.GetString());

	for (dCILInstr* instruction = argumentsList.m_node->GetInfo(); instruction; instruction = instruction->GetPrevius()) {
		dCILSingleArgInstr* const parameter = instruction->GetAsSingleArg();
		functionCall->AddArgument(parameter->GetArg0());
	}

	functionCall->Trace();
	dUserVariable variable(functionName);
	variable.m_data = result;
	variable.m_node = functionCall->GetNode();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitLoadConstant(const dUserVariable& constName)
{
	dUserVariable variable(constName);
	dString localVariableAliasName(m_currentClosure->NewTemp());
	dCILInstr::dArgType type(dCILInstr::m_luaType);
	dCILInstrMove* const move = new dCILInstrMove(*m_currentClosure, localVariableAliasName, type, constName.GetString(), type);
	variable.m_data = localVariableAliasName;
	variable.m_node = move->GetNode();
	move->Trace();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitLoadVariable(const dUserVariable& varName)
{
	dUserVariable variable(varName);

	dCILInstrMove* definition = NULL;
	for (dList<dCILInstrMove*>::dListNode* node = m_currentClosure->m_argumnets.GetFirst(); node; node = node->GetNext()) {
		dCILInstrMove* const intruction = node->GetInfo()->GetAsMove();
		if (intruction) {
			const dCILInstr::dArg& argName = intruction->GetArg1();
			if (argName.m_label == varName.GetString()) {
				definition = intruction;
				break;
			}
		}
	}

	dAssert(definition);
	dString outVarName(m_currentClosure->NewTemp());
	dCILInstr::dArgType type(dCILInstr::m_luaType);
	dCILInstrMove* const move = new dCILInstrMove(*m_currentClosure, outVarName, type, definition->GetArg0().m_label, type);
	variable.m_data = outVarName;
	variable.m_node = move->GetNode();
	move->Trace();
	return variable;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitBinaryExpression(const dUserVariable& arg0, const dUserVariable& binaryOperator, const dUserVariable& arg1)
{
	int token = binaryOperator.GetToken();
	dCILThreeArgInstr::dOperator operation = dCILThreeArgInstr::m_operatorsCount;
	switch (token)
	{
		case _IDENTICAL:
			operation = dCILThreeArgInstr::m_identical;
			break;
		case _DIFFERENT:
			operation = dCILThreeArgInstr::m_different;
			break;

		case '*':
			operation = dCILThreeArgInstr::m_mul;
			break;

		case '-':
			operation = dCILThreeArgInstr::m_sub;
			break;

		default:
			dAssert(0);
	}

	dCILInstr::dArgType type(dCILInstr::m_luaType);
	dAssert (operation != dCILThreeArgInstr::m_operatorsCount);

	dString outVarName(m_currentClosure->NewTemp());
	dCILInstrIntergerLogical* const instruction = new dCILInstrIntergerLogical(*m_currentClosure, operation, outVarName, type, arg0.GetString(), type, arg1.GetString(), type);

	dUserVariable variable(arg0);
	variable.m_data = outVarName;
	variable.m_node = instruction->GetNode();
	instruction->Trace();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitIf(const dUserVariable& expression)
{
	dCILInstr::dArgType type(dCILInstr::m_luaType);

	dString label2(m_currentClosure->NewLabel());
	dString label1(m_currentClosure->NewLabel());

	dCILInstrConditional* const conditional = new dCILInstrConditional(*m_currentClosure, dCILInstrConditional::m_ifnot, expression.GetString(), type, label1, label2);
	dCILInstrLabel* const taget2 = new dCILInstrLabel(*m_currentClosure, label2);
	conditional->SetTargets(taget2, taget2);
	
	dUserVariable variable;
	variable.m_node = conditional->GetNode();
	conditional->Trace();
	taget2->Trace();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitIfElse(const dUserVariable& ifStatement)
{
	dAssert(ifStatement.m_node);
	dCILInstrConditional* const conditional = ifStatement.m_node->GetInfo()->GetAsIF();
	dAssert(conditional);

	dCILInstrLabel* const target = new dCILInstrLabel(*m_currentClosure, conditional->GetArg1().m_label);
	conditional->SetTargets(target, conditional->GetTrueTarget()->GetInfo()->GetAsLabel());

	dUserVariable variable;
	variable.m_node = target->GetNode();
	target->Trace();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitReturn(const dUserVariable& expression)
{
	dAssert(expression.m_node);
	dCILInstr::dArgType type(dCILInstr::m_luaType);

	dCILInstrMove* const move = new dCILInstrMove(*m_currentClosure, m_currentClosure->m_returnVariable, type, expression.GetString(), type);
	dCILInstrGoto* const gotoJump = new dCILInstrGoto(*m_currentClosure, m_currentClosure->m_returnLabel);

	dUserVariable variable;
//	variable.m_data = outVarName;
	variable.m_node = move->GetNode();
	move->Trace();
	gotoJump->Trace();
	return variable;
}

