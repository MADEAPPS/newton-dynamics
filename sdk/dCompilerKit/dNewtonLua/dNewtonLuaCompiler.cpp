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
//	,m_packageRootDirectory (pakacgesRootNameDirectory)
//	,m_currentPackage(NULL)
//	//,m_currentFunction(NULL)
//	,m_classList()
//	,m_scopeStack()
//	,m_allNodes()
{
//	_mkdir(m_packageRootDirectory.GetStr());
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


#if 0

// parcel callbacks section
bool dNewtonLuaCompiler::Parse(dLittleScriptLexical& scanner)
{
	return dLittleScriptParser::Parse(scanner);
}



void dNewtonLuaCompiler::ImportClass (const dString& className)
{
}

void dNewtonLuaCompiler::ImportAllClasses (const dString& className)
{
}


dDAGScopeBlockNode* dNewtonLuaCompiler::GetCurrentScope() const
{
	dAssert (m_scopeStack.GetCount());
	return m_scopeStack.GetLast()->GetInfo();
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpressionNodeConstant (const dUserVariable& value)
{
	dUserVariable returnNode;
	dCILInstr::dArgType type (dCILInstr::m_constInt);

	switch (int (value.m_token))
	{
		case _THIS:
			type = dCILInstr::m_classPointer;
			break;

		case _FLOAT_CONST:
			type = dCILInstr::m_constFloat;
			break;

		case _INTEGER_CONST:
			type = dCILInstr::m_constInt;
			break;

//		case STRING_VALUE:
//			type = dDAGExpressionNodeConstant::m_stringValue;
//			break;

		default:
			dAssert (0);
	}

	dDAGExpressionNodeConstant* const node = new dDAGExpressionNodeConstant (m_allNodes, type, value.m_data.GetStr());
	returnNode.m_node = node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpressionNodeOperatorThisConstant(const dUserVariable& value)
{
	dUserVariable returnNode;
	dDAGExpressionNodeConstant* const node = new dDAGExpressionNodeOperatorThisConstant (m_allNodes);
	returnNode.m_node = node;
	return returnNode;

}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpresionNodePrefixPostfixOperator (const dUserVariable& expression, bool isPrefix, bool isIncrement)
{
	dUserVariable returnNode;
	dAssert (expression.m_node && expression.m_node->IsType(dDAGExpressionNodeVariable::GetRttiType()));
	
	dDAGExpressionNodePrefixPostfix* const node = new dDAGExpressionNodePrefixPostfix (m_allNodes, (dDAGExpressionNode*)expression.m_node, isPrefix, isIncrement);
	returnNode.m_node = node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpressionNodeBinaryOperator (const dUserVariable& expressionA, const dUserVariable& binaryOperator, const dUserVariable& expressionB)
{
	dUserVariable returnNode;

	dAssert (expressionA.m_node && expressionA.m_node->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (expressionB.m_node && expressionB.m_node->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGExpressionNodeBinaryOperator::dBinaryOperator binOperator = dDAGExpressionNodeBinaryOperator::m_add;
	switch (int (binaryOperator.m_token))
	{	
		case '+':
			binOperator = dDAGExpressionNodeBinaryOperator::m_add;
			break;

		case '-':
			binOperator = dDAGExpressionNodeBinaryOperator::m_sub;
			break;

		case '*':
			binOperator = dDAGExpressionNodeBinaryOperator::m_mul;
			break;

		case '/':
			binOperator = dDAGExpressionNodeBinaryOperator::m_div;
			break;

		case '%':
			binOperator = dDAGExpressionNodeBinaryOperator::m_mod;
			break;

		case _IDENTICAL:
			binOperator = dDAGExpressionNodeBinaryOperator::m_identical;
			break;

		case _DIFFERENT:
			binOperator = dDAGExpressionNodeBinaryOperator::m_different;
			break;


		case '<':
			binOperator = dDAGExpressionNodeBinaryOperator::m_less;
			break;

		case '>':
			binOperator = dDAGExpressionNodeBinaryOperator::m_greather;
			break;

		case _LESS_EQUAL:
			binOperator = dDAGExpressionNodeBinaryOperator::m_lessEqual;
			break;

		case _GREATHER_EQUAL:
			binOperator = dDAGExpressionNodeBinaryOperator::m_greatherEqual;
			break;

		default:
			dAssert (0);
	}


//	_ASSERTE (GetCurrentScope());
	dDAGExpressionNodeBinaryOperator* const node = new dDAGExpressionNodeBinaryOperator (m_allNodes, binOperator, (dDAGExpressionNode*)expressionA.m_node, (dDAGExpressionNode*)expressionB.m_node);
	returnNode.m_node = node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpressionNodeLogiOperator (const dUserVariable& expressionA, const dUserVariable& logicOperator, const dUserVariable& expressionB)
{
	dUserVariable returnNode;

	dAssert (expressionA.m_node && expressionA.m_node->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (expressionB.m_node && expressionB.m_node->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGExpressionNodeLogicOperator::dLogicOperator logOperator = dDAGExpressionNodeLogicOperator::m_and;
	switch (int (logicOperator.m_token))
	{
		case _LOGIC_OR:
			logOperator = dDAGExpressionNodeLogicOperator::m_or;
			break;

		case _LOGIC_AND:
			logOperator = dDAGExpressionNodeLogicOperator::m_and;
			break;

		default:
			dAssert (0);
	}

	dAssert (GetCurrentScope());
	dDAGExpressionNodeLogicOperator* const node = new dDAGExpressionNodeLogicOperator (m_allNodes, logOperator, (dDAGExpressionNode*)expressionA.m_node, (dDAGExpressionNode*)expressionB.m_node);

//	expressionA.m_node->Release();
//	expressionB.m_node->Release();
	returnNode.m_node = node;
	return returnNode;
}



dDAGClassNode* dNewtonLuaCompiler::GetCurrentClass() const
{
	dAssert (m_classList.GetCount());
	return m_classList.GetLast()->GetInfo();
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::CreateClass (const dString& visibility, const dString& callType, const dString& className, const dString& superClassName, const dString& interfaces)
{
	dUserVariable returnNode;
	dDAGClassNode* const classNode = new dDAGClassNode (m_allNodes);
	m_classList.Append(classNode);
	returnNode.m_node = classNode;
	classNode->FinalizeImplementation (visibility.GetStr(), className.GetStr(), NULL);
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitTypeNode (const dUserVariable& type, const dUserVariable& dim)
{
	dUserVariable returnNode;
	dDAGTypeNode* const typeNode = new dDAGTypeNode (m_allNodes, type.m_data);

	if (dim.m_node) {
		dDAGDimensionNode* const dimList = (dDAGDimensionNode*) dim.m_node;
		dAssert (dimList->IsType(dDAGDimensionNode::GetRttiType()));
		typeNode->AddDimensions (dimList);
	}

	returnNode.m_node = typeNode;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::AddClassFunction (const dUserVariable& returnType, const dString& name, const dString& visibility)
{
	dUserVariable returnNode;

	dDAGTypeNode* const typeNode = (dDAGTypeNode*) returnType.m_node;
	dAssert (typeNode->IsType(dDAGTypeNode::GetRttiType()));

	dDAGFunctionNode* const functionNode = new dDAGFunctionNode (m_allNodes, typeNode, name.GetStr(), visibility.GetStr());
	GetCurrentClass()->AddFunction(functionNode);

	returnNode.m_node = functionNode;
	//m_currentFunction = functionNode;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::AddClassContructor (const dString& name, const dString& visibility)
{
	dDAGClassNode* const curClass = GetCurrentClass();

	dUserVariable typeVariable;
	dAssert (name == curClass->m_name);

	typeVariable.m_node = new dDAGTypeNode (m_allNodes, "void");
	dUserVariable returnNode (AddClassFunction (typeVariable, name, "public static"));

	curClass->m_constructors.Append((dDAGFunctionNode*)returnNode.m_node);
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::FunctionAddParameterNode (const dUserVariable& parameter)
{
	dUserVariable returnNode;

	dDAGParameterNode* const parameterNode = (dDAGParameterNode*)parameter.m_node;
	dAssert (parameterNode->GetTypeId() == dDAGParameterNode::GetRttiType());

	dDAGFunctionNode* const function = GetCurrentClass()->GetCurrentFunction();
	function->AddParameter(parameterNode);
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::FunctionAddBodyBlock (const dUserVariable& functionBody)
{
	dUserVariable returnNode;

	dDAGFunctionNode* const function = GetCurrentClass()->GetCurrentFunction();

	dDAGScopeBlockNode* const bodyNode = (dDAGScopeBlockNode*) functionBody.m_node;
	dAssert (bodyNode->IsType (dDAGScopeBlockNode::GetRttiType()));
	function->SetBody(bodyNode);

	returnNode.m_node = function;
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewDimensionNode(const dUserVariable& expression)
{
	dUserVariable returnNode;
	dDAGExpressionNode* const expressionNode = (dDAGExpressionNode*)expression.m_node;
	dAssert (!expressionNode || (expressionNode->IsType (dDAGExpressionNode::GetRttiType())));
	dDAGDimensionNode* const node = new dDAGDimensionNode (m_allNodes, expressionNode);

	returnNode.m_node = node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpressionOperatorNew (const dString& typeName, const dUserVariable& dimension)
{
	dUserVariable returnNode;

	dDAGDimensionNode* const dimensionNode = (dDAGDimensionNode*) dimension.m_node;
	dAssert (dimensionNode->IsType (dDAGDimensionNode::GetRttiType()));
	dDAGExpressionNodeNew* const node = new dDAGExpressionNodeNew (m_allNodes, typeName.GetStr(), dimensionNode);

	returnNode.m_node = node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::ConcatenateDimensionNode(const dUserVariable& dim0, const dUserVariable& dim1)
{
	dUserVariable returnNode;

	dAssert (dim0.m_node->GetTypeId() == dDAGDimensionNode::GetRttiType());
	dAssert (dim1.m_node->GetTypeId() == dDAGDimensionNode::GetRttiType());

	dDAGDimensionNode* param = (dDAGDimensionNode*) dim0.m_node;
	for ( ;param->m_next; param = param->m_next);
	param->m_next = (dDAGDimensionNode*) dim1.m_node;

	returnNode.m_node = dim0.m_node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewParameterNode (const dUserVariable& primitiveType, const dString& name)
{
	dUserVariable returnNode;

	dDAGParameterNode* const parameter = new dDAGParameterNode(m_allNodes, name, "");
	dDAGTypeNode* const typeNode = (dDAGTypeNode*) primitiveType.m_node;
	if (typeNode) {
		dAssert (typeNode->GetTypeId() == dDAGTypeNode::GetRttiType());
		parameter->SetType(typeNode);
	}
	returnNode.m_node = parameter;
	return returnNode;
}



dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::BeginScopeBlock ()
{
	dUserVariable returnNode;
	dDAGScopeBlockNode* const scope = new dDAGScopeBlockNode (m_allNodes);
	m_scopeStack.Append(scope);
	returnNode.m_node = scope;
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpressionNodeVariable (const dString& name, const dString& modifiers, const dUserVariable& dimArray)
{
	dUserVariable returnNode;

//	_ASSERTE (GetCurrentScope());
	dDAGDimensionNode* const dimensionNode = (dDAGDimensionNode*) dimArray.m_node;
	dAssert (!dimensionNode || dimensionNode->IsType(dDAGDimensionNode::GetRttiType()));

	dDAGExpressionNodeVariable* const node = new dDAGExpressionNodeVariable (m_allNodes, name, modifiers, dimensionNode);
	returnNode.m_node = node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewVariableStatement (const dString& name, const dString& modifiers)
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGParameterNode (m_allNodes, name, modifiers);
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewVariableToCurrentBlock (const dString& modifiers, const dUserVariable& type, const dString& name)
{
	dUserVariable variableName(NewVariableStatement (name, modifiers));

	dDAGParameterNode* const variableNameNode = (dDAGParameterNode*)variableName.m_node;
	dAssert (variableNameNode->IsType(dDAGParameterNode::GetRttiType()));

	dDAGTypeNode* const typeNode = (dDAGTypeNode*)type.m_node;
	dAssert (typeNode->GetTypeId() == dDAGTypeNode::GetRttiType());
	variableNameNode->SetType(typeNode);

	dAssert (m_scopeStack.GetCount());
	dDAGScopeBlockNode* const block = GetCurrentScope();
	block->AddStatement(variableNameNode);

	dUserVariable returnNode (NewExpressionNodeVariable (name, modifiers));
	dAssert (returnNode.m_node->GetTypeId() == dDAGExpressionNodeVariable::GetRttiType());
	dDAGExpressionNodeVariable* const node = (dDAGExpressionNodeVariable*) returnNode.m_node;
	node->SetType((dDAGTypeNode*) typeNode->Clone (m_allNodes));
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::AddStatementToCurrentBlock(const dUserVariable& statement)
{
	dUserVariable returnNode;

	dDAGScopeBlockNode* const block = GetCurrentScope();

	dDAGFunctionStatement* const statementNode = (dDAGFunctionStatement*)statement.m_node;
	if (statementNode) {
		if (statementNode->IsType(dDAGExpressionNode::GetRttiType())) {
			for (dDAGParameterNode* node = (dDAGParameterNode*) statementNode; node; node = (dDAGParameterNode*) node->m_next) {
				block->AddStatement(node);
			}
		} else {
			dAssert (statementNode->IsType(dDAGFunctionStatement::GetRttiType()));
			block->AddStatement(statementNode);
		}
	}

	returnNode.m_node = block;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::AddClassVariable (const dString& modifiers, const dUserVariable& type, const dString& name)
{
/*
	dUserVariable variableName(NewVariableStatement (name, modifiers));

	dDAGParameterNode* const variableNameNode = (dDAGParameterNode*)variableName.m_node;
	dAssert (variableNameNode->IsType(dDAGParameterNode::GetRttiType()));

	dDAGTypeNode* const typeNode = (dDAGTypeNode*)type.m_node;
	dAssert (typeNode->GetTypeId() == dDAGTypeNode::GetRttiType());
	variableNameNode->SetType(typeNode);

	dDAGClassNode* const curClass = GetCurrentClass();
	curClass->AddVariable(variableNameNode);
*/
	dUserVariable returnNode (NewExpressionNodeVariable (name, modifiers));
	dAssert (returnNode.m_node->GetTypeId() == dDAGExpressionNodeVariable::GetRttiType());
	dDAGExpressionNodeVariable* const node = (dDAGExpressionNodeVariable*) returnNode.m_node;
	node->SetType((dDAGTypeNode*)type.m_node);
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::ConcatenateVariables(const dUserVariable& classVariable, const dUserVariable& variable)
{
	dUserVariable returnNode (variable);

	dDAGExpressionClassVariable* const classVariableNode  = (dDAGExpressionClassVariable*)classVariable.m_node;
	dAssert (classVariableNode->GetTypeId() == dDAGExpressionClassVariable::GetRttiType());
	dDAGExpressionNodeVariable* const nodeA = (dDAGExpressionNodeVariable*)classVariableNode->m_expression;
	dDAGExpressionNodeVariable* const nodeB = (dDAGExpressionNodeVariable*)variable.m_node;

	dAssert (nodeA->GetTypeId() == dDAGExpressionNodeVariable::GetRttiType());
	dAssert (nodeB->GetTypeId() == dDAGExpressionNodeVariable::GetRttiType());

	nodeB->SetType((dDAGTypeNode*)nodeA->m_type->Clone(m_allNodes));
	nodeB->InitParam (*nodeA);
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::AddClassVariableInitilization(const dUserVariable& statement)
{
/*
//	dUserVariable returnNode;
	dDAGScopeBlockNode* const block = GetCurrentScope();

	dDAGFunctionStatement* const statementNode = (dDAGFunctionStatement*)statement.m_node;
	if (statementNode) {
		if (statementNode->IsType(dDAGExpressionNode::GetRttiType())) {
			for (dDAGParameterNode* node = (dDAGParameterNode*) statementNode; node; node = (dDAGParameterNode*) node->m_next) {
				block->AddStatement(node);
			}
		} else {
			dAssert (statementNode->IsType(dDAGFunctionStatement::GetRttiType()));
			block->AddStatement(statementNode);
		}
	}

	returnNode.m_node = block;
	//	return returnNode;
*/

	dUserVariable returnNode;
	returnNode.m_node = new dDAGExpressionClassVariable(m_allNodes, (dDAGExpressionNode*)statement.m_node);
	dDAGClassNode* const curClass = GetCurrentClass();
	curClass->AddVariable ((dDAGExpressionClassVariable*)returnNode.m_node);
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::ConcatenateExpressions(const dUserVariable& expressionA, const dUserVariable& expressionB)
{
	dUserVariable returnNode;
	dDAGExpressionNode* const nodeA = (dDAGExpressionNode*)expressionA.m_node;
	dDAGExpressionNode* const nodeB = (dDAGExpressionNode*)expressionB.m_node;
	dAssert (nodeA->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (nodeB->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGExpressionNode* nextNode = nodeA;
	for (; nextNode->m_next; nextNode = (dDAGExpressionNode*) nextNode->m_next);
	nextNode->m_next = nodeB;

	dDAGExpressionNodeVariable* const leftVarListA = nodeA->FindLeftVariable();
	if (leftVarListA && leftVarListA->m_type) {
		dDAGExpressionNodeVariable* const leftVarListB = nodeB->FindLeftVariable();
		if (leftVarListB && !leftVarListB->m_type) {

			dUserVariable localVariable(NewVariableStatement (leftVarListB->m_name, ""));

			dDAGParameterNode* const variableNode = (dDAGParameterNode*)localVariable.m_node;
			dAssert (variableNode->IsType(dDAGParameterNode::GetRttiType()));
			variableNode->SetType((dDAGTypeNode*) leftVarListA->m_type->Clone(m_allNodes));
			leftVarListB->SetType((dDAGTypeNode*) leftVarListA->m_type->Clone(m_allNodes));

			if (m_scopeStack.GetCount()) {
				dDAGScopeBlockNode* const block = GetCurrentScope();
				block->AddStatement(variableNode);
			} else {
				dAssert (0);
				//dDAGClassNode* const curClass = GetCurrentClass();
				//curClass->AddVariable(variableNode);
			}
		}
	}
	returnNode.m_node = nodeA;

	return returnNode;
}




dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EndScopeBlock ()
{
	dUserVariable returnNode;
	returnNode.m_node = GetCurrentScope();
	m_scopeStack.Remove(m_scopeStack.GetLast());
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpresionNodeAssigment (const dUserVariable& leftVariable, const dUserVariable& expression)
{
	dUserVariable returnNode;

	dDAGExpressionNodeVariable* const leftNode = (dDAGExpressionNodeVariable*) leftVariable.m_node;
	dAssert (leftNode->IsType(dDAGExpressionNodeVariable::GetRttiType()));

	dDAGExpressionNode* const expressionNode = (dDAGExpressionNode*) expression.m_node;
	dAssert (expressionNode->IsType(dDAGExpressionNode::GetRttiType()));
	dDAGExpressionNodeAssigment* const assigment = new dDAGExpressionNodeAssigment(m_allNodes, leftNode, expressionNode);

	returnNode.m_node = assigment;
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpresionNodeAssigment (const dUserVariable& leftVariable, const dUserVariable& assigmentOperator, const dUserVariable& expression)
{
	dUserVariable leftVariableCopy;

	dDAGExpressionNodeVariable* const leftNode = (dDAGExpressionNodeVariable*) leftVariable.m_node;
	dAssert (leftNode->IsType(dDAGExpressionNodeVariable::GetRttiType()));


	dUserVariable tmpOperator;
	switch (assigmentOperator.m_token) 
	{
		case _ASS_ADD:
		{
			tmpOperator.m_token = dToken ('+');
			break;
		}
		case _ASS_SUB:
		{
			tmpOperator.m_token = dToken ('-');
			break;
		}
		case _ASS_MUL: 
		{
			tmpOperator.m_token = dToken ('*');
			break;
		}
		case _ASS_DIV: 
		{
			tmpOperator.m_token = dToken ('/');
			break;
		}
		case _ASS_MOD:
		{
			tmpOperator.m_token = dToken ('%');
			break;
		}
		
		case _ASS_SHL:
		{
			tmpOperator.m_token = dToken ('<<');
			break;
		}

		case _ASS_SHR:
		{
			tmpOperator.m_token = dToken ('>>');
			break;
		}

		case _ASS_AND:
		{
			tmpOperator.m_token = dToken ('&');
			break;
		}

		case _ASS_XOR:
		{
			tmpOperator.m_token = dToken ('^');
			break;
		}

		case _ASS_OR:
		{
			tmpOperator.m_token = dToken ('|');
			break;
		}

		default:
			dAssert (0);
	}

	dUserVariable expressionA;
	expressionA.m_node = leftNode->Clone (m_allNodes);
	return NewExpresionNodeAssigment (leftVariable, NewExpressionNodeBinaryOperator (expressionA, tmpOperator, expression));
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewReturnStatement(const dUserVariable& expression)
{
	dUserVariable returnNode;

	dDAGExpressionNode* const expresionNode = (dDAGExpressionNode*) expression.m_node;
	dDAGFunctionStatementReturn* const stmt = new dDAGFunctionStatementReturn(m_allNodes, expresionNode);

	returnNode.m_node = stmt;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewExpressionFunctionCall (const dString& name, const dUserVariable& argumnetList)
{
	dUserVariable returnNode;

	//dAssert (m_currentFunction);
	dDAGExpressionNode* const argumentListNode = (dDAGExpressionNode*) argumnetList.m_node;
	dAssert (!argumentListNode || argumentListNode->IsType(dDAGExpressionNode::GetRttiType()));
	dDAGExpressionNodeFunctionCall* const fntCall = new dDAGExpressionNodeFunctionCall(m_allNodes, name.GetStr(), argumentListNode);

	returnNode.m_node = fntCall;
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewIFStatement(const dUserVariable& expression, const dUserVariable& thenExpression, const dUserVariable& elseExpression)
{
	dUserVariable returnNode;

	dDAGExpressionNode* const expresionNode = (dDAGExpressionNode*) expression.m_node;
	dAssert (expresionNode->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGFunctionStatement* const thenStmt = (dDAGFunctionStatement*) thenExpression.m_node;

	dDAGFunctionStatement* elseStmt = NULL;
	if (elseExpression.m_node) {
		elseStmt = (dDAGFunctionStatement*) elseExpression.m_node;
	}

	dDAGFunctionStatementIF* const stmt = new dDAGFunctionStatementIF(m_allNodes, expresionNode, thenStmt, elseStmt);

	dAssert (thenStmt->IsType(dDAGFunctionStatement::GetRttiType()) || thenStmt->IsType(dDAGExpressionNode::GetRttiType()));
	returnNode.m_node = stmt;
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewDoStatement(const dUserVariable& expression, const dUserVariable& statement)
{
	dDAGExpressionNode* const conditionalNode = (dDAGExpressionNode*) expression.m_node;
	dDAGFunctionStatement* const bodyStmt = (dDAGFunctionStatement*) statement.m_node;

	dAssert (conditionalNode->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (!bodyStmt || bodyStmt->IsType(dDAGFunctionStatement::GetRttiType()) || bodyStmt->IsType(dDAGExpressionNode::GetRttiType()));

	dUserVariable doVariable;
	doVariable.m_node = new dDAGFunctionStatementDO(m_allNodes, conditionalNode, bodyStmt);
	return doVariable;
}



dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewForStatement(const dUserVariable& init_exp, const dUserVariable& conditional, const dUserVariable& step_Exp, const dUserVariable& statement)
{
	dDAGExpressionNode* const conditionalNode = (dDAGExpressionNode*) conditional.m_node;
	dDAGFunctionStatement* const initStmtNode = (dDAGFunctionStatement*) init_exp.m_node;
	dDAGFunctionStatement* const stepStmtNode = (dDAGFunctionStatement*) step_Exp.m_node;
	dDAGFunctionStatement* const bodyStmt = (dDAGFunctionStatement*) statement.m_node;

	dAssert (!conditionalNode || conditionalNode->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (!initStmtNode || initStmtNode->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (!stepStmtNode || stepStmtNode->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (!bodyStmt || bodyStmt->IsType(dDAGFunctionStatement::GetRttiType()) || bodyStmt->IsType(dDAGExpressionNode::GetRttiType()));

	dUserVariable forVariable;
	forVariable.m_node = new dDAGFunctionStatementFOR(m_allNodes, initStmtNode, conditionalNode, stepStmtNode, bodyStmt);
	return forVariable;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewWhileStatement(const dUserVariable& expression, const dUserVariable& statement)
{
	dDAGExpressionNode* const conditionalNode = (dDAGExpressionNode*) expression.m_node;
	dDAGFunctionStatement* const bodyStmt = (dDAGFunctionStatement*) statement.m_node;

	dAssert (conditionalNode->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (!bodyStmt || bodyStmt->IsType(dDAGFunctionStatement::GetRttiType()) || bodyStmt->IsType(dDAGExpressionNode::GetRttiType()));

	dUserVariable whileNode;
	whileNode.m_node = new dDAGFunctionStatementWHILE(m_allNodes, conditionalNode, bodyStmt);
	return whileNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewBreakStatement()
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGFunctionStatementBREAK(m_allNodes);
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewContinueStatement()
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGFunctionStatementCONTINUE(m_allNodes);
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewEmptyStatement()
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGFunctionStatement(m_allNodes);
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::ConcatenateCaseStatement (const dUserVariable& firstStatement, const dUserVariable& lastStatement)
{
	dUserVariable returnNode;

	dDAGFunctionStatement* const node = (dDAGFunctionStatement*)firstStatement.m_node;
	dAssert (node->IsType(dDAGFunctionStatement::GetRttiType()));

	dDAGFunctionStatement* nextNode = node;
	for (; nextNode->m_next; nextNode = (dDAGFunctionStatement*) nextNode->m_next);
	nextNode->m_next = (dDAGFunctionStatement*)lastStatement.m_node;
	dAssert (nextNode->m_next->IsType(dDAGFunctionStatement::GetRttiType()));

	returnNode.m_node = node;
	return returnNode;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::ConcatenateCaseBlocks (const dUserVariable& firstStatement, const dUserVariable& lastStatement)
{
	return ConcatenateCaseStatement (firstStatement, lastStatement);
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::ConcatenateParametersExpressions(const dUserVariable& expressionA, const dUserVariable& expressionB)
{
	dUserVariable returnNode;

	dAssert (expressionA.m_node->IsType(dDAGExpressionNode::GetRttiType()));
	dAssert (expressionB.m_node->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGExpressionNode* const node = (dDAGExpressionNode*)expressionA.m_node;

	dDAGExpressionNode* nextNode = node;
	for (; nextNode->m_next; nextNode = (dDAGExpressionNode*) nextNode->m_next);
	nextNode->m_next = expressionB.m_node;
	
	returnNode.m_node = node;
	return returnNode;
}



dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewCaseStatement(const dString& constID, const dUserVariable& statementList)
{
	dUserVariable returnNode;

	dDAGFunctionStatement* const statementListNode = (dDAGFunctionStatement*) statementList.m_node;
	dAssert (statementListNode->IsType(dDAGFunctionStatement::GetRttiType()));

	dDAGFunctionStatementCase* const caseNode = new dDAGFunctionStatementCase(m_allNodes, constID.GetStr(), statementListNode);

dAssert (0);
	for (dDAGFunctionStatement* nextNode = statementListNode; nextNode; nextNode = (dDAGFunctionStatement*) nextNode->m_next) {
//		nextNode->Release();
	}

	returnNode.m_node = caseNode;
	return returnNode;
}


dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::NewSwitchStatement(const dUserVariable& expression, const dUserVariable& caseStatementList)
{
	dUserVariable returnNode;

	dDAGFunctionStatementCase* const casetListNode = (dDAGFunctionStatementCase*) caseStatementList.m_node;
	dAssert (casetListNode->IsType(dDAGFunctionStatementCase::GetRttiType()));

	dDAGExpressionNode* const expressionNode = (dDAGExpressionNode*) expression.m_node;
	dAssert (expressionNode->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGFunctionStatementSWITCH* const switchNode = new dDAGFunctionStatementSWITCH (m_allNodes, expressionNode, casetListNode);

dAssert (0);
//	expressionNode->Release();
	for (dDAGFunctionStatement* nextNode = casetListNode; nextNode; nextNode = (dDAGFunctionStatement*) nextNode->m_next) {
		dAssert (0);
//		_ASSERTE (nextNode->IsType(dDAGFunctionStatementCase::GetRttiType()));
//		nextNode->Release();
	}

	returnNode.m_node = switchNode;
	return returnNode;
}



void dNewtonLuaCompiler::OpenPackage (const dString& packageName)
{
	dAssert (0);
/*
	m_packageFileName = m_packageRootDirectory + dString ("/") + dString (packageName);

	for (int i = m_packageFileName.Find('_'); i >= 0; i = m_packageFileName.Find('_')) {
		m_packageFileName.Replace (i, 1, "");
	}

	for (int i = m_packageFileName.Find('.'); i >= 0; i = m_packageFileName.Find('.')) {
		m_packageFileName.Replace (i, 1, "/");
	}

	m_packageFileName += ".pkg";

	m_currentPackage = new dScriptPackage;
	if (_access(m_packageFileName.GetStr(), 0) != -1) {
		// load package does not exist, make a new package 
		m_currentPackage->Load (m_packageFileName.GetStr());	
	}
*/
}

#endif


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
//	m_funtions.Insert(function->GetNode(), functionName.GetString());

	dUserVariable variable(functionName);
	variable.m_node = function->GetNode();
	function->Trace();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitFunctionParameter(const dUserVariable& parameter)
{
	dCILInstrArgument* const localVariable = new dCILInstrArgument(*m_currentClosure, parameter.GetString(), dCILInstr::dArgType(dCILInstr::m_luaType));
	dUserVariable variable(parameter);
	variable.m_node = localVariable->GetNode();
	localVariable->Trace();
	return variable;
}

dNewtonLuaCompiler::dUserVariable dNewtonLuaCompiler::EmitParametersToLocalVariables(const dUserVariable& parameterList)
{
//	dCILInstrArgument* const localVariable = new dCILInstrArgument(m_cil, parameter.GetString(), dCILInstr::dArgType(dCILInstr::m_luaType));
//	dUserVariable variable(parameter);
//	variable.m_node = localVariable->GetNode();
//	variable.m_next = predecessor;
//	localVariable->Trace();
/*
	for (dList<dDAGParameterNode*>::dListNode* argNode = parameterList; argNode; argNode = argNode->GetNext()) {
		dDAGParameterNode* const arg = argNode->GetInfo();
//		dTree<dCILInstr::dArg, dString>::dTreeNode* const varNameNode = m_body->FindVariable(arg->m_name);
//		dAssert(varNameNode);
		dCILInstrArgument* const localVariable = new dCILInstrArgument(m_cil, varNameNode->GetInfo().m_label, dCILInstr::dArgType(dCILInstr::m_luaType));
		localVariable->Trace();
	}

	for (dCIL::dListNode* argNode = parameterList.m_node; argNode; argNode = argNode->GetInfo()->) {
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
*/
	return parameterList;
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

	dCILInstrArgument* definition = NULL;
	for (dCIL::dListNode* node = m_currentClosure->GetFirst(); node; node = node->GetNext()) {
		dCILInstrArgument* const intruction = node->GetInfo()->GetAsArgument();
		if (intruction) {
			dCILInstr::dArg* const argName = intruction->GetGeneratedVariable();
			if (argName->m_label == varName.GetString()) {
				definition = intruction;
				break;
			}
		}
	}

	dAssert(definition);
	dString outVarName(m_currentClosure->NewTemp());
	dCILInstr::dArgType type(dCILInstr::m_luaType);
	dCILInstrMove* const move = new dCILInstrMove(*m_currentClosure, outVarName, type, varName.GetString(), type);
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

	dString label1(m_currentClosure->NewLabel());
	dString label2(m_currentClosure->NewLabel());

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
/*
	dCILInstr::dArgType type(dCILInstr::m_luaType);

	dString label1(m_currentClosure->NewLabel());
	dString label2(m_currentClosure->NewLabel());

	dCILInstrConditional* const conditional = new dCILInstrConditional(*m_currentClosure, dCILInstrConditional::m_ifnot, expression.GetString(), type, label1, label2);
	dCILInstrLabel* const taget2 = new dCILInstrLabel(*m_currentClosure, label2);
	conditional->SetTargets(taget2, taget2);
*/
	dAssert(ifStatement.m_node);
	dCILInstrConditional* const conditional = ifStatement.m_node->GetInfo()->GetAsIF();
	dAssert(conditional);

//	dList<dCILInstr*>::dListNode* GetTrueTarget() const;
//	dList<dCILInstr*>::dListNode* GetFalseTarget() const;

	dCILInstrLabel* const target = new dCILInstrLabel(*m_currentClosure, conditional->GetArg1().m_label);
	conditional->SetTargets(target, conditional->GetTrueTarget()->GetInfo()->GetAsLabel());

	dUserVariable variable;
	variable.m_node = target->GetNode();
//	conditional->Trace();
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