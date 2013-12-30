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

//
//Auto generated Parser Generator class: dScriptCompiler.cpp
//

#include "dLSCstdafx.h"
#include "dLittleScriptLexical.h"
#include "dLittleScriptCompiler.h"

#include "dDAG.h"
#include "dDAGTypeNode.h"
#include "dDAGClassNode.h"
#include "dDAGFunctionNode.h"
#include "dDAGParameterNode.h"
#include "dDAGDimensionNode.h"
#include "dDAGScopeBlockNode.h"
#include "dDAGExpressionNode.h"
#include "dDAGFunctionModifier.h"
#include "dDAGFunctionStatement.h"
#include "dDAGExpressionNodeNew.h"
#include "dDAGFunctionStatementIF.h"
#include "dDAGFunctionStatementDO.h"
#include "dDAGFunctionStatementFOR.h"
#include "dDAGFunctionStatementFlow.h"
#include "dDAGFunctionStatementCase.h"
#include "dDAGFunctionStatementBREAK.h"
#include "dDAGFunctionStatementWHILE.h"
#include "dDAGExpressionNodeConstant.h"
#include "dDAGExpressionNodeVariable.h"
#include "dDAGFunctionStatementSWITCH.h"
#include "dDAGFunctionStatementReturn.h"
#include "dDAGFunctionStatementCONTINUE.h"
#include "dDAGExpressionNodeFunctionCall.h"
#include "dDAGExpressionNodeAssigment.h"
#include "dDAGExpressionNodePrefixPostfix.h"
#include "dDAGExpressionNodeLogicOperator.h"
#include "dDAGExpressionNodeBinaryOperator.h"


#include "dScriptPackage.h"


void* operator new (size_t size)
{
	void* const ptr = malloc (size);
	return ptr;
}


dScriptCompiler::dScriptCompiler(const char* const pakacgesRootNameDirectory)
	:dLittleScriptParser ()
	,m_packageRootDirectory (pakacgesRootNameDirectory)
	,m_currentPackage(NULL)
	,m_currentFunction(NULL)
	,m_classList()
	,m_scopeStack()
	,m_allNodes()
{
	_mkdir(m_packageRootDirectory.GetStr());
}

dScriptCompiler::~dScriptCompiler()
{
	for(dList<dDAG*>::dListNode* nodePtr = m_allNodes.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext() ) {
		dDAG* const node = nodePtr->GetInfo();
		delete node;
	}

	if (m_currentPackage) {
		m_currentPackage->Save (m_packageFileName.GetStr());

		delete m_currentPackage;
	}
}


#if 0


void dScriptCompiler::DisplayError (const char* format, ...) const
{
	va_list v_args;
	char* const text = (char*) malloc (strlen (format) + 2048);

	text[0] = 0;
	va_start (v_args, format);     
	vsprintf(text, format, v_args);
	va_end (v_args);            

	fprintf (stderr, text);
#ifdef _MSC_VER  
	OutputDebugStringA (text);
#endif	

	free (text);
}

void dScriptCompiler::SyntaxError (const dLittleScriptLexical& scanner, const dUserVariable& errorToken, const dUserVariable& errorTokenMarker)
{
	const char* const data = scanner.GetData();
	int start = errorToken.m_scannerIndex;
	int lineNumber = errorToken.m_scannerLine + 1;
	while (data[start] && isspace (data[start])) {
		if (data[start] == '\n') {
			lineNumber ++;
		}
		start ++;
	}

	int end = errorTokenMarker.m_scannerIndex;
	while (data[end] && isspace (data[end])) {
		end --;
	}
	_ASSERTE (end >= start);

	int length = end - start + 1;
	dString errorLine (&data[start], length);
	DisplayError ("%s (%d) : syntax error on line: %s\n", m_fileName, lineNumber, errorLine.GetStr());
}


#endif

// parcel callbacks section
bool dScriptCompiler::Parse(dLittleScriptLexical& scanner)
{
	return dLittleScriptParser::Parse(scanner);
}


int dScriptCompiler::CompileSource (const char* const source)
{
	dLittleScriptLexical scanner (source);

	bool status = Parse(scanner);

	if (status) {
		for (dList<dDAGClassNode*>::dListNode* node = m_classList.GetFirst(); node; node = node->GetNext()) {
			dDAGClassNode* const scripClass =  node->GetInfo();
			scripClass->ConnectParent (NULL);
		}

		
		for (dList<dDAGClassNode*>::dListNode* node = m_classList.GetFirst(); node; node = node->GetNext()) {
			dDAGClassNode* const scripClass = node->GetInfo();
			dCIL cil;
			scripClass->CompileCIL (cil);

			dTrace(("\n"));
			dTrace(("optimized version\n"));
			cil.Trace();

			//_ASSERTE (m_currentPackage);
			//m_currentPackage->AddClass(scripClass, cil);
		}
	}
	return 0;
}

void dScriptCompiler::ImportClass (const dString& className)
{
}

void dScriptCompiler::ImportAllClasses (const dString& className)
{
}





dDAGScopeBlockNode* dScriptCompiler::GetCurrentScope() const
{
	_ASSERTE (m_scopeStack.GetCount());
	return m_scopeStack.GetLast()->GetInfo();
}


dScriptCompiler::dUserVariable dScriptCompiler::NewExpressionNodeConstant (const dUserVariable& value)
{
	dUserVariable returnNode;

	dDAGExpressionNodeConstant::dType type = dDAGExpressionNodeConstant::m_intValue;
	switch (int (value.m_token))
	{
		case _FLOAT_CONST:
			type = dDAGExpressionNodeConstant::m_floatValue;
			break;

		case _INTEGER_CONST:
			type = dDAGExpressionNodeConstant::m_intValue;
			break;
/*
		case STRING_VALUE:
			type = dDAGExpressionNodeConstant::m_stringValue;
			break;
*/
		default:
			_ASSERTE (0);
	}

//	_ASSERTE (GetCurrentScope());
	dDAGExpressionNodeConstant* const node = new dDAGExpressionNodeConstant (m_allNodes, type, value.m_data.GetStr());
	returnNode.m_node = node;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewExpresionNodePrefixPostfixOperator (const dUserVariable& expression, bool isPrefix, bool isIncrement)
{
	dUserVariable returnNode;
	_ASSERTE (expression.m_node && expression.m_node->IsType(dDAGExpressionNodeVariable::GetRttiType()));
	
	dDAGExpressionNodePrefixPostfix* const node = new dDAGExpressionNodePrefixPostfix (m_allNodes, (dDAGExpressionNode*)expression.m_node, isPrefix, isIncrement);
	returnNode.m_node = node;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::NewExpressionNodeBinaryOperator (const dUserVariable& expressionA, const dUserVariable& binaryOperator, const dUserVariable& expressionB)
{
	dUserVariable returnNode;

	_ASSERTE (expressionA.m_node && expressionA.m_node->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (expressionB.m_node && expressionB.m_node->IsType(dDAGExpressionNode::GetRttiType()));

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
			_ASSERTE (0);
	}


//	_ASSERTE (GetCurrentScope());
	dDAGExpressionNodeBinaryOperator* const node = new dDAGExpressionNodeBinaryOperator (m_allNodes, binOperator, (dDAGExpressionNode*)expressionA.m_node, (dDAGExpressionNode*)expressionB.m_node);
	returnNode.m_node = node;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::NewExpressionNodeLogiOperator (const dUserVariable& expressionA, const dUserVariable& logicOperator, const dUserVariable& expressionB)
{
	dUserVariable returnNode;

	_ASSERTE (expressionA.m_node && expressionA.m_node->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (expressionB.m_node && expressionB.m_node->IsType(dDAGExpressionNode::GetRttiType()));

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
			_ASSERTE (0);
	}

	_ASSERTE (GetCurrentScope());
	dDAGExpressionNodeLogicOperator* const node = new dDAGExpressionNodeLogicOperator (m_allNodes, logOperator, (dDAGExpressionNode*)expressionA.m_node, (dDAGExpressionNode*)expressionB.m_node);
_ASSERTE (0);
//	expressionA.m_node->Release();
//	expressionB.m_node->Release();
	returnNode.m_node = node;
	return returnNode;
}



dDAGClassNode* dScriptCompiler::GetCurrentClass() const
{
	_ASSERTE (m_classList.GetCount());
	return m_classList.GetLast()->GetInfo();
}

dScriptCompiler::dUserVariable dScriptCompiler::CreateClass (const dString& visibility, const dString& callType, const dString& className, const dString& superClassName, const dString& interfaces)
{
	dUserVariable returnNode;
	dDAGClassNode* const classNode = new dDAGClassNode (m_allNodes);
	m_classList.Append(classNode);
	returnNode.m_node = classNode;
	classNode->FinalizeImplementation (visibility.GetStr(), className.GetStr(), NULL);
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::EmitTypeNode (const dUserVariable& type, const dUserVariable& dim)
{
	dUserVariable returnNode;
	dDAGTypeNode* const typeNode = new dDAGTypeNode (m_allNodes, type.m_data.GetStr());

	if (dim.m_node) {
		dDAGDimensionNode* const dimList = (dDAGDimensionNode*) dim.m_node;
		_ASSERTE (dimList->IsType(dDAGDimensionNode::GetRttiType()));
		typeNode->AddDimensions (dimList);
	}

	returnNode.m_node = typeNode;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::AddClassFunction (const dUserVariable& returnType, const dString& name, const dString& visibility)
{
	dUserVariable returnNode;

	dDAGTypeNode* const typeNode = (dDAGTypeNode*) returnType.m_node;
	_ASSERTE (typeNode->IsType(dDAGTypeNode::GetRttiType()));

	dDAGFunctionNode* const functionNode = new dDAGFunctionNode (m_allNodes, typeNode, name.GetStr(), visibility.GetStr());
	GetCurrentClass()->AddFunction(functionNode);

	returnNode.m_node = functionNode;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::AddClassContructor (const dString& name, const dString& visibility)
{
	dDAGClassNode* const curClass = GetCurrentClass();

	dUserVariable typeVariable;
	_ASSERTE (name == curClass->m_name);

	typeVariable.m_node = new dDAGTypeNode (m_allNodes, "void");
	dUserVariable returnNode (AddClassFunction (typeVariable, name, "public static"));

	curClass->m_constructors.Append((dDAGFunctionNode*)returnNode.m_node);
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::FunctionAddParameterNode (const dUserVariable& parameter)
{
	dUserVariable returnNode;

	dDAGParameterNode* const parameterNode = (dDAGParameterNode*)parameter.m_node;
	_ASSERTE (parameterNode->GetTypeId() == dDAGParameterNode::GetRttiType());

	dDAGFunctionNode* const function = GetCurrentClass()->GetCurrentFunction();
	function->AddParameter(parameterNode);
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::FunctionAddBodyBlock (const dUserVariable& functionBody)
{
	dUserVariable returnNode;

	dDAGFunctionNode* const function = GetCurrentClass()->GetCurrentFunction();

	dDAGScopeBlockNode* const bodyNode = (dDAGScopeBlockNode*) functionBody.m_node;
	_ASSERTE (bodyNode->IsType (dDAGScopeBlockNode::GetRttiType()));
	function->SetBody(bodyNode);

	returnNode.m_node = function;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewDimensionNode(const dUserVariable& expression)
{
	dUserVariable returnNode;
	dDAGExpressionNode* const expressionNode = (dDAGExpressionNode*)expression.m_node;
	_ASSERTE (!expressionNode || (expressionNode->IsType (dDAGExpressionNode::GetRttiType())));
	dDAGDimensionNode* const node = new dDAGDimensionNode (m_allNodes, expressionNode);

	returnNode.m_node = node;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::NewExpressionOperatorNew (const dString& typeName, const dUserVariable& dimension)
{
	dUserVariable returnNode;

	dDAGDimensionNode* const dimensionNode = (dDAGDimensionNode*) dimension.m_node;
	_ASSERTE (dimensionNode->IsType (dDAGDimensionNode::GetRttiType()));
	dDAGExpressionNodeNew* const node = new dDAGExpressionNodeNew (m_allNodes, typeName.GetStr(), dimensionNode);

	returnNode.m_node = node;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::ConcatenateDimensionNode(const dUserVariable& dim0, const dUserVariable& dim1)
{
	dUserVariable returnNode;

	_ASSERTE (dim0.m_node->GetTypeId() == dDAGDimensionNode::GetRttiType());
	_ASSERTE (dim1.m_node->GetTypeId() == dDAGDimensionNode::GetRttiType());

	dDAGDimensionNode* param = (dDAGDimensionNode*) dim0.m_node;
	for ( ;param->m_next; param = param->m_next);
	param->m_next = (dDAGDimensionNode*) dim1.m_node;

	returnNode.m_node = dim0.m_node;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::NewParameterNode (const dUserVariable& primitiveType, const dString& name)
{
	dUserVariable returnNode;

	dDAGParameterNode* const parameter = new dDAGParameterNode(m_allNodes, name.GetStr());
	dDAGTypeNode* const typeNode = (dDAGTypeNode*) primitiveType.m_node;
	if (typeNode) {
		_ASSERTE (typeNode->GetTypeId() == dDAGTypeNode::GetRttiType());
		parameter->SetType(typeNode);
	}
	returnNode.m_node = parameter;
	return returnNode;
}



dScriptCompiler::dUserVariable dScriptCompiler::BeginScopeBlock ()
{
	dUserVariable returnNode;
	dDAGScopeBlockNode* const scope = new dDAGScopeBlockNode (m_allNodes);
	m_scopeStack.Append(scope);
	returnNode.m_node = scope;
	return returnNode;
}

/*
*/

dScriptCompiler::dUserVariable dScriptCompiler::NewExpressionNodeVariable (const dString& name, const dUserVariable& dimArray)
{
	dUserVariable returnNode;

//	_ASSERTE (GetCurrentScope());
	dDAGDimensionNode* const dimensionNode = (dDAGDimensionNode*) dimArray.m_node;
	_ASSERTE (!dimensionNode || dimensionNode->IsType(dDAGDimensionNode::GetRttiType()));

	dDAGExpressionNodeVariable* const node = new dDAGExpressionNodeVariable (m_allNodes, name.GetStr(), dimensionNode);
	returnNode.m_node = node;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::NewVariableStatement(const dString& name)
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGParameterNode (m_allNodes, name.GetStr());
	return returnNode;
}

/*
dScriptCompiler::dUserVariable dScriptCompiler::SetLocalVariablesType (const dUserVariable& type, const dUserVariable& variableList)
{
	dUserVariable returnNode;

	dDAGParameterNode* const node = (dDAGParameterNode*)variableList.m_node;
	_ASSERTE (node->GetTypeId() == dDAGParameterNode::GetRttiType());

	dDAGTypeNode* const typeNode = (dDAGTypeNode*)type.m_node;
	_ASSERTE (typeNode->GetTypeId() == dDAGTypeNode::GetRttiType());

	for (dDAGFunctionStatement* nextNode = node; nextNode; nextNode = nextNode->m_next) {
		_ASSERTE (nextNode->GetTypeId() == dDAGParameterNode::GetRttiType());
		((dDAGParameterNode*)nextNode)->SetType(typeNode);
	}
	_ASSERTE (0);
	//	typeNode->Release();

	returnNode.m_node = node;
	return returnNode;
}
*/

dScriptCompiler::dUserVariable dScriptCompiler::ConcatenateVariables(const dUserVariable& variableA, const dUserVariable& variableB)
{
	dAssert (0);
	dUserVariable returnNode;

	dDAGFunctionStatement* const node = (dDAGFunctionStatement*)variableA.m_node;
	_ASSERTE (node->GetTypeId() == dDAGParameterNode::GetRttiType());

	dDAGFunctionStatement* nextNode = node;
	for (; nextNode->m_next; nextNode = (dDAGFunctionStatement*)nextNode->m_next);
	nextNode->m_next = variableB.m_node;
	_ASSERTE (nextNode->m_next->IsType(dDAGFunctionStatement::GetRttiType()));

	returnNode.m_node = node;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::AddClassVariable (const dString& modifiers, const dUserVariable& type, const dUserVariable& variableList)
{
	dAssert (0);

	dUserVariable returnNode;

	dDAGParameterNode* const node = (dDAGParameterNode*)variableList.m_node;
	_ASSERTE (node->GetTypeId() == dDAGParameterNode::GetRttiType());

	dDAGTypeNode* const typeNode = (dDAGTypeNode*)type.m_node;
	_ASSERTE (typeNode->GetTypeId() == dDAGTypeNode::GetRttiType());

	node->SetType(typeNode);
	for (dDAGParameterNode* nextNode = (dDAGParameterNode*) node->m_next; nextNode; nextNode = (dDAGParameterNode*) nextNode->m_next) {
		_ASSERTE (nextNode->GetTypeId() == dDAGParameterNode::GetRttiType());
		nextNode->SetType((dDAGTypeNode*) typeNode->Clone (m_allNodes));
	}

	dDAGClassNode* const curClass = GetCurrentClass();
	for (dDAGParameterNode* nextNode = (dDAGParameterNode*) node; nextNode; nextNode = (dDAGParameterNode*) nextNode->m_next) {
		curClass->AddVariable(nextNode);
	}
	return returnNode;
}



dScriptCompiler::dUserVariable dScriptCompiler::NewVariableToCurrentBlock (const dString& modifiers, const dUserVariable& type, const dString& name)
{
	dUserVariable variableName(NewVariableStatement (name));

	dDAGParameterNode* const variableNameNode = (dDAGParameterNode*)variableName.m_node;
	_ASSERTE (variableNameNode->IsType(dDAGParameterNode::GetRttiType()));

	dDAGTypeNode* const typeNode = (dDAGTypeNode*)type.m_node;
	_ASSERTE (typeNode->GetTypeId() == dDAGTypeNode::GetRttiType());
	variableNameNode->SetType(typeNode);

	if (m_scopeStack.GetCount()) {
		dDAGScopeBlockNode* const block = GetCurrentScope();
		block->AddStatement(variableNameNode);
	} else {
		dDAGClassNode* const curClass = GetCurrentClass();
		curClass->AddVariable(variableNameNode);
	}

	dUserVariable returnNode (NewExpressionNodeVariable (name));
	_ASSERTE (returnNode.m_node->GetTypeId() == dDAGExpressionNodeVariable::GetRttiType());
	dDAGExpressionNodeVariable* const node = (dDAGExpressionNodeVariable*) returnNode.m_node;
	node->SetType((dDAGTypeNode*) typeNode->Clone (m_allNodes));

	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::AddStatementToCurrentBlock(const dUserVariable& statement)
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
			_ASSERTE (statementNode->IsType(dDAGFunctionStatement::GetRttiType()));
			block->AddStatement(statementNode);
		}
	}

	returnNode.m_node = block;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::ConcatenateExpressions(const dUserVariable& expressionA, const dUserVariable& expressionB)
{
	dUserVariable returnNode;

	dDAGExpressionNode* const nodeA = (dDAGExpressionNode*)expressionA.m_node;
	dDAGExpressionNode* const nodeB = (dDAGExpressionNode*)expressionB.m_node;
	_ASSERTE (nodeA->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (nodeB->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGExpressionNode* nextNode = nodeA;
	for (; nextNode->m_next; nextNode = (dDAGExpressionNode*) nextNode->m_next);
	nextNode->m_next = nodeB;

	dDAGExpressionNodeVariable* const leftVarListA = nodeA->FindLeftVariable();
	if (leftVarListA && leftVarListA->m_type) {
		dDAGExpressionNodeVariable* const leftVarListB = nodeB->FindLeftVariable();
		if (leftVarListB && !leftVarListB->m_type) {
			dUserVariable localVariable(NewVariableStatement (leftVarListB->m_name));

			dDAGParameterNode* const variableNode = (dDAGParameterNode*)localVariable.m_node;
			_ASSERTE (variableNode->IsType(dDAGParameterNode::GetRttiType()));
			variableNode->SetType((dDAGTypeNode*) leftVarListA->m_type->Clone(m_allNodes));
			leftVarListB->SetType((dDAGTypeNode*) leftVarListA->m_type->Clone(m_allNodes));

			if (m_scopeStack.GetCount()) {
				dDAGScopeBlockNode* const block = GetCurrentScope();
				block->AddStatement(variableNode);
			} else {
				dDAGClassNode* const curClass = GetCurrentClass();
				curClass->AddVariable(variableNode);
			}
		}
	}

	returnNode.m_node = nodeA;
	return returnNode;
}




dScriptCompiler::dUserVariable dScriptCompiler::EndScopeBlock ()
{
	dUserVariable returnNode;
	returnNode.m_node = GetCurrentScope();
	m_scopeStack.Remove(m_scopeStack.GetLast());
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewExpresionNodeAssigment (const dUserVariable& leftVariable, const dUserVariable& expression)
{
	dUserVariable returnNode;

	dDAGExpressionNodeVariable* const leftNode = (dDAGExpressionNodeVariable*) leftVariable.m_node;
	_ASSERTE (leftNode->IsType(dDAGExpressionNodeVariable::GetRttiType()));

	dDAGExpressionNode* const expressionNode = (dDAGExpressionNode*) expression.m_node;
	_ASSERTE (expressionNode->IsType(dDAGExpressionNode::GetRttiType()));
	dDAGExpressionNodeAssigment* const assigment = new dDAGExpressionNodeAssigment(m_allNodes, leftNode, expressionNode);

	returnNode.m_node = assigment;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewExpresionNodeAssigment (const dUserVariable& leftVariable, const dUserVariable& assigmentOperator, const dUserVariable& expression)
{
	dUserVariable leftVariableCopy;

	dDAGExpressionNodeVariable* const leftNode = (dDAGExpressionNodeVariable*) leftVariable.m_node;
	_ASSERTE (leftNode->IsType(dDAGExpressionNodeVariable::GetRttiType()));


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
			_ASSERTE (0);
	}

	dUserVariable expressionA;
	expressionA.m_node = leftNode->Clone (m_allNodes);
	return NewExpresionNodeAssigment (leftVariable, NewExpressionNodeBinaryOperator (expressionA, tmpOperator, expression));
}


dScriptCompiler::dUserVariable dScriptCompiler::NewReturnStatement(const dUserVariable& expression)
{
	dUserVariable returnNode;

	dDAGExpressionNode* const expresionNode = (dDAGExpressionNode*) expression.m_node;
	dDAGFunctionStatementReturn* const stmt = new dDAGFunctionStatementReturn(m_allNodes, expresionNode);

	returnNode.m_node = stmt;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::NewExpressionFunctionCall (const dString& name, const dUserVariable& argumnetList)
{
	dUserVariable returnNode;

	dDAGExpressionNode* const argumentListNode = (dDAGExpressionNode*) argumnetList.m_node;
	_ASSERTE (!argumentListNode || argumentListNode->IsType(dDAGExpressionNode::GetRttiType()));
	dDAGExpressionNodeFunctionCall* const fntCall = new dDAGExpressionNodeFunctionCall(m_allNodes, name.GetStr(), argumentListNode);

	returnNode.m_node = fntCall;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewIFStatement(const dUserVariable& expression, const dUserVariable& thenExpression, const dUserVariable& elseExpression)
{
	dUserVariable returnNode;

	dDAGExpressionNode* const expresionNode = (dDAGExpressionNode*) expression.m_node;
	_ASSERTE (expresionNode->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGFunctionStatement* const thenStmt = (dDAGFunctionStatement*) thenExpression.m_node;

	dDAGFunctionStatement* elseStmt = NULL;
	if (elseExpression.m_node) {
		elseStmt = (dDAGFunctionStatement*) elseExpression.m_node;
	}

	dDAGFunctionStatementIF* const stmt = new dDAGFunctionStatementIF(m_allNodes, expresionNode, thenStmt, elseStmt);

	_ASSERTE (thenStmt->IsType(dDAGFunctionStatement::GetRttiType()) || thenStmt->IsType(dDAGExpressionNode::GetRttiType()));
	returnNode.m_node = stmt;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewDoStatement(const dUserVariable& expression, const dUserVariable& statement)
{
	dDAGExpressionNode* const conditionalNode = (dDAGExpressionNode*) expression.m_node;
	dDAGFunctionStatement* const bodyStmt = (dDAGFunctionStatement*) statement.m_node;

	_ASSERTE (conditionalNode->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (!bodyStmt || bodyStmt->IsType(dDAGFunctionStatement::GetRttiType()) || bodyStmt->IsType(dDAGExpressionNode::GetRttiType()));

	dUserVariable doVariable;
	doVariable.m_node = new dDAGFunctionStatementDO(m_allNodes, conditionalNode, bodyStmt);
	return doVariable;
}



dScriptCompiler::dUserVariable dScriptCompiler::NewForStatement(const dUserVariable& init_exp, const dUserVariable& conditional, const dUserVariable& step_Exp, const dUserVariable& statement)
{
	dDAGExpressionNode* const conditionalNode = (dDAGExpressionNode*) conditional.m_node;
	dDAGFunctionStatement* const initStmtNode = (dDAGFunctionStatement*) init_exp.m_node;
	dDAGFunctionStatement* const stepStmtNode = (dDAGFunctionStatement*) step_Exp.m_node;
	dDAGFunctionStatement* const bodyStmt = (dDAGFunctionStatement*) statement.m_node;

	_ASSERTE (!conditionalNode || conditionalNode->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (!initStmtNode || initStmtNode->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (!stepStmtNode || stepStmtNode->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (!bodyStmt || bodyStmt->IsType(dDAGFunctionStatement::GetRttiType()) || bodyStmt->IsType(dDAGExpressionNode::GetRttiType()));

	dUserVariable forVariable;
	forVariable.m_node = new dDAGFunctionStatementFOR(m_allNodes, initStmtNode, conditionalNode, stepStmtNode, bodyStmt);
	return forVariable;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewWhileStatement(const dUserVariable& expression, const dUserVariable& statement)
{
	dDAGExpressionNode* const conditionalNode = (dDAGExpressionNode*) expression.m_node;
	dDAGFunctionStatement* const bodyStmt = (dDAGFunctionStatement*) statement.m_node;

	_ASSERTE (conditionalNode->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (!bodyStmt || bodyStmt->IsType(dDAGFunctionStatement::GetRttiType()) || bodyStmt->IsType(dDAGExpressionNode::GetRttiType()));

	dUserVariable whileNode;
	whileNode.m_node = new dDAGFunctionStatementWHILE(m_allNodes, conditionalNode, bodyStmt);
	return whileNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewBreakStatement()
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGFunctionStatementBREAK(m_allNodes);
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewContinueStatement()
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGFunctionStatementCONTINUE(m_allNodes);
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::NewEmptyStatement()
{
	dUserVariable returnNode;
	returnNode.m_node = new dDAGFunctionStatement(m_allNodes);
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::ConcatenateCaseStatement (const dUserVariable& firstStatement, const dUserVariable& lastStatement)
{
	dUserVariable returnNode;

	dDAGFunctionStatement* const node = (dDAGFunctionStatement*)firstStatement.m_node;
	_ASSERTE (node->IsType(dDAGFunctionStatement::GetRttiType()));

	dDAGFunctionStatement* nextNode = node;
	for (; nextNode->m_next; nextNode = (dDAGFunctionStatement*) nextNode->m_next);
	nextNode->m_next = (dDAGFunctionStatement*)lastStatement.m_node;
	_ASSERTE (nextNode->m_next->IsType(dDAGFunctionStatement::GetRttiType()));

	returnNode.m_node = node;
	return returnNode;
}

dScriptCompiler::dUserVariable dScriptCompiler::ConcatenateCaseBlocks (const dUserVariable& firstStatement, const dUserVariable& lastStatement)
{
	return ConcatenateCaseStatement (firstStatement, lastStatement);
}

dScriptCompiler::dUserVariable dScriptCompiler::ConcatenateParametersExpressions(const dUserVariable& expressionA, const dUserVariable& expressionB)
{
	dUserVariable returnNode;

	_ASSERTE (expressionA.m_node->IsType(dDAGExpressionNode::GetRttiType()));
	_ASSERTE (expressionB.m_node->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGExpressionNode* const node = (dDAGExpressionNode*)expressionA.m_node;

	dDAGExpressionNode* nextNode = node;
	for (; nextNode->m_next; nextNode = (dDAGExpressionNode*) nextNode->m_next);
	nextNode->m_next = expressionB.m_node;
	
	returnNode.m_node = node;
	return returnNode;
}



dScriptCompiler::dUserVariable dScriptCompiler::NewCaseStatement(const dString& constID, const dUserVariable& statementList)
{
	dUserVariable returnNode;

	dDAGFunctionStatement* const statementListNode = (dDAGFunctionStatement*) statementList.m_node;
	_ASSERTE (statementListNode->IsType(dDAGFunctionStatement::GetRttiType()));

	dDAGFunctionStatementCase* const caseNode = new dDAGFunctionStatementCase(m_allNodes, constID.GetStr(), statementListNode);

_ASSERTE (0);
	for (dDAGFunctionStatement* nextNode = statementListNode; nextNode; nextNode = (dDAGFunctionStatement*) nextNode->m_next) {
//		nextNode->Release();
	}

	returnNode.m_node = caseNode;
	return returnNode;
}


dScriptCompiler::dUserVariable dScriptCompiler::NewSwitchStatement(const dUserVariable& expression, const dUserVariable& caseStatementList)
{
	dUserVariable returnNode;

	dDAGFunctionStatementCase* const casetListNode = (dDAGFunctionStatementCase*) caseStatementList.m_node;
	_ASSERTE (casetListNode->IsType(dDAGFunctionStatementCase::GetRttiType()));

	dDAGExpressionNode* const expressionNode = (dDAGExpressionNode*) expression.m_node;
	_ASSERTE (expressionNode->IsType(dDAGExpressionNode::GetRttiType()));

	dDAGFunctionStatementSWITCH* const switchNode = new dDAGFunctionStatementSWITCH (m_allNodes, expressionNode, casetListNode);

_ASSERTE (0);
//	expressionNode->Release();
	for (dDAGFunctionStatement* nextNode = casetListNode; nextNode; nextNode = (dDAGFunctionStatement*) nextNode->m_next) {
		_ASSERTE (0);
//		_ASSERTE (nextNode->IsType(dDAGFunctionStatementCase::GetRttiType()));
//		nextNode->Release();
	}

	returnNode.m_node = switchNode;
	return returnNode;
}



void dScriptCompiler::OpenPackage (const dString& packageName)
{
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
}


