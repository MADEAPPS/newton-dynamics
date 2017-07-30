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


#ifndef __NewtonBinding_h__
#define __NewtonBinding_h__

#include "stdafx.h"
#include "newtonGrammar.h"

class NewtonBinding: public newtonGrammar
{
	public:
	NewtonBinding(const char* const outputPath);
	virtual ~NewtonBinding();

	virtual void Parse();

	virtual void BlockEnd ();

	virtual void EngineVersion (const string& mayor, const string& minor);
	virtual void DeclareDataType (const string& singlePrecision, const string& doublePrecision);

	virtual void ConstantDefinition (const string& constName, const string& value);
	virtual void InternalEngineStruct (const string& structName);

	virtual void StructDeclareStart (const string& structName);
	virtual void StructAddDataType (const string& dataType, const string& dataName);
	virtual void StructNameLessUnion ();
	virtual void StructDeclareEnd ();
	
	virtual void FunctionCallbackDeclaration (const string& returnType, const string& callbackName);
	virtual void FunctionDeclaration (const string& returnType, const string& callbackName);
	virtual void FunctionArgumentSeparator ();
	virtual void FunctionArgument (const string& argType, const string& name);
	virtual void FunctionDeclarationEnd ();

	protected:
	static char* m_licence;
	char* m_newtonHeader;
};

#endif
