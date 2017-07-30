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
//Auto generated Parser Generator class: NewtonBinding.cpp
//


#include <stdafx.h>
#include "lexical.h"
#include "newtonBinding.h"

#define newtonHeaderPath	"../../../coreLibrary_300/source/newton/newton.h"

char* NewtonBinding::m_licence =	"/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>\n"
									"* \n"
									"* This software is provided 'as-is', without any express or implied\n"
									"* warranty. In no event will the authors be held liable for any damages\n"
									"* arising from the use of this software.\n"
									"* \n"
									"* Permission is granted to anyone to use this software for any purpose,\n"
									"* including commercial applications, and to alter it and redistribute it\n"
									"* freely, subject to the following restrictions:\n"
									"* \n"
									"* 1. The origin of this software must not be misrepresented; you must not\n"
									"* claim that you wrote the original software. If you use this software\n"
									"* in a product, an acknowledgment in the product documentation would be\n"
									"* appreciated but is not required.\n"
									"* \n"
									"* 2. Altered source versions must be plainly marked as such, and must not be\n"
									"* misrepresented as being the original software.\n"
									"* \n"
									"* 3. This notice may not be removed or altered from any source distribution.\n"
									"*/\n\n";


NewtonBinding::NewtonBinding(const char* const outputPath)
	:newtonGrammar()
{
	FILE* const inputFile = fopen (newtonHeaderPath, "rb");
	_ASSERTE (inputFile);

	fseek (inputFile, 0, SEEK_END);
	int size = ftell (inputFile);
	fseek (inputFile, 0, SEEK_SET);

	m_newtonHeader = new char [size + 1];
	memset (m_newtonHeader, 0, size + 1);
	fread (m_newtonHeader, 1, size, inputFile);
	fclose (inputFile);
}

NewtonBinding::~NewtonBinding()
{
	delete[] m_newtonHeader;
}

void NewtonBinding::Parse()
{
	lexical scanner(m_newtonHeader);
	newtonGrammar::Parse (scanner);
}


void NewtonBinding::BlockEnd ()
{
	DTRACE (("\n"));
}

void NewtonBinding::EngineVersion (const string& mayor, const string& minor)
{
//	#define NEWTON_MAJOR_VERSION 3 
//	#define NEWTON_MINOR_VERSION 02 
	DTRACE (("%s\n", m_licence));
	DTRACE (("NEWTON_MAJOR_VERSION %s\n", mayor.c_str()));
	DTRACE (("NEWTON_MINOR_VERSION %s\n", minor.c_str()));
}


void NewtonBinding::DeclareDataType  (const string& singlePrecision, const string& doublePrecision)
{
	//#ifndef dFloat
	//	#define dFloat float
	//#endif
	//#ifndef dFloat64
	//	#define dFloat64 double
	//#endif
	DTRACE (("#define dFloat %s\n", singlePrecision.c_str()));
	DTRACE (("#define dFloat64 %s\n", doublePrecision.c_str()));
}


void NewtonBinding::ConstantDefinition (const string& constName, const string& value)
{
	//#define NEWTON_PROFILER_WORLD_UPDATE	0
	DTRACE (("#define %s %s\n", constName.c_str(), value.c_str()));
}


void NewtonBinding::InternalEngineStruct (const string& structName)
{
	// typedef struct NewtonMesh{} NewtonMesh;
	DTRACE (("#typedef struct %s{} %s;\n", structName.c_str(), structName.c_str()));
}


void NewtonBinding::StructDeclareStart (const string& structName)
{
	//struct NewtonBoxParam {
	DTRACE (("struct %s {\n", structName.c_str()));
}

void NewtonBinding::StructAddDataType (const string& dataType, const string& dataName)
{
	//dFloat m_x;
	DTRACE (("\t%s %s;\n", dataType.c_str(), dataName.c_str()));
}

void NewtonBinding::StructNameLessUnion ()
{
	//union {
	DTRACE (("\tunion {\n"));
}


void NewtonBinding::StructDeclareEnd ()
{
	// };
	DTRACE (("};\n"));
}


void NewtonBinding::FunctionDeclarationEnd ()
{
	// );
	DTRACE ((");\n"));
}


void NewtonBinding::FunctionArgumentSeparator ()
{
	// ,
	DTRACE ((", "));
}

void NewtonBinding::FunctionArgument (const string& argType, const string& name)
{
	// int sizeInBytes
	DTRACE (("%s %s", argType.c_str(), name.c_str()));
}

void NewtonBinding::FunctionCallbackDeclaration (const string& returnType, const string& callbackName)
{
	//typedef void* (*NewtonAllocMemory) (
	DTRACE (("typedef %s (*%s) (", returnType.c_str(), callbackName.c_str()));
}


void NewtonBinding::FunctionDeclaration (const string& returnType, const string& functionName)
{
	//NEWTON_API int NewtonWorldGetVersion
	DTRACE (("%s %s (", returnType.c_str(), functionName.c_str()));
}