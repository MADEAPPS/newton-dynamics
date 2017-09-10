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

//
// Auto generated Parser Generator class: dLittleScriptParser.h
//

#ifndef __dLittleScriptParser_h__
#define __dLittleScriptParser_h__

#ifdef _MSC_VER
#pragma warning (disable: 4702) // warning C4702: unreachable code
#pragma warning (disable: 4100) // warning C4100: unreferenced formal parameter
#pragma warning (disable: 4201) // warning C4201: nonstandard extension used : nameless struct/union
#endif


#include <dList.h>
#include <dString.h>

class dLittleScriptLexical;

class dLittleScriptParser
{
	public:
	enum dToken
	{
		ACCEPTING_TOKEN = 254, 
		ERROR_TOKEN = 255, 
		_BYTE = 256, 
		_INT,
		_SHORT,
		_LONG,
		_BOOLEAN,
		_FLOAT,
		_DOUBLE,
		_CONST,
		_VOID,
		_CLASS,
		_IMPORT,
		_FINAL,
		_PUBLIC,
		_PRIVATE,
		_STATIC,
		_NATIVE,
		_PACKAGE,
		_INTERFACE,
		_IDENTIFIER,
		_OP_DIM,
		_IF,
		_ELSE,
		_SWITCH,
		_CASE,
		_DEFAULT,
		_BREAK,
		_CONTINUE,
		_DO,
		_FOR,
		_WHILE,
		_RETURN,
		_IDENTICAL,
		_DIFFERENT,
		_LESS_EQUAL,
		_GREATHER_EQUAL,
		_SHIFT_RIGHT,
		_SHIFT_LEFT,
		_LOGIC_OR,
		_LOGIC_AND,
		_NEW,
		_THIS,
		_FLOAT_CONST,
		_INTEGER_CONST,
		_OP_INC,
		_OP_DEC,
		_ASS_MUL,
		_ASS_DIV,
		_ASS_MOD,
		_ASS_ADD,
		_ASS_SUB,
		_ASS_SHL,
		_ASS_SHR,
		_ASS_AND,
		_ASS_XOR,
		_ASS_OR
	};

	enum ActionType;
	class dStackPair;
	class dGotoEntry;
	class dActionEntry;
	class dDefualtUserVariable
	{
		public:
		dDefualtUserVariable () 
			:m_scannerLine (0), m_scannerIndex(0), m_token (dToken (0)), m_data("")
		{
		}

		dDefualtUserVariable (dToken token, const char* const data, int scannerLine, int scannerIndex)
			:m_scannerLine (scannerLine), m_scannerIndex(scannerIndex), m_token(token), m_data (data) 
		{
		}

		dToken GetToken() const 
		{
			return m_token;
		}

		const dString& GetString() const 
		{
			return m_data;
		}

		//protected:
		int m_scannerLine;
		int m_scannerIndex;
		dToken m_token;
		dString m_data;
		//friend dLittleScriptParser;
	};



	class dUserVariable: public dDefualtUserVariable
	{
		public:
		dUserVariable () 
			:dDefualtUserVariable (), m_node(NULL)
		{
		}
		
		dUserVariable (dToken token, const char* const text, int scannerLine, int scannerIndex)
			:dDefualtUserVariable (token, text, scannerLine, scannerIndex)
			,m_node(NULL)
		{
		}
		class dDAG* m_node;
	};


	dLittleScriptParser();
	virtual ~dLittleScriptParser();
	virtual bool Parse(dLittleScriptLexical& scanner);

	private:
	const dGotoEntry* FindGoto (const dGotoEntry* const gotoList, int count, dToken token) const;
	const dActionEntry* FindAction (const dActionEntry* const list, int count, dToken token) const;
	const dActionEntry* GetNextAction (dList<dStackPair>& stack, dToken token, dLittleScriptLexical& scanner) const;

	bool m_grammarError;
};

#endif
