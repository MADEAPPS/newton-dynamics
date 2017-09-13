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
// Auto generated Parser Generator class: dNewtonLuaParcer.h
//

#ifndef __dNewtonLuaParcer_h__
#define __dNewtonLuaParcer_h__

#include <dList.h>
#include <dString.h>

class dNewtonLuaLex;

class dNewtonLuaParcer
{
	public:
	enum dToken
	{
		ACCEPTING_TOKEN = 254, 
		ERROR_TOKEN = 255, 
		_AND = 256, 
		_BREAK,
		_DO,
		_ELSE,
		_ELSEIF,
		_END,
		_FALSE,
		_FOR,
		_FUNCTION,
		_GOTO,
		_IF,
		_IN,
		_LOCAL,
		_NIL,
		_NOT,
		_OR,
		_REPEAT,
		_RETURN,
		_THEN,
		_TRUE,
		_UNTIL,
		_WHILE,
		_LEFT_SHIFT,
		_RIGHT_SHIFT,
		_INTEGER_DIVIDE,
		_IDENTICAL,
		_DIFFERENT,
		_LEFT_EQUAL,
		_GREATHER_EQUAL,
		_DOUBLE_COLUMN,
		_DOUBLE_DOT,
		_TRIPLE_DOT,
		_INTEGER,
		_FLOAT,
		_LABEL,
		_STRING
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
		//friend dNewtonLuaParcer;
	};



	class dUserVariable: public dDefualtUserVariable
	{
		public:
		dUserVariable () 
			:dDefualtUserVariable ()
			,m_node(NULL)
		{
		}
		
		dUserVariable (dToken token, const char* const text, int scannerLine, int scannerIndex)
			:dDefualtUserVariable (token, text, scannerLine, scannerIndex)
			,m_node(NULL)
		{
		}
		dCIL::dListNode* m_node;
	};


	dNewtonLuaParcer();
	virtual ~dNewtonLuaParcer();
	virtual bool Parse(dNewtonLuaLex& scanner);

	private:
	const dGotoEntry* FindGoto (const dGotoEntry* const gotoList, int count, dToken token) const;
	const dActionEntry* FindAction (const dActionEntry* const list, int count, dToken token) const;
	const dActionEntry* GetNextAction (dList<dStackPair>& stack, dToken token, dNewtonLuaLex& scanner) const;

	bool m_grammarError;
};

#endif
