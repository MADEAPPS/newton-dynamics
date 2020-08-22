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
// Auto generated Parser Generator class: ansi_c_parcel.h
//

#ifndef __ansi_c_parcel_h__
#define __ansi_c_parcel_h__

#include <dList.h>
#include <dString.h>

class ansi_c_lex;

class ansi_c_parcel
{
	public:
	enum dToken
	{
		ACCEPTING_TOKEN = 254, 
		ERROR_TOKEN = 255, 
		IDENTIFIER = 256, 
		CONSTANT,
		STRING_LITERAL,
		SIZEOF,
		PTR_OP,
		INC_OP,
		DEC_OP,
		LEFT_OP,
		RIGHT_OP,
		LE_OP,
		GE_OP,
		EQ_OP,
		NE_OP,
		AND_OP,
		OR_OP,
		MUL_ASSIGN,
		DIV_ASSIGN,
		MOD_ASSIGN,
		ADD_ASSIGN,
		SUB_ASSIGN,
		LEFT_ASSIGN,
		RIGHT_ASSIGN,
		AND_ASSIGN,
		XOR_ASSIGN,
		OR_ASSIGN,
		TYPE_NAME,
		TYPEDEF,
		EXTERN,
		STATIC,
		AUTO,
		REGISTER,
		CHAR,
		SHORT,
		INT,
		LONG,
		SIGNED,
		UNSIGNED,
		FLOAT,
		DOUBLE,
		CONST,
		VOLATILE,
		VOID,
		STRUCT,
		UNION,
		ENUM,
		ELLIPSIS,
		CASE,
		DEFAULT,
		IF,
		ELSE,
		SWITCH,
		WHILE,
		DO,
		FOR,
		GOTO,
		CONTINUE,
		BREAK,
		RETURN
	};

	enum ActionType;
	class dStackPair;
	class dGotoEntry;
	class dActionEntry;
	class dDefualtUserVariable
	{
		public:
		dDefualtUserVariable () 
			:m_scannerLine (0), m_scannerIndex(0), m_token (dToken (0)), m_data()
		{
		}

		dDefualtUserVariable (const dDefualtUserVariable& copy) 
			:m_scannerLine(copy.m_scannerLine), m_scannerIndex(copy.m_scannerIndex), m_token(copy.m_token), m_data(copy.m_data)
		{
		}

		dDefualtUserVariable (dToken token, const char* const data, int scannerLine, int scannerIndex)
			:m_scannerLine (scannerLine), m_scannerIndex(scannerIndex), m_token(token), m_data (data) 
		{
		}

		dDefualtUserVariable& operator= (const dDefualtUserVariable& src)
		{
			m_scannerLine = src.m_scannerLine;  
			m_scannerIndex = src.m_scannerIndex;
			m_token = src.m_token;
			m_data = src.m_data;
			return *this;
		}

		dToken GetToken() const 
		{
			return m_token;
		}

		const dString& GetString() const 
		{
			return m_data;
		}

		int m_scannerLine;
		int m_scannerIndex;
		dToken m_token;
		dString m_data;
	};


	class dUserVariable: public dDefualtUserVariable
	{
		public:
		dUserVariable () 
			:dDefualtUserVariable ()
		{
		}

		dUserVariable (const dUserVariable& copy) 
			:dDefualtUserVariable(copy)
		{
		}

		dUserVariable& operator= (const dUserVariable& src)
		{
			dDefualtUserVariable& me = *this;
			me = src;
			return *this;
		}
		dUserVariable (dToken token, const char* const data, int scannerLine, int scannerIndex)
			:dDefualtUserVariable  (token, data, scannerLine, scannerIndex)
		{
		}
	};



	ansi_c_parcel();
	virtual ~ansi_c_parcel();
	virtual bool Parse(ansi_c_lex& scanner);

	private:
	const dGotoEntry* FindGoto (const dGotoEntry* const gotoList, int count, dToken token) const;
	const dActionEntry* FindAction (const dActionEntry* const list, int count, dToken token) const;
	const dActionEntry* GetNextAction (dList<dStackPair>& stack, dToken token, ansi_c_lex& scanner) const;

	bool m_grammarError;
};

#endif
