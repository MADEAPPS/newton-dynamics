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
// Auto generated Parser Generator class: dNewtonLuaParcer.cpp
//

// Newton Tool embedded Lua script Language
// based of https://www.lua.org/manual/5.3/manual.html#9
//

#include "dNewtonLuaStdafx.h"
#include "dNewtonLuaLex.h"
#include "dNewtonLuaParcer.h"
#include "dNewtonLuaCompiler.h"

	#define MyModule ((dNewtonLuaCompiler*) this)


#include "dNewtonLuaParcer.h"
#include <dList.h>

#define MAX_USER_PARAM	64

enum dNewtonLuaParcer::ActionType
{
	dSHIFT = 0,
	dREDUCE,
	dACCEPT,
	dERROR
};

class dNewtonLuaParcer::dActionEntry
{
	public:
	dActionEntry (short token, char errorItem, char stateType, short nextState, short ruleSymbols, short ruleIndex)
		:m_token(token), m_errorRule(errorItem), m_stateType (stateType), m_nextState(nextState), m_ruleSymbols(ruleSymbols), m_ruleIndex(ruleIndex)
	{
	}

	short m_token;
	char m_errorRule;
	char m_stateType;// 0 = shift, 1 = reduce, 2 = accept
	short m_nextState;
	short m_ruleSymbols;
	short m_ruleIndex;
};

class dNewtonLuaParcer::dGotoEntry
{
	public:
	dGotoEntry (short token, short nextState)
		:m_token(token), m_nextState(nextState)
	{
	}

	short  m_token;
	short  m_nextState;
};



class dNewtonLuaParcer::dStackPair
{
	public:
	dStackPair()
		:m_state(0), m_scannerLine(0), m_scannerIndex(0), m_token(dToken (0)), m_value()
	{
	}

	int m_state;
	int m_scannerLine;
	int m_scannerIndex;
	dToken m_token;
	dUserVariable m_value;
};


dNewtonLuaParcer::dNewtonLuaParcer()
{
}

dNewtonLuaParcer::~dNewtonLuaParcer()
{
}


const dNewtonLuaParcer::dActionEntry* dNewtonLuaParcer::FindAction (const dActionEntry* const actionList, int count, dToken token) const
{
	int i0 = 0;
	int i1 = count - 1;
	while ((i1 - i0) >= 4) {
		int i = (i1 + i0 + 1)>>1;

		const dActionEntry& action = actionList[i];
		dToken actionToken (dToken(action.m_token));
		if (token <= actionToken) {
			i1 = i;
		} else {
			i0 = i;
		}
	}

	for (int i = i0; i <= i1; i ++) {
		const dActionEntry& action = actionList[i];
		dToken actionToken (dToken(action.m_token));
		if (token == actionToken) {
			return& action;
		}
	}


	return NULL;
}

const dNewtonLuaParcer::dGotoEntry* dNewtonLuaParcer::FindGoto (const dGotoEntry* const gotoList, int count, dToken token) const
{
	int i0 = 0;
	int i1 = count - 1;
	while ((i1 - i0) >= 4) {
		int i = (i1 + i0 + 1)>>1;

		const dGotoEntry& action = gotoList[i];
		dToken actionToken (dToken(action.m_token));
		if (token <= actionToken) {
			i1 = i;
		} else {
			i0 = i;
		}
	}

	for (int i = i0; i <= i1; i ++) {
		const dGotoEntry& action = gotoList[i];
		dToken actionToken (dToken(action.m_token));
		if (token == actionToken) {
			return &action;
		}
	}

	dAssert (0);
	return NULL;
}



const dNewtonLuaParcer::dActionEntry* dNewtonLuaParcer::GetNextAction (dList<dStackPair>& stack, dToken token, dNewtonLuaLex& scanner) const
{
	static short actionsCount[] = {
			8, 8, 9, 9, 9, 2, 9, 9, 9, 9, 1, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 
			8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 8, 9, 9, 9, 9, 9, 9, 9, 8, 9, 9, 
			9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 
			9, 9, 9, 9, 9, 9};
	static short actionsStart[] = {
			0, 8, 16, 25, 34, 43, 45, 54, 63, 72, 81, 82, 8, 91, 99, 107, 115, 123, 131, 139, 147, 0, 0, 0, 
			0, 0, 0, 0, 155, 163, 8, 8, 8, 8, 8, 8, 171, 8, 180, 189, 198, 207, 216, 225, 234, 8, 16, 25, 
			243, 45, 54, 63, 72, 82, 252, 260, 268, 276, 284, 292, 300, 308, 316, 155, 155, 155, 155, 155, 155, 155, 171, 180, 
			189, 324, 333, 342, 225, 351};
	static dActionEntry actionTable[] = {
			dActionEntry (40, 0, 0, 1, 0, 0), dActionEntry (262, 0, 0, 3, 0, 0), dActionEntry (269, 0, 0, 6, 0, 0), dActionEntry (275, 0, 0, 2, 0, 0), 
			dActionEntry (288, 0, 0, 8, 0, 0), dActionEntry (289, 0, 0, 11, 0, 0), dActionEntry (290, 0, 0, 9, 0, 0), dActionEntry (291, 0, 0, 7, 0, 0), 
			dActionEntry (40, 0, 0, 12, 0, 0), dActionEntry (262, 0, 0, 14, 0, 0), dActionEntry (269, 0, 0, 16, 0, 0), dActionEntry (275, 0, 0, 13, 0, 0), 
			dActionEntry (288, 0, 0, 18, 0, 0), dActionEntry (289, 0, 0, 20, 0, 0), dActionEntry (290, 0, 0, 19, 0, 0), dActionEntry (291, 0, 0, 17, 0, 0), 
			dActionEntry (42, 0, 1, 0, 1, 10), dActionEntry (43, 0, 1, 0, 1, 10), dActionEntry (44, 0, 1, 0, 1, 10), dActionEntry (45, 0, 1, 0, 1, 10), 
			dActionEntry (47, 0, 1, 0, 1, 10), dActionEntry (254, 0, 1, 0, 1, 10), dActionEntry (271, 0, 1, 0, 1, 10), dActionEntry (280, 0, 1, 0, 1, 10), 
			dActionEntry (281, 0, 1, 0, 1, 10), dActionEntry (42, 0, 1, 0, 1, 11), dActionEntry (43, 0, 1, 0, 1, 11), dActionEntry (44, 0, 1, 0, 1, 11), 
			dActionEntry (45, 0, 1, 0, 1, 11), dActionEntry (47, 0, 1, 0, 1, 11), dActionEntry (254, 0, 1, 0, 1, 11), dActionEntry (271, 0, 1, 0, 1, 11), 
			dActionEntry (280, 0, 1, 0, 1, 11), dActionEntry (281, 0, 1, 0, 1, 11), dActionEntry (42, 0, 0, 22, 0, 0), dActionEntry (43, 0, 0, 23, 0, 0), 
			dActionEntry (44, 0, 1, 1, 1, 16), dActionEntry (45, 0, 0, 25, 0, 0), dActionEntry (47, 0, 0, 21, 0, 0), dActionEntry (254, 0, 1, 1, 1, 16), 
			dActionEntry (271, 0, 0, 24, 0, 0), dActionEntry (280, 0, 0, 26, 0, 0), dActionEntry (281, 0, 0, 27, 0, 0), dActionEntry (44, 0, 0, 28, 0, 0), 
			dActionEntry (254, 0, 1, 2, 1, 18), dActionEntry (42, 0, 1, 0, 1, 9), dActionEntry (43, 0, 1, 0, 1, 9), dActionEntry (44, 0, 1, 0, 1, 9), 
			dActionEntry (45, 0, 1, 0, 1, 9), dActionEntry (47, 0, 1, 0, 1, 9), dActionEntry (254, 0, 1, 0, 1, 9), dActionEntry (271, 0, 1, 0, 1, 9), 
			dActionEntry (280, 0, 1, 0, 1, 9), dActionEntry (281, 0, 1, 0, 1, 9), dActionEntry (42, 0, 1, 0, 1, 14), dActionEntry (43, 0, 1, 0, 1, 14), 
			dActionEntry (44, 0, 1, 0, 1, 14), dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), dActionEntry (254, 0, 1, 0, 1, 14), 
			dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (280, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (42, 0, 1, 0, 1, 15), 
			dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (44, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), 
			dActionEntry (254, 0, 1, 0, 1, 15), dActionEntry (271, 0, 1, 0, 1, 15), dActionEntry (280, 0, 1, 0, 1, 15), dActionEntry (281, 0, 1, 0, 1, 15), 
			dActionEntry (42, 0, 1, 0, 1, 13), dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (44, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), 
			dActionEntry (47, 0, 1, 0, 1, 13), dActionEntry (254, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (280, 0, 1, 0, 1, 13), 
			dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (254, 0, 2, 0, 0, 0), dActionEntry (42, 0, 1, 0, 1, 12), dActionEntry (43, 0, 1, 0, 1, 12), 
			dActionEntry (44, 0, 1, 0, 1, 12), dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), dActionEntry (254, 0, 1, 0, 1, 12), 
			dActionEntry (271, 0, 1, 0, 1, 12), dActionEntry (280, 0, 1, 0, 1, 12), dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (41, 0, 1, 0, 1, 10), 
			dActionEntry (42, 0, 1, 0, 1, 10), dActionEntry (43, 0, 1, 0, 1, 10), dActionEntry (45, 0, 1, 0, 1, 10), dActionEntry (47, 0, 1, 0, 1, 10), 
			dActionEntry (271, 0, 1, 0, 1, 10), dActionEntry (280, 0, 1, 0, 1, 10), dActionEntry (281, 0, 1, 0, 1, 10), dActionEntry (41, 0, 1, 0, 1, 11), 
			dActionEntry (42, 0, 1, 0, 1, 11), dActionEntry (43, 0, 1, 0, 1, 11), dActionEntry (45, 0, 1, 0, 1, 11), dActionEntry (47, 0, 1, 0, 1, 11), 
			dActionEntry (271, 0, 1, 0, 1, 11), dActionEntry (280, 0, 1, 0, 1, 11), dActionEntry (281, 0, 1, 0, 1, 11), dActionEntry (41, 0, 0, 36, 0, 0), 
			dActionEntry (42, 0, 0, 31, 0, 0), dActionEntry (43, 0, 0, 32, 0, 0), dActionEntry (45, 0, 0, 34, 0, 0), dActionEntry (47, 0, 0, 30, 0, 0), 
			dActionEntry (271, 0, 0, 33, 0, 0), dActionEntry (280, 0, 0, 35, 0, 0), dActionEntry (281, 0, 0, 37, 0, 0), dActionEntry (41, 0, 1, 0, 1, 9), 
			dActionEntry (42, 0, 1, 0, 1, 9), dActionEntry (43, 0, 1, 0, 1, 9), dActionEntry (45, 0, 1, 0, 1, 9), dActionEntry (47, 0, 1, 0, 1, 9), 
			dActionEntry (271, 0, 1, 0, 1, 9), dActionEntry (280, 0, 1, 0, 1, 9), dActionEntry (281, 0, 1, 0, 1, 9), dActionEntry (41, 0, 1, 0, 1, 14), 
			dActionEntry (42, 0, 1, 0, 1, 14), dActionEntry (43, 0, 1, 0, 1, 14), dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), 
			dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (280, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (41, 0, 1, 0, 1, 15), 
			dActionEntry (42, 0, 1, 0, 1, 15), dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), 
			dActionEntry (271, 0, 1, 0, 1, 15), dActionEntry (280, 0, 1, 0, 1, 15), dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (41, 0, 1, 0, 1, 13), 
			dActionEntry (42, 0, 1, 0, 1, 13), dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), 
			dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (280, 0, 1, 0, 1, 13), dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (41, 0, 1, 0, 1, 12), 
			dActionEntry (42, 0, 1, 0, 1, 12), dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), 
			dActionEntry (271, 0, 1, 0, 1, 12), dActionEntry (280, 0, 1, 0, 1, 12), dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (40, 0, 0, 45, 0, 0), 
			dActionEntry (262, 0, 0, 47, 0, 0), dActionEntry (269, 0, 0, 49, 0, 0), dActionEntry (275, 0, 0, 46, 0, 0), dActionEntry (288, 0, 0, 51, 0, 0), 
			dActionEntry (289, 0, 0, 53, 0, 0), dActionEntry (290, 0, 0, 52, 0, 0), dActionEntry (291, 0, 0, 50, 0, 0), dActionEntry (41, 0, 0, 54, 0, 0), 
			dActionEntry (42, 0, 0, 31, 0, 0), dActionEntry (43, 0, 0, 32, 0, 0), dActionEntry (45, 0, 0, 34, 0, 0), dActionEntry (47, 0, 0, 30, 0, 0), 
			dActionEntry (271, 0, 0, 33, 0, 0), dActionEntry (280, 0, 0, 35, 0, 0), dActionEntry (281, 0, 0, 37, 0, 0), dActionEntry (42, 0, 1, 0, 3, 8), 
			dActionEntry (43, 0, 1, 0, 3, 8), dActionEntry (44, 0, 1, 0, 3, 8), dActionEntry (45, 0, 1, 0, 3, 8), dActionEntry (47, 0, 1, 0, 3, 8), 
			dActionEntry (254, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (280, 0, 1, 0, 3, 8), dActionEntry (281, 0, 1, 0, 3, 8), 
			dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (44, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), 
			dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (254, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (280, 0, 1, 0, 3, 6), 
			dActionEntry (281, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (44, 0, 1, 0, 3, 5), 
			dActionEntry (45, 0, 1, 0, 3, 5), dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), 
			dActionEntry (280, 0, 1, 0, 3, 5), dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (42, 0, 0, 22, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), 
			dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 0, 21, 0, 0), dActionEntry (254, 0, 1, 0, 3, 3), 
			dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 26, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 22, 0, 0), 
			dActionEntry (43, 0, 0, 23, 0, 0), dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 0, 25, 0, 0), dActionEntry (47, 0, 0, 21, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 26, 0, 0), dActionEntry (281, 0, 0, 27, 0, 0), 
			dActionEntry (42, 0, 0, 22, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), 
			dActionEntry (47, 0, 0, 21, 0, 0), dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 26, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (42, 0, 1, 0, 3, 7), dActionEntry (43, 0, 1, 0, 3, 7), dActionEntry (44, 0, 1, 0, 3, 7), 
			dActionEntry (45, 0, 1, 0, 3, 7), dActionEntry (47, 0, 1, 0, 3, 7), dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), 
			dActionEntry (280, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (42, 0, 0, 22, 0, 0), dActionEntry (43, 0, 0, 23, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 0, 25, 0, 0), dActionEntry (47, 0, 0, 21, 0, 0), dActionEntry (254, 0, 1, 0, 3, 2), 
			dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 26, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 64, 0, 0), 
			dActionEntry (43, 0, 0, 65, 0, 0), dActionEntry (44, 0, 1, 1, 3, 17), dActionEntry (45, 0, 0, 67, 0, 0), dActionEntry (47, 0, 0, 63, 0, 0), 
			dActionEntry (254, 0, 1, 1, 3, 17), dActionEntry (271, 0, 0, 66, 0, 0), dActionEntry (280, 0, 0, 68, 0, 0), dActionEntry (281, 0, 0, 69, 0, 0), 
			dActionEntry (41, 0, 1, 0, 3, 8), dActionEntry (42, 0, 1, 0, 3, 8), dActionEntry (43, 0, 1, 0, 3, 8), dActionEntry (45, 0, 1, 0, 3, 8), 
			dActionEntry (47, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (280, 0, 1, 0, 3, 8), dActionEntry (281, 0, 1, 0, 3, 8), 
			dActionEntry (41, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), 
			dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (280, 0, 1, 0, 3, 6), dActionEntry (281, 0, 1, 0, 3, 6), 
			dActionEntry (41, 0, 1, 0, 3, 5), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), 
			dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (280, 0, 1, 0, 3, 5), dActionEntry (281, 0, 1, 0, 3, 5), 
			dActionEntry (41, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 31, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 0, 30, 0, 0), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 35, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), 
			dActionEntry (41, 0, 1, 0, 3, 1), dActionEntry (42, 0, 0, 31, 0, 0), dActionEntry (43, 0, 0, 32, 0, 0), dActionEntry (45, 0, 0, 34, 0, 0), 
			dActionEntry (47, 0, 0, 30, 0, 0), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 35, 0, 0), dActionEntry (281, 0, 0, 37, 0, 0), 
			dActionEntry (41, 0, 1, 0, 3, 4), dActionEntry (42, 0, 0, 31, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), 
			dActionEntry (47, 0, 0, 30, 0, 0), dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 35, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), 
			dActionEntry (41, 0, 1, 0, 3, 7), dActionEntry (42, 0, 1, 0, 3, 7), dActionEntry (43, 0, 1, 0, 3, 7), dActionEntry (45, 0, 1, 0, 3, 7), 
			dActionEntry (47, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (280, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), 
			dActionEntry (41, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 31, 0, 0), dActionEntry (43, 0, 0, 32, 0, 0), dActionEntry (45, 0, 0, 34, 0, 0), 
			dActionEntry (47, 0, 0, 30, 0, 0), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 35, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2), 
			dActionEntry (41, 0, 0, 70, 0, 0), dActionEntry (42, 0, 0, 31, 0, 0), dActionEntry (43, 0, 0, 32, 0, 0), dActionEntry (45, 0, 0, 34, 0, 0), 
			dActionEntry (47, 0, 0, 30, 0, 0), dActionEntry (271, 0, 0, 33, 0, 0), dActionEntry (280, 0, 0, 35, 0, 0), dActionEntry (281, 0, 0, 37, 0, 0), 
			dActionEntry (42, 0, 0, 64, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 0, 63, 0, 0), dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 68, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 64, 0, 0), dActionEntry (43, 0, 0, 65, 0, 0), dActionEntry (44, 0, 1, 0, 3, 1), 
			dActionEntry (45, 0, 0, 67, 0, 0), dActionEntry (47, 0, 0, 63, 0, 0), dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), 
			dActionEntry (280, 0, 0, 68, 0, 0), dActionEntry (281, 0, 0, 69, 0, 0), dActionEntry (42, 0, 0, 64, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), 
			dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 0, 63, 0, 0), dActionEntry (254, 0, 1, 0, 3, 4), 
			dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 68, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (42, 0, 0, 64, 0, 0), 
			dActionEntry (43, 0, 0, 65, 0, 0), dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 0, 67, 0, 0), dActionEntry (47, 0, 0, 63, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 68, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2)};

	bool errorMode = false;
	const dStackPair& stackTop = stack.GetLast()->GetInfo();
	int state = stackTop.m_state;
	int start = actionsStart[state];
	int count = actionsCount[state];

	const dActionEntry* const table = &actionTable[start];
	const dActionEntry* action = FindAction (table, count, token);
	while (!action && (stack.GetCount() > 1)) {
		errorMode = true; 

		// we found a syntax error, go into error recovering mode and find the token mark by a ". error" rule
		stack.Remove (stack.GetLast());

		const dStackPair& stackTop = stack.GetLast()->GetInfo();
		int state = stackTop.m_state;
		int start = actionsStart[state];
		int count = actionsCount[state];
		const dActionEntry* const table = &actionTable[start];
		action = FindAction (table, count, ERROR_TOKEN);
		if (action && !action->m_errorRule) {
			action = NULL;
		}
	}

	if (errorMode && action) {
		dStackPair& stackTop = stack.GetLast()->GetInfo();
		stackTop.m_token = ERROR_TOKEN;

		int state = action->m_nextState;
		int start = actionsStart[state];
		int count = actionsCount[state];
		const dActionEntry* const table = &actionTable[start];

		// find the next viable token to continues parsing
		while (!FindAction (table, count, token)) {
			token = dToken (scanner.NextToken());
			if (token == -1) {
				// reached end of the file, can not recover from this error;
				return NULL;
			}
		}
		action = FindAction (table, count, token);
		
		dStackPair& entry = stack.Append()->GetInfo();
		entry.m_state = state;
		entry.m_scannerLine = stackTop.m_scannerLine;
		entry.m_scannerIndex = stackTop.m_scannerIndex;
		entry.m_value = dUserVariable (ERROR_TOKEN, "error", entry.m_scannerLine, entry.m_scannerIndex);
		entry.m_token = token;
	}

	return action;
}


bool dNewtonLuaParcer::Parse(dNewtonLuaLex& scanner)
{
	static short gotoCount[] = {
			3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 
			1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 
			0, 0, 0, 0, 0, 0};
	static short gotoStart[] = {
			0, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 7, 
			8, 9, 10, 11, 12, 13, 13, 14, 15, 16, 17, 18, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 
			21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 23, 24, 25, 26, 27, 28, 28, 
			28, 28, 28, 28, 28, 28};
	static dGotoEntry gotoTable[] = {
			dGotoEntry (292, 4), dGotoEntry (293, 5), dGotoEntry (294, 10), dGotoEntry (292, 15), dGotoEntry (292, 29), 
			dGotoEntry (292, 38), dGotoEntry (292, 39), dGotoEntry (292, 40), dGotoEntry (292, 41), dGotoEntry (292, 42), 
			dGotoEntry (292, 43), dGotoEntry (292, 44), dGotoEntry (292, 48), dGotoEntry (292, 55), dGotoEntry (292, 56), 
			dGotoEntry (292, 57), dGotoEntry (292, 58), dGotoEntry (292, 59), dGotoEntry (292, 60), dGotoEntry (292, 61), 
			dGotoEntry (292, 62), dGotoEntry (292, 71), dGotoEntry (292, 72), dGotoEntry (292, 73), dGotoEntry (292, 74), 
			dGotoEntry (292, 75), dGotoEntry (292, 76), dGotoEntry (292, 77)};

	dList<dStackPair> stack;
	const int lastToken = 292;
	
	stack.Append ();
	m_grammarError = false;
	dToken token = dToken (scanner.NextToken());

	#ifdef D_DEBUG_PARCEL
	int oldLine = scanner.GetLineNumber();
	#endif

	for (bool terminate = false; !terminate;) {

		const dActionEntry* const action = GetNextAction (stack, token, scanner);
		if (!action) {
			terminate = true;
			fprintf (stderr, "unrecoverable parser error\n");
			dTrace (("unrecoverable parser error\n"));
		} else {
			switch (action->m_stateType) 
			{
				case dSHIFT: 
				{
					dStackPair& entry = stack.Append()->GetInfo();
					entry.m_state = action->m_nextState;
					entry.m_scannerLine = scanner.GetLineNumber();
					entry.m_scannerIndex = scanner.GetIndex();
					entry.m_value = dUserVariable (token, scanner.GetTokenString(), entry.m_scannerLine, entry.m_scannerIndex);

					#ifdef D_DEBUG_PARCEL
					if (scanner.GetLineNumber() != oldLine) {
						oldLine = scanner.GetLineNumber();
						dTrace (("\n"));
					}
					dTrace (("%s ", scanner.GetTokenString()));
					#endif
					
					token = dToken (scanner.NextToken());

					entry.m_token = token;
					if (token == -1) {
						token = ACCEPTING_TOKEN;
					}

					break;
				}

				case dREDUCE: 
				{
					dStackPair parameter[MAX_USER_PARAM];

					int reduceCount = action->m_ruleSymbols;
					dAssert (reduceCount < sizeof (parameter) / sizeof (parameter[0]));

					for (int i = 0; i < reduceCount; i ++) {
						parameter[reduceCount - i - 1] = stack.GetLast()->GetInfo();
						stack.Remove (stack.GetLast());
					}

					const dStackPair& stackTop = stack.GetLast()->GetInfo();
					int start = gotoStart[stackTop.m_state];
					int count = gotoCount[stackTop.m_state];
					const dGotoEntry* const table = &gotoTable[start];
					const dGotoEntry* const gotoEntry = FindGoto (table, count, dToken (action->m_nextState + lastToken));

					dStackPair& entry = stack.Append()->GetInfo();
					entry.m_state = gotoEntry->m_nextState;
					entry.m_scannerLine = scanner.GetLineNumber();
					entry.m_scannerIndex = scanner.GetIndex();
					entry.m_token = dToken (gotoEntry->m_token);
					
					switch (action->m_ruleIndex) 
					{
						//do user semantic Actions
						case 10:// expression : _TRUE 
{dAssert(0);}
break;

						case 11:// expression : _FALSE 
{dAssert(0);}
break;

						case 16:// expressionList : expression 
{entry.m_value = parameter[0].m_value;}
break;

						case 9:// expression : _NIL 
{dAssert(0);}
break;

						case 14:// expression : _STRING 
{dAssert(0);}
break;

						case 15:// expression : _INTEGER 
{entry.m_value = MyModule->EmitLoadConstant(parameter[0].m_value);}
break;

						case 13:// expression : _LABEL 
{entry.m_value = MyModule->EmitLoadVariable(parameter[0].m_value);}
break;

						case 12:// expression : _FLOAT 
{dAssert(0);}
break;

						case 8:// expression : ( expression ) 
{dAssert(0);}
break;

						case 6:// expression : expression / expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 5:// expression : expression * expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 3:// expression : expression + expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 1:// expression : expression _OR expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 4:// expression : expression - expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 7:// expression : expression _INTEGER_DIVIDE expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 2:// expression : expression _IDENTICAL expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 17:// expressionList : expressionList , expression 
{entry.m_value = MyModule->LinkExpresion(parameter[0].m_value, parameter[2].m_value);}
break;


						default:;
					}
					break;
				}
		
				case dACCEPT: // 2 = accept
				{
					// program parsed successfully, exit with successful code
					terminate = true;
					break;
				}
				
				default:  
				{
					dAssert (0);
					// syntax error parsing program
					//if (!ErrorHandler ("error")) {
					//}
					terminate = true;
					m_grammarError = true;
					break;
				}
			}
		}
	}
	return !m_grammarError;
}



