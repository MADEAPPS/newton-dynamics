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
			2, 2, 1, 2, 4, 4, 4, 4, 2, 1, 8, 1, 2, 1, 4, 10, 8, 12, 12, 12, 5, 12, 12, 12, 
			12, 12, 2, 8, 10, 10, 10, 3, 10, 1, 10, 10, 10, 10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 
			8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 1, 8, 8, 8, 8, 8, 8, 8, 8, 
			12, 8, 12, 12, 12, 12, 12, 12, 12, 8, 12, 12, 12, 12, 12, 12, 12, 12, 10, 10, 10, 10, 10, 10, 
			10, 10, 8, 10, 10, 10, 10, 10, 10, 10, 10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 
			8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 12, 12, 12, 12, 12, 12, 12, 12, 10, 10, 10, 10, 10, 
			10, 10, 10};
	static short actionsStart[] = {
			0, 2, 4, 5, 7, 11, 15, 19, 23, 25, 26, 34, 2, 35, 36, 40, 50, 58, 70, 82, 94, 99, 111, 123, 
			135, 147, 159, 50, 161, 171, 181, 191, 194, 204, 205, 215, 225, 235, 50, 245, 253, 261, 269, 277, 285, 293, 301, 26, 
			26, 26, 26, 26, 26, 26, 309, 317, 325, 325, 325, 325, 325, 325, 325, 333, 334, 342, 50, 50, 50, 50, 50, 50, 
			350, 50, 362, 374, 386, 398, 410, 422, 434, 50, 58, 70, 446, 99, 111, 123, 135, 147, 458, 468, 478, 488, 498, 508, 
			518, 528, 50, 161, 171, 538, 194, 205, 215, 225, 235, 548, 556, 564, 572, 580, 588, 596, 604, 612, 309, 309, 309, 309, 
			309, 309, 309, 620, 334, 334, 334, 334, 334, 334, 334, 350, 362, 374, 628, 640, 652, 422, 664, 458, 468, 478, 676, 686, 
			696, 518, 706};
	static dActionEntry actionTable[] = {
			dActionEntry (59, 0, 0, 5, 0, 0), dActionEntry (290, 0, 0, 8, 0, 0), dActionEntry (44, 0, 0, 11, 0, 0), dActionEntry (61, 0, 0, 10, 0, 0), 
			dActionEntry (254, 0, 1, 9, 1, 32), dActionEntry (44, 0, 1, 3, 1, 19), dActionEntry (61, 0, 1, 3, 1, 19), dActionEntry (59, 0, 0, 5, 0, 0), 
			dActionEntry (254, 0, 1, 8, 1, 30), dActionEntry (273, 0, 0, 15, 0, 0), dActionEntry (290, 0, 0, 8, 0, 0), dActionEntry (59, 0, 1, 5, 1, 22), 
			dActionEntry (254, 0, 1, 5, 1, 22), dActionEntry (273, 0, 1, 5, 1, 22), dActionEntry (290, 0, 1, 5, 1, 22), dActionEntry (59, 0, 1, 6, 1, 24), 
			dActionEntry (254, 0, 1, 6, 1, 24), dActionEntry (273, 0, 1, 6, 1, 24), dActionEntry (290, 0, 1, 6, 1, 24), dActionEntry (59, 0, 1, 5, 1, 23), 
			dActionEntry (254, 0, 1, 5, 1, 23), dActionEntry (273, 0, 1, 5, 1, 23), dActionEntry (290, 0, 1, 5, 1, 23), dActionEntry (44, 0, 1, 2, 1, 18), 
			dActionEntry (61, 0, 1, 2, 1, 18), dActionEntry (254, 0, 2, 0, 0, 0), dActionEntry (40, 0, 0, 16, 0, 0), dActionEntry (262, 0, 0, 18, 0, 0), 
			dActionEntry (269, 0, 0, 21, 0, 0), dActionEntry (275, 0, 0, 17, 0, 0), dActionEntry (288, 0, 0, 23, 0, 0), dActionEntry (289, 0, 0, 25, 0, 0), 
			dActionEntry (290, 0, 0, 24, 0, 0), dActionEntry (291, 0, 0, 22, 0, 0), dActionEntry (290, 0, 0, 8, 0, 0), dActionEntry (254, 0, 1, 8, 2, 31), 
			dActionEntry (59, 0, 1, 6, 2, 25), dActionEntry (254, 0, 1, 6, 2, 25), dActionEntry (273, 0, 1, 6, 2, 25), dActionEntry (290, 0, 1, 6, 2, 25), 
			dActionEntry (40, 0, 0, 27, 0, 0), dActionEntry (59, 0, 0, 33, 0, 0), dActionEntry (254, 0, 1, 7, 1, 26), dActionEntry (262, 0, 0, 29, 0, 0), 
			dActionEntry (269, 0, 0, 32, 0, 0), dActionEntry (275, 0, 0, 28, 0, 0), dActionEntry (288, 0, 0, 35, 0, 0), dActionEntry (289, 0, 0, 37, 0, 0), 
			dActionEntry (290, 0, 0, 36, 0, 0), dActionEntry (291, 0, 0, 34, 0, 0), dActionEntry (40, 0, 0, 38, 0, 0), dActionEntry (262, 0, 0, 40, 0, 0), 
			dActionEntry (269, 0, 0, 42, 0, 0), dActionEntry (275, 0, 0, 39, 0, 0), dActionEntry (288, 0, 0, 44, 0, 0), dActionEntry (289, 0, 0, 46, 0, 0), 
			dActionEntry (290, 0, 0, 45, 0, 0), dActionEntry (291, 0, 0, 43, 0, 0), dActionEntry (42, 0, 1, 0, 1, 10), dActionEntry (43, 0, 1, 0, 1, 10), 
			dActionEntry (44, 0, 1, 0, 1, 10), dActionEntry (45, 0, 1, 0, 1, 10), dActionEntry (47, 0, 1, 0, 1, 10), dActionEntry (59, 0, 1, 0, 1, 10), 
			dActionEntry (254, 0, 1, 0, 1, 10), dActionEntry (271, 0, 1, 0, 1, 10), dActionEntry (273, 0, 1, 0, 1, 10), dActionEntry (280, 0, 1, 0, 1, 10), 
			dActionEntry (281, 0, 1, 0, 1, 10), dActionEntry (290, 0, 1, 0, 1, 10), dActionEntry (42, 0, 1, 0, 1, 11), dActionEntry (43, 0, 1, 0, 1, 11), 
			dActionEntry (44, 0, 1, 0, 1, 11), dActionEntry (45, 0, 1, 0, 1, 11), dActionEntry (47, 0, 1, 0, 1, 11), dActionEntry (59, 0, 1, 0, 1, 11), 
			dActionEntry (254, 0, 1, 0, 1, 11), dActionEntry (271, 0, 1, 0, 1, 11), dActionEntry (273, 0, 1, 0, 1, 11), dActionEntry (280, 0, 1, 0, 1, 11), 
			dActionEntry (281, 0, 1, 0, 1, 11), dActionEntry (290, 0, 1, 0, 1, 11), dActionEntry (42, 0, 0, 48, 0, 0), dActionEntry (43, 0, 0, 49, 0, 0), 
			dActionEntry (44, 0, 1, 1, 1, 16), dActionEntry (45, 0, 0, 51, 0, 0), dActionEntry (47, 0, 0, 47, 0, 0), dActionEntry (59, 0, 1, 1, 1, 16), 
			dActionEntry (254, 0, 1, 1, 1, 16), dActionEntry (271, 0, 0, 50, 0, 0), dActionEntry (273, 0, 1, 1, 1, 16), dActionEntry (280, 0, 0, 52, 0, 0), 
			dActionEntry (281, 0, 0, 53, 0, 0), dActionEntry (290, 0, 1, 1, 1, 16), dActionEntry (44, 0, 0, 54, 0, 0), dActionEntry (59, 0, 1, 4, 3, 21), 
			dActionEntry (254, 0, 1, 4, 3, 21), dActionEntry (273, 0, 1, 4, 3, 21), dActionEntry (290, 0, 1, 4, 3, 21), dActionEntry (42, 0, 1, 0, 1, 9), 
			dActionEntry (43, 0, 1, 0, 1, 9), dActionEntry (44, 0, 1, 0, 1, 9), dActionEntry (45, 0, 1, 0, 1, 9), dActionEntry (47, 0, 1, 0, 1, 9), 
			dActionEntry (59, 0, 1, 0, 1, 9), dActionEntry (254, 0, 1, 0, 1, 9), dActionEntry (271, 0, 1, 0, 1, 9), dActionEntry (273, 0, 1, 0, 1, 9), 
			dActionEntry (280, 0, 1, 0, 1, 9), dActionEntry (281, 0, 1, 0, 1, 9), dActionEntry (290, 0, 1, 0, 1, 9), dActionEntry (42, 0, 1, 0, 1, 14), 
			dActionEntry (43, 0, 1, 0, 1, 14), dActionEntry (44, 0, 1, 0, 1, 14), dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), 
			dActionEntry (59, 0, 1, 0, 1, 14), dActionEntry (254, 0, 1, 0, 1, 14), dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (273, 0, 1, 0, 1, 14), 
			dActionEntry (280, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (290, 0, 1, 0, 1, 14), dActionEntry (42, 0, 1, 0, 1, 15), 
			dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (44, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), 
			dActionEntry (59, 0, 1, 0, 1, 15), dActionEntry (254, 0, 1, 0, 1, 15), dActionEntry (271, 0, 1, 0, 1, 15), dActionEntry (273, 0, 1, 0, 1, 15), 
			dActionEntry (280, 0, 1, 0, 1, 15), dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (290, 0, 1, 0, 1, 15), dActionEntry (42, 0, 1, 0, 1, 13), 
			dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (44, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), 
			dActionEntry (59, 0, 1, 0, 1, 13), dActionEntry (254, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (273, 0, 1, 0, 1, 13), 
			dActionEntry (280, 0, 1, 0, 1, 13), dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (290, 0, 1, 0, 1, 13), dActionEntry (42, 0, 1, 0, 1, 12), 
			dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (44, 0, 1, 0, 1, 12), dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), 
			dActionEntry (59, 0, 1, 0, 1, 12), dActionEntry (254, 0, 1, 0, 1, 12), dActionEntry (271, 0, 1, 0, 1, 12), dActionEntry (273, 0, 1, 0, 1, 12), 
			dActionEntry (280, 0, 1, 0, 1, 12), dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (290, 0, 1, 0, 1, 12), dActionEntry (44, 0, 1, 3, 3, 20), 
			dActionEntry (61, 0, 1, 3, 3, 20), dActionEntry (42, 0, 1, 0, 1, 10), dActionEntry (43, 0, 1, 0, 1, 10), dActionEntry (44, 0, 1, 0, 1, 10), 
			dActionEntry (45, 0, 1, 0, 1, 10), dActionEntry (47, 0, 1, 0, 1, 10), dActionEntry (59, 0, 1, 0, 1, 10), dActionEntry (254, 0, 1, 0, 1, 10), 
			dActionEntry (271, 0, 1, 0, 1, 10), dActionEntry (280, 0, 1, 0, 1, 10), dActionEntry (281, 0, 1, 0, 1, 10), dActionEntry (42, 0, 1, 0, 1, 11), 
			dActionEntry (43, 0, 1, 0, 1, 11), dActionEntry (44, 0, 1, 0, 1, 11), dActionEntry (45, 0, 1, 0, 1, 11), dActionEntry (47, 0, 1, 0, 1, 11), 
			dActionEntry (59, 0, 1, 0, 1, 11), dActionEntry (254, 0, 1, 0, 1, 11), dActionEntry (271, 0, 1, 0, 1, 11), dActionEntry (280, 0, 1, 0, 1, 11), 
			dActionEntry (281, 0, 1, 0, 1, 11), dActionEntry (42, 0, 0, 57, 0, 0), dActionEntry (43, 0, 0, 58, 0, 0), dActionEntry (44, 0, 1, 1, 1, 16), 
			dActionEntry (45, 0, 0, 60, 0, 0), dActionEntry (47, 0, 0, 56, 0, 0), dActionEntry (59, 0, 1, 1, 1, 16), dActionEntry (254, 0, 1, 1, 1, 16), 
			dActionEntry (271, 0, 0, 59, 0, 0), dActionEntry (280, 0, 0, 61, 0, 0), dActionEntry (281, 0, 0, 62, 0, 0), dActionEntry (44, 0, 0, 64, 0, 0), 
			dActionEntry (59, 0, 0, 63, 0, 0), dActionEntry (254, 0, 1, 7, 2, 28), dActionEntry (42, 0, 1, 0, 1, 9), dActionEntry (43, 0, 1, 0, 1, 9), 
			dActionEntry (44, 0, 1, 0, 1, 9), dActionEntry (45, 0, 1, 0, 1, 9), dActionEntry (47, 0, 1, 0, 1, 9), dActionEntry (59, 0, 1, 0, 1, 9), 
			dActionEntry (254, 0, 1, 0, 1, 9), dActionEntry (271, 0, 1, 0, 1, 9), dActionEntry (280, 0, 1, 0, 1, 9), dActionEntry (281, 0, 1, 0, 1, 9), 
			dActionEntry (254, 0, 1, 7, 2, 27), dActionEntry (42, 0, 1, 0, 1, 14), dActionEntry (43, 0, 1, 0, 1, 14), dActionEntry (44, 0, 1, 0, 1, 14), 
			dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), dActionEntry (59, 0, 1, 0, 1, 14), dActionEntry (254, 0, 1, 0, 1, 14), 
			dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (280, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (42, 0, 1, 0, 1, 15), 
			dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (44, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), 
			dActionEntry (59, 0, 1, 0, 1, 15), dActionEntry (254, 0, 1, 0, 1, 15), dActionEntry (271, 0, 1, 0, 1, 15), dActionEntry (280, 0, 1, 0, 1, 15), 
			dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (42, 0, 1, 0, 1, 13), dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (44, 0, 1, 0, 1, 13), 
			dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), dActionEntry (59, 0, 1, 0, 1, 13), dActionEntry (254, 0, 1, 0, 1, 13), 
			dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (280, 0, 1, 0, 1, 13), dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (42, 0, 1, 0, 1, 12), 
			dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (44, 0, 1, 0, 1, 12), dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), 
			dActionEntry (59, 0, 1, 0, 1, 12), dActionEntry (254, 0, 1, 0, 1, 12), dActionEntry (271, 0, 1, 0, 1, 12), dActionEntry (280, 0, 1, 0, 1, 12), 
			dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (41, 0, 1, 0, 1, 10), dActionEntry (42, 0, 1, 0, 1, 10), dActionEntry (43, 0, 1, 0, 1, 10), 
			dActionEntry (45, 0, 1, 0, 1, 10), dActionEntry (47, 0, 1, 0, 1, 10), dActionEntry (271, 0, 1, 0, 1, 10), dActionEntry (280, 0, 1, 0, 1, 10), 
			dActionEntry (281, 0, 1, 0, 1, 10), dActionEntry (41, 0, 1, 0, 1, 11), dActionEntry (42, 0, 1, 0, 1, 11), dActionEntry (43, 0, 1, 0, 1, 11), 
			dActionEntry (45, 0, 1, 0, 1, 11), dActionEntry (47, 0, 1, 0, 1, 11), dActionEntry (271, 0, 1, 0, 1, 11), dActionEntry (280, 0, 1, 0, 1, 11), 
			dActionEntry (281, 0, 1, 0, 1, 11), dActionEntry (41, 0, 0, 72, 0, 0), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 0, 68, 0, 0), 
			dActionEntry (45, 0, 0, 70, 0, 0), dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 0, 69, 0, 0), dActionEntry (280, 0, 0, 71, 0, 0), 
			dActionEntry (281, 0, 0, 73, 0, 0), dActionEntry (41, 0, 1, 0, 1, 9), dActionEntry (42, 0, 1, 0, 1, 9), dActionEntry (43, 0, 1, 0, 1, 9), 
			dActionEntry (45, 0, 1, 0, 1, 9), dActionEntry (47, 0, 1, 0, 1, 9), dActionEntry (271, 0, 1, 0, 1, 9), dActionEntry (280, 0, 1, 0, 1, 9), 
			dActionEntry (281, 0, 1, 0, 1, 9), dActionEntry (41, 0, 1, 0, 1, 14), dActionEntry (42, 0, 1, 0, 1, 14), dActionEntry (43, 0, 1, 0, 1, 14), 
			dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (280, 0, 1, 0, 1, 14), 
			dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (41, 0, 1, 0, 1, 15), dActionEntry (42, 0, 1, 0, 1, 15), dActionEntry (43, 0, 1, 0, 1, 15), 
			dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), dActionEntry (271, 0, 1, 0, 1, 15), dActionEntry (280, 0, 1, 0, 1, 15), 
			dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (41, 0, 1, 0, 1, 13), dActionEntry (42, 0, 1, 0, 1, 13), dActionEntry (43, 0, 1, 0, 1, 13), 
			dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (280, 0, 1, 0, 1, 13), 
			dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (41, 0, 1, 0, 1, 12), dActionEntry (42, 0, 1, 0, 1, 12), dActionEntry (43, 0, 1, 0, 1, 12), 
			dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), dActionEntry (271, 0, 1, 0, 1, 12), dActionEntry (280, 0, 1, 0, 1, 12), 
			dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (40, 0, 0, 81, 0, 0), dActionEntry (262, 0, 0, 83, 0, 0), dActionEntry (269, 0, 0, 85, 0, 0), 
			dActionEntry (275, 0, 0, 82, 0, 0), dActionEntry (288, 0, 0, 87, 0, 0), dActionEntry (289, 0, 0, 89, 0, 0), dActionEntry (290, 0, 0, 88, 0, 0), 
			dActionEntry (291, 0, 0, 86, 0, 0), dActionEntry (41, 0, 0, 90, 0, 0), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 0, 68, 0, 0), 
			dActionEntry (45, 0, 0, 70, 0, 0), dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 0, 69, 0, 0), dActionEntry (280, 0, 0, 71, 0, 0), 
			dActionEntry (281, 0, 0, 73, 0, 0), dActionEntry (40, 0, 0, 27, 0, 0), dActionEntry (262, 0, 0, 29, 0, 0), dActionEntry (269, 0, 0, 32, 0, 0), 
			dActionEntry (275, 0, 0, 28, 0, 0), dActionEntry (288, 0, 0, 35, 0, 0), dActionEntry (289, 0, 0, 37, 0, 0), dActionEntry (290, 0, 0, 36, 0, 0), 
			dActionEntry (291, 0, 0, 34, 0, 0), dActionEntry (254, 0, 1, 7, 3, 29), dActionEntry (40, 0, 0, 98, 0, 0), dActionEntry (262, 0, 0, 100, 0, 0), 
			dActionEntry (269, 0, 0, 102, 0, 0), dActionEntry (275, 0, 0, 99, 0, 0), dActionEntry (288, 0, 0, 104, 0, 0), dActionEntry (289, 0, 0, 106, 0, 0), 
			dActionEntry (290, 0, 0, 105, 0, 0), dActionEntry (291, 0, 0, 103, 0, 0), dActionEntry (41, 0, 0, 107, 0, 0), dActionEntry (42, 0, 0, 67, 0, 0), 
			dActionEntry (43, 0, 0, 68, 0, 0), dActionEntry (45, 0, 0, 70, 0, 0), dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 0, 69, 0, 0), 
			dActionEntry (280, 0, 0, 71, 0, 0), dActionEntry (281, 0, 0, 73, 0, 0), dActionEntry (42, 0, 1, 0, 3, 8), dActionEntry (43, 0, 1, 0, 3, 8), 
			dActionEntry (44, 0, 1, 0, 3, 8), dActionEntry (45, 0, 1, 0, 3, 8), dActionEntry (47, 0, 1, 0, 3, 8), dActionEntry (59, 0, 1, 0, 3, 8), 
			dActionEntry (254, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (273, 0, 1, 0, 3, 8), dActionEntry (280, 0, 1, 0, 3, 8), 
			dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (290, 0, 1, 0, 3, 8), dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), 
			dActionEntry (44, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (59, 0, 1, 0, 3, 6), 
			dActionEntry (254, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (273, 0, 1, 0, 3, 6), dActionEntry (280, 0, 1, 0, 3, 6), 
			dActionEntry (281, 0, 1, 0, 3, 6), dActionEntry (290, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), 
			dActionEntry (44, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (59, 0, 1, 0, 3, 5), 
			dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (273, 0, 1, 0, 3, 5), dActionEntry (280, 0, 1, 0, 3, 5), 
			dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (290, 0, 1, 0, 3, 5), dActionEntry (42, 0, 0, 48, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), 
			dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 0, 47, 0, 0), dActionEntry (59, 0, 1, 0, 3, 3), 
			dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (273, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 52, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (290, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 48, 0, 0), dActionEntry (43, 0, 0, 49, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 0, 51, 0, 0), dActionEntry (47, 0, 0, 47, 0, 0), dActionEntry (59, 0, 1, 0, 3, 1), 
			dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (273, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 52, 0, 0), 
			dActionEntry (281, 0, 0, 53, 0, 0), dActionEntry (290, 0, 1, 0, 3, 1), dActionEntry (42, 0, 0, 48, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), 
			dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 0, 47, 0, 0), dActionEntry (59, 0, 1, 0, 3, 4), 
			dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (273, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 52, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (290, 0, 1, 0, 3, 4), dActionEntry (42, 0, 1, 0, 3, 7), dActionEntry (43, 0, 1, 0, 3, 7), 
			dActionEntry (44, 0, 1, 0, 3, 7), dActionEntry (45, 0, 1, 0, 3, 7), dActionEntry (47, 0, 1, 0, 3, 7), dActionEntry (59, 0, 1, 0, 3, 7), 
			dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (273, 0, 1, 0, 3, 7), dActionEntry (280, 0, 1, 0, 3, 7), 
			dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (290, 0, 1, 0, 3, 7), dActionEntry (42, 0, 0, 48, 0, 0), dActionEntry (43, 0, 0, 49, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 0, 51, 0, 0), dActionEntry (47, 0, 0, 47, 0, 0), dActionEntry (59, 0, 1, 0, 3, 2), 
			dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (273, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 52, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (290, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 117, 0, 0), dActionEntry (43, 0, 0, 118, 0, 0), 
			dActionEntry (44, 0, 1, 1, 3, 17), dActionEntry (45, 0, 0, 120, 0, 0), dActionEntry (47, 0, 0, 116, 0, 0), dActionEntry (59, 0, 1, 1, 3, 17), 
			dActionEntry (254, 0, 1, 1, 3, 17), dActionEntry (271, 0, 0, 119, 0, 0), dActionEntry (273, 0, 1, 1, 3, 17), dActionEntry (280, 0, 0, 121, 0, 0), 
			dActionEntry (281, 0, 0, 122, 0, 0), dActionEntry (290, 0, 1, 1, 3, 17), dActionEntry (42, 0, 1, 0, 3, 8), dActionEntry (43, 0, 1, 0, 3, 8), 
			dActionEntry (44, 0, 1, 0, 3, 8), dActionEntry (45, 0, 1, 0, 3, 8), dActionEntry (47, 0, 1, 0, 3, 8), dActionEntry (59, 0, 1, 0, 3, 8), 
			dActionEntry (254, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (280, 0, 1, 0, 3, 8), dActionEntry (281, 0, 1, 0, 3, 8), 
			dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (44, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), 
			dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (59, 0, 1, 0, 3, 6), dActionEntry (254, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), 
			dActionEntry (280, 0, 1, 0, 3, 6), dActionEntry (281, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), 
			dActionEntry (44, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (59, 0, 1, 0, 3, 5), 
			dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (280, 0, 1, 0, 3, 5), dActionEntry (281, 0, 1, 0, 3, 5), 
			dActionEntry (42, 0, 0, 57, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 0, 56, 0, 0), dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), 
			dActionEntry (280, 0, 0, 61, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 57, 0, 0), dActionEntry (43, 0, 0, 58, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 0, 60, 0, 0), dActionEntry (47, 0, 0, 56, 0, 0), dActionEntry (59, 0, 1, 0, 3, 1), 
			dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 61, 0, 0), dActionEntry (281, 0, 0, 62, 0, 0), 
			dActionEntry (42, 0, 0, 57, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), 
			dActionEntry (47, 0, 0, 56, 0, 0), dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), 
			dActionEntry (280, 0, 0, 61, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (42, 0, 1, 0, 3, 7), dActionEntry (43, 0, 1, 0, 3, 7), 
			dActionEntry (44, 0, 1, 0, 3, 7), dActionEntry (45, 0, 1, 0, 3, 7), dActionEntry (47, 0, 1, 0, 3, 7), dActionEntry (59, 0, 1, 0, 3, 7), 
			dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (280, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), 
			dActionEntry (42, 0, 0, 57, 0, 0), dActionEntry (43, 0, 0, 58, 0, 0), dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 0, 60, 0, 0), 
			dActionEntry (47, 0, 0, 56, 0, 0), dActionEntry (59, 0, 1, 0, 3, 2), dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), 
			dActionEntry (280, 0, 0, 61, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 125, 0, 0), dActionEntry (43, 0, 0, 126, 0, 0), 
			dActionEntry (44, 0, 1, 1, 3, 17), dActionEntry (45, 0, 0, 128, 0, 0), dActionEntry (47, 0, 0, 124, 0, 0), dActionEntry (59, 0, 1, 1, 3, 17), 
			dActionEntry (254, 0, 1, 1, 3, 17), dActionEntry (271, 0, 0, 127, 0, 0), dActionEntry (280, 0, 0, 129, 0, 0), dActionEntry (281, 0, 0, 130, 0, 0), 
			dActionEntry (41, 0, 1, 0, 3, 8), dActionEntry (42, 0, 1, 0, 3, 8), dActionEntry (43, 0, 1, 0, 3, 8), dActionEntry (45, 0, 1, 0, 3, 8), 
			dActionEntry (47, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (280, 0, 1, 0, 3, 8), dActionEntry (281, 0, 1, 0, 3, 8), 
			dActionEntry (41, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), 
			dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (280, 0, 1, 0, 3, 6), dActionEntry (281, 0, 1, 0, 3, 6), 
			dActionEntry (41, 0, 1, 0, 3, 5), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), 
			dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (280, 0, 1, 0, 3, 5), dActionEntry (281, 0, 1, 0, 3, 5), 
			dActionEntry (41, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 71, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), 
			dActionEntry (41, 0, 1, 0, 3, 1), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 0, 68, 0, 0), dActionEntry (45, 0, 0, 70, 0, 0), 
			dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 71, 0, 0), dActionEntry (281, 0, 0, 73, 0, 0), 
			dActionEntry (41, 0, 1, 0, 3, 4), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), 
			dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 71, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), 
			dActionEntry (41, 0, 1, 0, 3, 7), dActionEntry (42, 0, 1, 0, 3, 7), dActionEntry (43, 0, 1, 0, 3, 7), dActionEntry (45, 0, 1, 0, 3, 7), 
			dActionEntry (47, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (280, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), 
			dActionEntry (41, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 0, 68, 0, 0), dActionEntry (45, 0, 0, 70, 0, 0), 
			dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 71, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2), 
			dActionEntry (41, 0, 0, 131, 0, 0), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 0, 68, 0, 0), dActionEntry (45, 0, 0, 70, 0, 0), 
			dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 0, 69, 0, 0), dActionEntry (280, 0, 0, 71, 0, 0), dActionEntry (281, 0, 0, 73, 0, 0), 
			dActionEntry (41, 0, 0, 139, 0, 0), dActionEntry (42, 0, 0, 67, 0, 0), dActionEntry (43, 0, 0, 68, 0, 0), dActionEntry (45, 0, 0, 70, 0, 0), 
			dActionEntry (47, 0, 0, 66, 0, 0), dActionEntry (271, 0, 0, 69, 0, 0), dActionEntry (280, 0, 0, 71, 0, 0), dActionEntry (281, 0, 0, 73, 0, 0), 
			dActionEntry (42, 0, 0, 117, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 0, 116, 0, 0), dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), 
			dActionEntry (273, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 121, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (290, 0, 1, 0, 3, 3), 
			dActionEntry (42, 0, 0, 117, 0, 0), dActionEntry (43, 0, 0, 118, 0, 0), dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 0, 120, 0, 0), 
			dActionEntry (47, 0, 0, 116, 0, 0), dActionEntry (59, 0, 1, 0, 3, 1), dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), 
			dActionEntry (273, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 121, 0, 0), dActionEntry (281, 0, 0, 122, 0, 0), dActionEntry (290, 0, 1, 0, 3, 1), 
			dActionEntry (42, 0, 0, 117, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), 
			dActionEntry (47, 0, 0, 116, 0, 0), dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), 
			dActionEntry (273, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 121, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (290, 0, 1, 0, 3, 4), 
			dActionEntry (42, 0, 0, 117, 0, 0), dActionEntry (43, 0, 0, 118, 0, 0), dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 0, 120, 0, 0), 
			dActionEntry (47, 0, 0, 116, 0, 0), dActionEntry (59, 0, 1, 0, 3, 2), dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), 
			dActionEntry (273, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 121, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (290, 0, 1, 0, 3, 2), 
			dActionEntry (42, 0, 0, 125, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 0, 124, 0, 0), dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), 
			dActionEntry (280, 0, 0, 129, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 125, 0, 0), dActionEntry (43, 0, 0, 126, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 0, 128, 0, 0), dActionEntry (47, 0, 0, 124, 0, 0), dActionEntry (59, 0, 1, 0, 3, 1), 
			dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 129, 0, 0), dActionEntry (281, 0, 0, 130, 0, 0), 
			dActionEntry (42, 0, 0, 125, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), 
			dActionEntry (47, 0, 0, 124, 0, 0), dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), 
			dActionEntry (280, 0, 0, 129, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (42, 0, 0, 125, 0, 0), dActionEntry (43, 0, 0, 126, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 0, 128, 0, 0), dActionEntry (47, 0, 0, 124, 0, 0), dActionEntry (59, 0, 1, 0, 3, 2), 
			dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 129, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2)};

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
			7, 0, 0, 0, 5, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 
			1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 
			0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 
			1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 0};
	static short gotoStart[] = {
			0, 7, 7, 7, 7, 12, 12, 12, 12, 12, 12, 14, 15, 15, 15, 15, 17, 18, 18, 18, 18, 18, 18, 18, 
			18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 
			21, 22, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 33, 34, 35, 35, 36, 36, 37, 38, 39, 40, 41, 
			42, 42, 43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 
			44, 44, 44, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 46, 47, 48, 
			49, 50, 51, 52, 52, 53, 54, 55, 56, 57, 58, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 
			59, 59, 59};
	static dGotoEntry gotoTable[] = {
			dGotoEntry (294, 3), dGotoEntry (295, 1), dGotoEntry (296, 7), dGotoEntry (297, 6), dGotoEntry (298, 4), 
			dGotoEntry (300, 2), dGotoEntry (301, 9), dGotoEntry (294, 3), dGotoEntry (295, 12), dGotoEntry (296, 7), 
			dGotoEntry (297, 14), dGotoEntry (299, 13), dGotoEntry (292, 19), dGotoEntry (293, 20), dGotoEntry (294, 26), 
			dGotoEntry (292, 30), dGotoEntry (293, 31), dGotoEntry (292, 41), dGotoEntry (292, 55), dGotoEntry (292, 65), 
			dGotoEntry (292, 74), dGotoEntry (292, 75), dGotoEntry (292, 76), dGotoEntry (292, 77), dGotoEntry (292, 78), 
			dGotoEntry (292, 79), dGotoEntry (292, 80), dGotoEntry (292, 84), dGotoEntry (292, 91), dGotoEntry (292, 92), 
			dGotoEntry (292, 93), dGotoEntry (292, 94), dGotoEntry (292, 95), dGotoEntry (292, 96), dGotoEntry (292, 97), 
			dGotoEntry (292, 101), dGotoEntry (292, 108), dGotoEntry (292, 109), dGotoEntry (292, 110), dGotoEntry (292, 111), 
			dGotoEntry (292, 112), dGotoEntry (292, 113), dGotoEntry (292, 114), dGotoEntry (292, 115), dGotoEntry (292, 123), 
			dGotoEntry (292, 132), dGotoEntry (292, 133), dGotoEntry (292, 134), dGotoEntry (292, 135), dGotoEntry (292, 136), 
			dGotoEntry (292, 137), dGotoEntry (292, 138), dGotoEntry (292, 140), dGotoEntry (292, 141), dGotoEntry (292, 142), 
			dGotoEntry (292, 143), dGotoEntry (292, 144), dGotoEntry (292, 145), dGotoEntry (292, 146)};

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
						case 32:// chunk : block 
{MyModule->CloseFunctionDeclaration();}
break;

						case 19:// variableList : variable 
{entry.m_value = parameter[0].m_value;}
break;

						case 18:// variable : _LABEL 
{entry.m_value = parameter[0].m_value;}
break;

						case 26:// returnStatement : _RETURN 
{entry.m_value = MyModule->EmitReturn(dUserVariable());}
break;

						case 10:// expression : _TRUE 
{dAssert(0);}
break;

						case 11:// expression : _FALSE 
{dAssert(0);}
break;

						case 16:// expressionList : expression 
{entry.m_value = parameter[0].m_value;}
break;

						case 21:// assigment : variableList = expressionList 
{entry.m_value = MyModule->EmitAssigmentStatement(parameter[0].m_value, parameter[2].m_value);}
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

						case 20:// variableList : variableList , variable 
{entry.m_value = MyModule->LinkExpresion(parameter[0].m_value, parameter[2].m_value);}
break;

						case 28:// returnStatement : _RETURN expressionList 
{entry.m_value = MyModule->EmitReturn(parameter[1].m_value);}
break;

						case 27:// returnStatement : _RETURN ; 
{entry.m_value = MyModule->EmitReturn(dUserVariable());}
break;

						case 29:// returnStatement : _RETURN expressionList ; 
{entry.m_value = MyModule->EmitReturn(parameter[1].m_value);}
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



