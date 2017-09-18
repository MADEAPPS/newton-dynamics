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
			1, 1, 3, 3, 3, 1, 1, 3, 10, 8, 10, 10, 10, 3, 10, 1, 10, 10, 10, 10, 8, 8, 8, 8, 
			8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 1, 8, 8, 8, 8, 8, 8, 8, 8, 10, 8, 10, 
			10, 10, 10, 10, 10, 10, 8, 10, 10, 10, 10, 10, 10, 10, 10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 
			8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10};
	static short actionsStart[] = {
			0, 1, 2, 5, 8, 11, 12, 13, 16, 26, 34, 44, 54, 64, 67, 77, 78, 88, 98, 108, 26, 118, 126, 134, 
			142, 150, 158, 166, 174, 182, 182, 182, 182, 182, 182, 182, 190, 191, 199, 26, 26, 26, 26, 26, 26, 207, 26, 217, 
			227, 237, 247, 257, 267, 277, 26, 34, 44, 287, 67, 78, 88, 98, 108, 297, 305, 313, 321, 329, 337, 345, 353, 361, 
			191, 191, 191, 191, 191, 191, 191, 207, 217, 227, 369, 379, 389, 267, 399};
	static dActionEntry actionTable[] = {
			dActionEntry (59, 0, 0, 3, 0, 0), dActionEntry (254, 0, 1, 6, 1, 27), dActionEntry (59, 0, 0, 3, 0, 0), dActionEntry (254, 0, 1, 5, 1, 25), 
			dActionEntry (273, 0, 0, 8, 0, 0), dActionEntry (59, 0, 1, 1, 1, 16), dActionEntry (254, 0, 1, 1, 1, 16), dActionEntry (273, 0, 1, 1, 1, 16), 
			dActionEntry (59, 0, 1, 2, 1, 17), dActionEntry (254, 0, 1, 2, 1, 17), dActionEntry (273, 0, 1, 2, 1, 17), dActionEntry (254, 0, 2, 0, 0, 0), 
			dActionEntry (254, 0, 1, 5, 2, 26), dActionEntry (59, 0, 1, 2, 2, 18), dActionEntry (254, 0, 1, 2, 2, 18), dActionEntry (273, 0, 1, 2, 2, 18), 
			dActionEntry (40, 0, 0, 9, 0, 0), dActionEntry (59, 0, 0, 15, 0, 0), dActionEntry (254, 0, 1, 4, 1, 21), dActionEntry (262, 0, 0, 11, 0, 0), 
			dActionEntry (269, 0, 0, 14, 0, 0), dActionEntry (275, 0, 0, 10, 0, 0), dActionEntry (288, 0, 0, 17, 0, 0), dActionEntry (289, 0, 0, 19, 0, 0), 
			dActionEntry (290, 0, 0, 18, 0, 0), dActionEntry (291, 0, 0, 16, 0, 0), dActionEntry (40, 0, 0, 20, 0, 0), dActionEntry (262, 0, 0, 22, 0, 0), 
			dActionEntry (269, 0, 0, 24, 0, 0), dActionEntry (275, 0, 0, 21, 0, 0), dActionEntry (288, 0, 0, 26, 0, 0), dActionEntry (289, 0, 0, 28, 0, 0), 
			dActionEntry (290, 0, 0, 27, 0, 0), dActionEntry (291, 0, 0, 25, 0, 0), dActionEntry (42, 0, 1, 0, 1, 10), dActionEntry (43, 0, 1, 0, 1, 10), 
			dActionEntry (44, 0, 1, 0, 1, 10), dActionEntry (45, 0, 1, 0, 1, 10), dActionEntry (47, 0, 1, 0, 1, 10), dActionEntry (59, 0, 1, 0, 1, 10), 
			dActionEntry (254, 0, 1, 0, 1, 10), dActionEntry (271, 0, 1, 0, 1, 10), dActionEntry (280, 0, 1, 0, 1, 10), dActionEntry (281, 0, 1, 0, 1, 10), 
			dActionEntry (42, 0, 1, 0, 1, 11), dActionEntry (43, 0, 1, 0, 1, 11), dActionEntry (44, 0, 1, 0, 1, 11), dActionEntry (45, 0, 1, 0, 1, 11), 
			dActionEntry (47, 0, 1, 0, 1, 11), dActionEntry (59, 0, 1, 0, 1, 11), dActionEntry (254, 0, 1, 0, 1, 11), dActionEntry (271, 0, 1, 0, 1, 11), 
			dActionEntry (280, 0, 1, 0, 1, 11), dActionEntry (281, 0, 1, 0, 1, 11), dActionEntry (42, 0, 0, 30, 0, 0), dActionEntry (43, 0, 0, 31, 0, 0), 
			dActionEntry (44, 0, 1, 3, 1, 19), dActionEntry (45, 0, 0, 33, 0, 0), dActionEntry (47, 0, 0, 29, 0, 0), dActionEntry (59, 0, 1, 3, 1, 19), 
			dActionEntry (254, 0, 1, 3, 1, 19), dActionEntry (271, 0, 0, 32, 0, 0), dActionEntry (280, 0, 0, 34, 0, 0), dActionEntry (281, 0, 0, 35, 0, 0), 
			dActionEntry (44, 0, 0, 37, 0, 0), dActionEntry (59, 0, 0, 36, 0, 0), dActionEntry (254, 0, 1, 4, 2, 23), dActionEntry (42, 0, 1, 0, 1, 9), 
			dActionEntry (43, 0, 1, 0, 1, 9), dActionEntry (44, 0, 1, 0, 1, 9), dActionEntry (45, 0, 1, 0, 1, 9), dActionEntry (47, 0, 1, 0, 1, 9), 
			dActionEntry (59, 0, 1, 0, 1, 9), dActionEntry (254, 0, 1, 0, 1, 9), dActionEntry (271, 0, 1, 0, 1, 9), dActionEntry (280, 0, 1, 0, 1, 9), 
			dActionEntry (281, 0, 1, 0, 1, 9), dActionEntry (254, 0, 1, 4, 2, 22), dActionEntry (42, 0, 1, 0, 1, 14), dActionEntry (43, 0, 1, 0, 1, 14), 
			dActionEntry (44, 0, 1, 0, 1, 14), dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), dActionEntry (59, 0, 1, 0, 1, 14), 
			dActionEntry (254, 0, 1, 0, 1, 14), dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (280, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), 
			dActionEntry (42, 0, 1, 0, 1, 15), dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (44, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), 
			dActionEntry (47, 0, 1, 0, 1, 15), dActionEntry (59, 0, 1, 0, 1, 15), dActionEntry (254, 0, 1, 0, 1, 15), dActionEntry (271, 0, 1, 0, 1, 15), 
			dActionEntry (280, 0, 1, 0, 1, 15), dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (42, 0, 1, 0, 1, 13), dActionEntry (43, 0, 1, 0, 1, 13), 
			dActionEntry (44, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), dActionEntry (59, 0, 1, 0, 1, 13), 
			dActionEntry (254, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (280, 0, 1, 0, 1, 13), dActionEntry (281, 0, 1, 0, 1, 13), 
			dActionEntry (42, 0, 1, 0, 1, 12), dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (44, 0, 1, 0, 1, 12), dActionEntry (45, 0, 1, 0, 1, 12), 
			dActionEntry (47, 0, 1, 0, 1, 12), dActionEntry (59, 0, 1, 0, 1, 12), dActionEntry (254, 0, 1, 0, 1, 12), dActionEntry (271, 0, 1, 0, 1, 12), 
			dActionEntry (280, 0, 1, 0, 1, 12), dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (41, 0, 1, 0, 1, 10), dActionEntry (42, 0, 1, 0, 1, 10), 
			dActionEntry (43, 0, 1, 0, 1, 10), dActionEntry (45, 0, 1, 0, 1, 10), dActionEntry (47, 0, 1, 0, 1, 10), dActionEntry (271, 0, 1, 0, 1, 10), 
			dActionEntry (280, 0, 1, 0, 1, 10), dActionEntry (281, 0, 1, 0, 1, 10), dActionEntry (41, 0, 1, 0, 1, 11), dActionEntry (42, 0, 1, 0, 1, 11), 
			dActionEntry (43, 0, 1, 0, 1, 11), dActionEntry (45, 0, 1, 0, 1, 11), dActionEntry (47, 0, 1, 0, 1, 11), dActionEntry (271, 0, 1, 0, 1, 11), 
			dActionEntry (280, 0, 1, 0, 1, 11), dActionEntry (281, 0, 1, 0, 1, 11), dActionEntry (41, 0, 0, 45, 0, 0), dActionEntry (42, 0, 0, 40, 0, 0), 
			dActionEntry (43, 0, 0, 41, 0, 0), dActionEntry (45, 0, 0, 43, 0, 0), dActionEntry (47, 0, 0, 39, 0, 0), dActionEntry (271, 0, 0, 42, 0, 0), 
			dActionEntry (280, 0, 0, 44, 0, 0), dActionEntry (281, 0, 0, 46, 0, 0), dActionEntry (41, 0, 1, 0, 1, 9), dActionEntry (42, 0, 1, 0, 1, 9), 
			dActionEntry (43, 0, 1, 0, 1, 9), dActionEntry (45, 0, 1, 0, 1, 9), dActionEntry (47, 0, 1, 0, 1, 9), dActionEntry (271, 0, 1, 0, 1, 9), 
			dActionEntry (280, 0, 1, 0, 1, 9), dActionEntry (281, 0, 1, 0, 1, 9), dActionEntry (41, 0, 1, 0, 1, 14), dActionEntry (42, 0, 1, 0, 1, 14), 
			dActionEntry (43, 0, 1, 0, 1, 14), dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), dActionEntry (271, 0, 1, 0, 1, 14), 
			dActionEntry (280, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (41, 0, 1, 0, 1, 15), dActionEntry (42, 0, 1, 0, 1, 15), 
			dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), dActionEntry (271, 0, 1, 0, 1, 15), 
			dActionEntry (280, 0, 1, 0, 1, 15), dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (41, 0, 1, 0, 1, 13), dActionEntry (42, 0, 1, 0, 1, 13), 
			dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), 
			dActionEntry (280, 0, 1, 0, 1, 13), dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (41, 0, 1, 0, 1, 12), dActionEntry (42, 0, 1, 0, 1, 12), 
			dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), dActionEntry (271, 0, 1, 0, 1, 12), 
			dActionEntry (280, 0, 1, 0, 1, 12), dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (40, 0, 0, 9, 0, 0), dActionEntry (262, 0, 0, 11, 0, 0), 
			dActionEntry (269, 0, 0, 14, 0, 0), dActionEntry (275, 0, 0, 10, 0, 0), dActionEntry (288, 0, 0, 17, 0, 0), dActionEntry (289, 0, 0, 19, 0, 0), 
			dActionEntry (290, 0, 0, 18, 0, 0), dActionEntry (291, 0, 0, 16, 0, 0), dActionEntry (254, 0, 1, 4, 3, 24), dActionEntry (40, 0, 0, 54, 0, 0), 
			dActionEntry (262, 0, 0, 56, 0, 0), dActionEntry (269, 0, 0, 58, 0, 0), dActionEntry (275, 0, 0, 55, 0, 0), dActionEntry (288, 0, 0, 60, 0, 0), 
			dActionEntry (289, 0, 0, 62, 0, 0), dActionEntry (290, 0, 0, 61, 0, 0), dActionEntry (291, 0, 0, 59, 0, 0), dActionEntry (41, 0, 0, 63, 0, 0), 
			dActionEntry (42, 0, 0, 40, 0, 0), dActionEntry (43, 0, 0, 41, 0, 0), dActionEntry (45, 0, 0, 43, 0, 0), dActionEntry (47, 0, 0, 39, 0, 0), 
			dActionEntry (271, 0, 0, 42, 0, 0), dActionEntry (280, 0, 0, 44, 0, 0), dActionEntry (281, 0, 0, 46, 0, 0), dActionEntry (42, 0, 1, 0, 3, 8), 
			dActionEntry (43, 0, 1, 0, 3, 8), dActionEntry (44, 0, 1, 0, 3, 8), dActionEntry (45, 0, 1, 0, 3, 8), dActionEntry (47, 0, 1, 0, 3, 8), 
			dActionEntry (59, 0, 1, 0, 3, 8), dActionEntry (254, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (280, 0, 1, 0, 3, 8), 
			dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (44, 0, 1, 0, 3, 6), 
			dActionEntry (45, 0, 1, 0, 3, 6), dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (59, 0, 1, 0, 3, 6), dActionEntry (254, 0, 1, 0, 3, 6), 
			dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (280, 0, 1, 0, 3, 6), dActionEntry (281, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 5), 
			dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (44, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), dActionEntry (47, 0, 1, 0, 3, 5), 
			dActionEntry (59, 0, 1, 0, 3, 5), dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (280, 0, 1, 0, 3, 5), 
			dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (42, 0, 0, 30, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), 
			dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 0, 29, 0, 0), dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (254, 0, 1, 0, 3, 3), 
			dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 34, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 30, 0, 0), 
			dActionEntry (43, 0, 0, 31, 0, 0), dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 0, 33, 0, 0), dActionEntry (47, 0, 0, 29, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 1), dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 34, 0, 0), 
			dActionEntry (281, 0, 0, 35, 0, 0), dActionEntry (42, 0, 0, 30, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), 
			dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 0, 29, 0, 0), dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (254, 0, 1, 0, 3, 4), 
			dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 34, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (42, 0, 1, 0, 3, 7), 
			dActionEntry (43, 0, 1, 0, 3, 7), dActionEntry (44, 0, 1, 0, 3, 7), dActionEntry (45, 0, 1, 0, 3, 7), dActionEntry (47, 0, 1, 0, 3, 7), 
			dActionEntry (59, 0, 1, 0, 3, 7), dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (280, 0, 1, 0, 3, 7), 
			dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (42, 0, 0, 30, 0, 0), dActionEntry (43, 0, 0, 31, 0, 0), dActionEntry (44, 0, 1, 0, 3, 2), 
			dActionEntry (45, 0, 0, 33, 0, 0), dActionEntry (47, 0, 0, 29, 0, 0), dActionEntry (59, 0, 1, 0, 3, 2), dActionEntry (254, 0, 1, 0, 3, 2), 
			dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 34, 0, 0), dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 73, 0, 0), 
			dActionEntry (43, 0, 0, 74, 0, 0), dActionEntry (44, 0, 1, 3, 3, 20), dActionEntry (45, 0, 0, 76, 0, 0), dActionEntry (47, 0, 0, 72, 0, 0), 
			dActionEntry (59, 0, 1, 3, 3, 20), dActionEntry (254, 0, 1, 3, 3, 20), dActionEntry (271, 0, 0, 75, 0, 0), dActionEntry (280, 0, 0, 77, 0, 0), 
			dActionEntry (281, 0, 0, 78, 0, 0), dActionEntry (41, 0, 1, 0, 3, 8), dActionEntry (42, 0, 1, 0, 3, 8), dActionEntry (43, 0, 1, 0, 3, 8), 
			dActionEntry (45, 0, 1, 0, 3, 8), dActionEntry (47, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (280, 0, 1, 0, 3, 8), 
			dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (41, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), 
			dActionEntry (45, 0, 1, 0, 3, 6), dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (280, 0, 1, 0, 3, 6), 
			dActionEntry (281, 0, 1, 0, 3, 6), dActionEntry (41, 0, 1, 0, 3, 5), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), 
			dActionEntry (45, 0, 1, 0, 3, 5), dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (280, 0, 1, 0, 3, 5), 
			dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (41, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 40, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), 
			dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 0, 39, 0, 0), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 44, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (41, 0, 1, 0, 3, 1), dActionEntry (42, 0, 0, 40, 0, 0), dActionEntry (43, 0, 0, 41, 0, 0), 
			dActionEntry (45, 0, 0, 43, 0, 0), dActionEntry (47, 0, 0, 39, 0, 0), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 44, 0, 0), 
			dActionEntry (281, 0, 0, 46, 0, 0), dActionEntry (41, 0, 1, 0, 3, 4), dActionEntry (42, 0, 0, 40, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), 
			dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 0, 39, 0, 0), dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 44, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (41, 0, 1, 0, 3, 7), dActionEntry (42, 0, 1, 0, 3, 7), dActionEntry (43, 0, 1, 0, 3, 7), 
			dActionEntry (45, 0, 1, 0, 3, 7), dActionEntry (47, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (280, 0, 1, 0, 3, 7), 
			dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (41, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 40, 0, 0), dActionEntry (43, 0, 0, 41, 0, 0), 
			dActionEntry (45, 0, 0, 43, 0, 0), dActionEntry (47, 0, 0, 39, 0, 0), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 44, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (41, 0, 0, 79, 0, 0), dActionEntry (42, 0, 0, 40, 0, 0), dActionEntry (43, 0, 0, 41, 0, 0), 
			dActionEntry (45, 0, 0, 43, 0, 0), dActionEntry (47, 0, 0, 39, 0, 0), dActionEntry (271, 0, 0, 42, 0, 0), dActionEntry (280, 0, 0, 44, 0, 0), 
			dActionEntry (281, 0, 0, 46, 0, 0), dActionEntry (42, 0, 0, 73, 0, 0), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), 
			dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 0, 72, 0, 0), dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (254, 0, 1, 0, 3, 3), 
			dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (280, 0, 0, 77, 0, 0), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (42, 0, 0, 73, 0, 0), 
			dActionEntry (43, 0, 0, 74, 0, 0), dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 0, 76, 0, 0), dActionEntry (47, 0, 0, 72, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 1), dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (280, 0, 0, 77, 0, 0), 
			dActionEntry (281, 0, 0, 78, 0, 0), dActionEntry (42, 0, 0, 73, 0, 0), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), 
			dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 0, 72, 0, 0), dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (254, 0, 1, 0, 3, 4), 
			dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (280, 0, 0, 77, 0, 0), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (42, 0, 0, 73, 0, 0), 
			dActionEntry (43, 0, 0, 74, 0, 0), dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 0, 76, 0, 0), dActionEntry (47, 0, 0, 72, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 2), dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (280, 0, 0, 77, 0, 0), 
			dActionEntry (281, 0, 1, 0, 3, 2)};

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
			4, 0, 2, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 
			0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
	static short gotoStart[] = {
			0, 4, 4, 6, 6, 6, 6, 6, 6, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 
			10, 10, 10, 10, 10, 10, 11, 12, 13, 14, 15, 16, 17, 17, 18, 18, 19, 20, 21, 22, 23, 24, 24, 25, 
			25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 
			26, 27, 28, 29, 30, 31, 32, 33, 33, 33, 33, 33, 33, 33, 33};
	static dGotoEntry gotoTable[] = {
			dGotoEntry (293, 4), dGotoEntry (294, 2), dGotoEntry (297, 1), dGotoEntry (298, 5), dGotoEntry (293, 7), 
			dGotoEntry (296, 6), dGotoEntry (292, 12), dGotoEntry (295, 13), dGotoEntry (292, 23), dGotoEntry (292, 38), 
			dGotoEntry (292, 47), dGotoEntry (292, 48), dGotoEntry (292, 49), dGotoEntry (292, 50), dGotoEntry (292, 51), 
			dGotoEntry (292, 52), dGotoEntry (292, 53), dGotoEntry (292, 57), dGotoEntry (292, 64), dGotoEntry (292, 65), 
			dGotoEntry (292, 66), dGotoEntry (292, 67), dGotoEntry (292, 68), dGotoEntry (292, 69), dGotoEntry (292, 70), 
			dGotoEntry (292, 71), dGotoEntry (292, 80), dGotoEntry (292, 81), dGotoEntry (292, 82), dGotoEntry (292, 83), 
			dGotoEntry (292, 84), dGotoEntry (292, 85), dGotoEntry (292, 86)};

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
						case 27:// chunk : block 
{MyModule->CloseFunctionDeclaration();}
break;

						case 21:// returnStatement : _RETURN 
{entry.m_value = MyModule->EmitReturn(dUserVariable());}
break;

						case 10:// expression : _TRUE 
{dAssert(0);}
break;

						case 11:// expression : _FALSE 
{dAssert(0);}
break;

						case 19:// expressionList : expression 
{entry.m_value = parameter[0].m_value;}
break;

						case 23:// returnStatement : _RETURN expressionList 
{entry.m_value = MyModule->EmitReturn(parameter[1].m_value);}
break;

						case 9:// expression : _NIL 
{dAssert(0);}
break;

						case 22:// returnStatement : _RETURN ; 
{entry.m_value = MyModule->EmitReturn(dUserVariable());}
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

						case 24:// returnStatement : _RETURN expressionList ; 
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

						case 20:// expressionList : expressionList , expression 
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



