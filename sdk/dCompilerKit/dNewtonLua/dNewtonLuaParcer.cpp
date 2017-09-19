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
			3, 2, 1, 2, 5, 5, 5, 5, 1, 5, 2, 6, 1, 8, 1, 2, 1, 5, 10, 7, 7, 8, 8, 16, 
			16, 16, 6, 16, 16, 16, 16, 16, 2, 8, 13, 13, 13, 3, 13, 1, 13, 13, 13, 13, 1, 6, 8, 11, 
			11, 11, 11, 11, 11, 11, 11, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 8, 8, 8, 8, 8, 
			8, 8, 8, 8, 8, 1, 8, 7, 11, 8, 8, 8, 8, 8, 8, 8, 8, 8, 16, 8, 16, 16, 16, 16, 
			16, 16, 16, 16, 16, 16, 8, 16, 16, 16, 16, 16, 16, 16, 16, 13, 13, 13, 13, 13, 13, 13, 13, 13, 
			13, 13, 8, 13, 13, 13, 13, 13, 13, 13, 13, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 8, 
			8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 16, 16, 16, 16, 
			16, 16, 16, 16, 16, 16, 16, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13};
	static short actionsStart[] = {
			0, 3, 5, 6, 8, 13, 18, 23, 28, 29, 34, 36, 42, 43, 51, 3, 52, 53, 58, 68, 75, 43, 82, 90, 
			106, 122, 138, 144, 160, 176, 192, 208, 224, 82, 226, 239, 252, 265, 268, 281, 282, 295, 308, 321, 334, 335, 82, 341, 
			352, 363, 374, 385, 396, 407, 418, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 429, 437, 448, 448, 448, 448, 448, 
			448, 448, 448, 448, 448, 456, 457, 465, 472, 82, 82, 82, 82, 82, 82, 82, 82, 82, 483, 82, 499, 515, 531, 547, 
			563, 579, 595, 611, 627, 643, 82, 90, 106, 659, 144, 160, 176, 192, 208, 675, 688, 701, 714, 727, 740, 753, 766, 779, 
			792, 805, 82, 226, 239, 818, 268, 282, 295, 308, 321, 831, 842, 853, 864, 875, 886, 897, 908, 919, 930, 941, 952, 429, 
			429, 429, 429, 429, 429, 429, 429, 429, 429, 963, 457, 457, 457, 457, 457, 457, 457, 457, 457, 457, 483, 974, 990, 1006, 
			547, 1022, 1038, 1054, 1070, 1086, 1102, 675, 1118, 1131, 1144, 727, 1157, 1170, 1183, 1196, 1209, 1222};
	static dActionEntry actionTable[] = {
			dActionEntry (59, 0, 0, 6, 0, 0), dActionEntry (268, 0, 0, 8, 0, 0), dActionEntry (290, 0, 0, 10, 0, 0), dActionEntry (44, 0, 0, 14, 0, 0), 
			dActionEntry (61, 0, 0, 13, 0, 0), dActionEntry (254, 0, 1, 12, 1, 41), dActionEntry (44, 0, 1, 4, 1, 24), dActionEntry (61, 0, 1, 4, 1, 24), 
			dActionEntry (59, 0, 0, 6, 0, 0), dActionEntry (254, 0, 1, 11, 1, 39), dActionEntry (268, 0, 0, 8, 0, 0), dActionEntry (273, 0, 0, 18, 0, 0), 
			dActionEntry (290, 0, 0, 10, 0, 0), dActionEntry (59, 0, 1, 8, 1, 32), dActionEntry (254, 0, 1, 8, 1, 32), dActionEntry (268, 0, 1, 8, 1, 32), 
			dActionEntry (273, 0, 1, 8, 1, 32), dActionEntry (290, 0, 1, 8, 1, 32), dActionEntry (59, 0, 1, 8, 1, 30), dActionEntry (254, 0, 1, 8, 1, 30), 
			dActionEntry (268, 0, 1, 8, 1, 30), dActionEntry (273, 0, 1, 8, 1, 30), dActionEntry (290, 0, 1, 8, 1, 30), dActionEntry (59, 0, 1, 9, 1, 33), 
			dActionEntry (254, 0, 1, 9, 1, 33), dActionEntry (268, 0, 1, 9, 1, 33), dActionEntry (273, 0, 1, 9, 1, 33), dActionEntry (290, 0, 1, 9, 1, 33), 
			dActionEntry (290, 0, 0, 19, 0, 0), dActionEntry (59, 0, 1, 8, 1, 31), dActionEntry (254, 0, 1, 8, 1, 31), dActionEntry (268, 0, 1, 8, 1, 31), 
			dActionEntry (273, 0, 1, 8, 1, 31), dActionEntry (290, 0, 1, 8, 1, 31), dActionEntry (44, 0, 1, 3, 1, 23), dActionEntry (61, 0, 1, 3, 1, 23), 
			dActionEntry (59, 0, 1, 7, 1, 28), dActionEntry (61, 0, 0, 21, 0, 0), dActionEntry (254, 0, 1, 7, 1, 28), dActionEntry (268, 0, 1, 7, 1, 28), 
			dActionEntry (273, 0, 1, 7, 1, 28), dActionEntry (290, 0, 1, 7, 1, 28), dActionEntry (254, 0, 2, 0, 0, 0), dActionEntry (40, 0, 0, 22, 0, 0), 
			dActionEntry (262, 0, 0, 24, 0, 0), dActionEntry (269, 0, 0, 27, 0, 0), dActionEntry (275, 0, 0, 23, 0, 0), dActionEntry (288, 0, 0, 29, 0, 0), 
			dActionEntry (289, 0, 0, 31, 0, 0), dActionEntry (290, 0, 0, 30, 0, 0), dActionEntry (291, 0, 0, 28, 0, 0), dActionEntry (290, 0, 0, 10, 0, 0), 
			dActionEntry (254, 0, 1, 11, 2, 40), dActionEntry (59, 0, 1, 9, 2, 34), dActionEntry (254, 0, 1, 9, 2, 34), dActionEntry (268, 0, 1, 9, 2, 34), 
			dActionEntry (273, 0, 1, 9, 2, 34), dActionEntry (290, 0, 1, 9, 2, 34), dActionEntry (40, 0, 0, 33, 0, 0), dActionEntry (59, 0, 0, 39, 0, 0), 
			dActionEntry (254, 0, 1, 10, 1, 35), dActionEntry (262, 0, 0, 35, 0, 0), dActionEntry (269, 0, 0, 38, 0, 0), dActionEntry (275, 0, 0, 34, 0, 0), 
			dActionEntry (288, 0, 0, 41, 0, 0), dActionEntry (289, 0, 0, 43, 0, 0), dActionEntry (290, 0, 0, 42, 0, 0), dActionEntry (291, 0, 0, 40, 0, 0), 
			dActionEntry (44, 0, 1, 2, 1, 21), dActionEntry (59, 0, 1, 2, 1, 21), dActionEntry (61, 0, 1, 2, 1, 21), dActionEntry (254, 0, 1, 2, 1, 21), 
			dActionEntry (268, 0, 1, 2, 1, 21), dActionEntry (273, 0, 1, 2, 1, 21), dActionEntry (290, 0, 1, 2, 1, 21), dActionEntry (44, 0, 0, 44, 0, 0), 
			dActionEntry (59, 0, 1, 6, 2, 27), dActionEntry (61, 0, 1, 6, 2, 27), dActionEntry (254, 0, 1, 6, 2, 27), dActionEntry (268, 0, 1, 6, 2, 27), 
			dActionEntry (273, 0, 1, 6, 2, 27), dActionEntry (290, 0, 1, 6, 2, 27), dActionEntry (40, 0, 0, 46, 0, 0), dActionEntry (262, 0, 0, 48, 0, 0), 
			dActionEntry (269, 0, 0, 50, 0, 0), dActionEntry (275, 0, 0, 47, 0, 0), dActionEntry (288, 0, 0, 52, 0, 0), dActionEntry (289, 0, 0, 54, 0, 0), 
			dActionEntry (290, 0, 0, 53, 0, 0), dActionEntry (291, 0, 0, 51, 0, 0), dActionEntry (37, 0, 1, 0, 1, 13), dActionEntry (42, 0, 1, 0, 1, 13), 
			dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (44, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), 
			dActionEntry (59, 0, 1, 0, 1, 13), dActionEntry (60, 0, 1, 0, 1, 13), dActionEntry (62, 0, 1, 0, 1, 13), dActionEntry (94, 0, 1, 0, 1, 13), 
			dActionEntry (254, 0, 1, 0, 1, 13), dActionEntry (268, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (273, 0, 1, 0, 1, 13), 
			dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (290, 0, 1, 0, 1, 13), dActionEntry (37, 0, 1, 0, 1, 14), dActionEntry (42, 0, 1, 0, 1, 14), 
			dActionEntry (43, 0, 1, 0, 1, 14), dActionEntry (44, 0, 1, 0, 1, 14), dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), 
			dActionEntry (59, 0, 1, 0, 1, 14), dActionEntry (60, 0, 1, 0, 1, 14), dActionEntry (62, 0, 1, 0, 1, 14), dActionEntry (94, 0, 1, 0, 1, 14), 
			dActionEntry (254, 0, 1, 0, 1, 14), dActionEntry (268, 0, 1, 0, 1, 14), dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (273, 0, 1, 0, 1, 14), 
			dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (290, 0, 1, 0, 1, 14), dActionEntry (37, 0, 0, 62, 0, 0), dActionEntry (42, 0, 0, 56, 0, 0), 
			dActionEntry (43, 0, 0, 57, 0, 0), dActionEntry (44, 0, 1, 1, 1, 19), dActionEntry (45, 0, 0, 60, 0, 0), dActionEntry (47, 0, 0, 55, 0, 0), 
			dActionEntry (59, 0, 1, 1, 1, 19), dActionEntry (60, 0, 0, 63, 0, 0), dActionEntry (62, 0, 0, 61, 0, 0), dActionEntry (94, 0, 0, 58, 0, 0), 
			dActionEntry (254, 0, 1, 1, 1, 19), dActionEntry (268, 0, 1, 1, 1, 19), dActionEntry (271, 0, 0, 59, 0, 0), dActionEntry (273, 0, 1, 1, 1, 19), 
			dActionEntry (281, 0, 0, 64, 0, 0), dActionEntry (290, 0, 1, 1, 1, 19), dActionEntry (44, 0, 0, 65, 0, 0), dActionEntry (59, 0, 1, 5, 3, 26), 
			dActionEntry (254, 0, 1, 5, 3, 26), dActionEntry (268, 0, 1, 5, 3, 26), dActionEntry (273, 0, 1, 5, 3, 26), dActionEntry (290, 0, 1, 5, 3, 26), 
			dActionEntry (37, 0, 1, 0, 1, 12), dActionEntry (42, 0, 1, 0, 1, 12), dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (44, 0, 1, 0, 1, 12), 
			dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), dActionEntry (59, 0, 1, 0, 1, 12), dActionEntry (60, 0, 1, 0, 1, 12), 
			dActionEntry (62, 0, 1, 0, 1, 12), dActionEntry (94, 0, 1, 0, 1, 12), dActionEntry (254, 0, 1, 0, 1, 12), dActionEntry (268, 0, 1, 0, 1, 12), 
			dActionEntry (271, 0, 1, 0, 1, 12), dActionEntry (273, 0, 1, 0, 1, 12), dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (290, 0, 1, 0, 1, 12), 
			dActionEntry (37, 0, 1, 0, 1, 17), dActionEntry (42, 0, 1, 0, 1, 17), dActionEntry (43, 0, 1, 0, 1, 17), dActionEntry (44, 0, 1, 0, 1, 17), 
			dActionEntry (45, 0, 1, 0, 1, 17), dActionEntry (47, 0, 1, 0, 1, 17), dActionEntry (59, 0, 1, 0, 1, 17), dActionEntry (60, 0, 1, 0, 1, 17), 
			dActionEntry (62, 0, 1, 0, 1, 17), dActionEntry (94, 0, 1, 0, 1, 17), dActionEntry (254, 0, 1, 0, 1, 17), dActionEntry (268, 0, 1, 0, 1, 17), 
			dActionEntry (271, 0, 1, 0, 1, 17), dActionEntry (273, 0, 1, 0, 1, 17), dActionEntry (281, 0, 1, 0, 1, 17), dActionEntry (290, 0, 1, 0, 1, 17), 
			dActionEntry (37, 0, 1, 0, 1, 18), dActionEntry (42, 0, 1, 0, 1, 18), dActionEntry (43, 0, 1, 0, 1, 18), dActionEntry (44, 0, 1, 0, 1, 18), 
			dActionEntry (45, 0, 1, 0, 1, 18), dActionEntry (47, 0, 1, 0, 1, 18), dActionEntry (59, 0, 1, 0, 1, 18), dActionEntry (60, 0, 1, 0, 1, 18), 
			dActionEntry (62, 0, 1, 0, 1, 18), dActionEntry (94, 0, 1, 0, 1, 18), dActionEntry (254, 0, 1, 0, 1, 18), dActionEntry (268, 0, 1, 0, 1, 18), 
			dActionEntry (271, 0, 1, 0, 1, 18), dActionEntry (273, 0, 1, 0, 1, 18), dActionEntry (281, 0, 1, 0, 1, 18), dActionEntry (290, 0, 1, 0, 1, 18), 
			dActionEntry (37, 0, 1, 0, 1, 16), dActionEntry (42, 0, 1, 0, 1, 16), dActionEntry (43, 0, 1, 0, 1, 16), dActionEntry (44, 0, 1, 0, 1, 16), 
			dActionEntry (45, 0, 1, 0, 1, 16), dActionEntry (47, 0, 1, 0, 1, 16), dActionEntry (59, 0, 1, 0, 1, 16), dActionEntry (60, 0, 1, 0, 1, 16), 
			dActionEntry (62, 0, 1, 0, 1, 16), dActionEntry (94, 0, 1, 0, 1, 16), dActionEntry (254, 0, 1, 0, 1, 16), dActionEntry (268, 0, 1, 0, 1, 16), 
			dActionEntry (271, 0, 1, 0, 1, 16), dActionEntry (273, 0, 1, 0, 1, 16), dActionEntry (281, 0, 1, 0, 1, 16), dActionEntry (290, 0, 1, 0, 1, 16), 
			dActionEntry (37, 0, 1, 0, 1, 15), dActionEntry (42, 0, 1, 0, 1, 15), dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (44, 0, 1, 0, 1, 15), 
			dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), dActionEntry (59, 0, 1, 0, 1, 15), dActionEntry (60, 0, 1, 0, 1, 15), 
			dActionEntry (62, 0, 1, 0, 1, 15), dActionEntry (94, 0, 1, 0, 1, 15), dActionEntry (254, 0, 1, 0, 1, 15), dActionEntry (268, 0, 1, 0, 1, 15), 
			dActionEntry (271, 0, 1, 0, 1, 15), dActionEntry (273, 0, 1, 0, 1, 15), dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (290, 0, 1, 0, 1, 15), 
			dActionEntry (44, 0, 1, 4, 3, 25), dActionEntry (61, 0, 1, 4, 3, 25), dActionEntry (37, 0, 1, 0, 1, 13), dActionEntry (42, 0, 1, 0, 1, 13), 
			dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (44, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), 
			dActionEntry (59, 0, 1, 0, 1, 13), dActionEntry (60, 0, 1, 0, 1, 13), dActionEntry (62, 0, 1, 0, 1, 13), dActionEntry (94, 0, 1, 0, 1, 13), 
			dActionEntry (254, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (281, 0, 1, 0, 1, 13), dActionEntry (37, 0, 1, 0, 1, 14), 
			dActionEntry (42, 0, 1, 0, 1, 14), dActionEntry (43, 0, 1, 0, 1, 14), dActionEntry (44, 0, 1, 0, 1, 14), dActionEntry (45, 0, 1, 0, 1, 14), 
			dActionEntry (47, 0, 1, 0, 1, 14), dActionEntry (59, 0, 1, 0, 1, 14), dActionEntry (60, 0, 1, 0, 1, 14), dActionEntry (62, 0, 1, 0, 1, 14), 
			dActionEntry (94, 0, 1, 0, 1, 14), dActionEntry (254, 0, 1, 0, 1, 14), dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), 
			dActionEntry (37, 0, 0, 74, 0, 0), dActionEntry (42, 0, 0, 68, 0, 0), dActionEntry (43, 0, 0, 69, 0, 0), dActionEntry (44, 0, 1, 1, 1, 19), 
			dActionEntry (45, 0, 0, 72, 0, 0), dActionEntry (47, 0, 0, 67, 0, 0), dActionEntry (59, 0, 1, 1, 1, 19), dActionEntry (60, 0, 0, 75, 0, 0), 
			dActionEntry (62, 0, 0, 73, 0, 0), dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 1, 1, 19), dActionEntry (271, 0, 0, 71, 0, 0), 
			dActionEntry (281, 0, 0, 76, 0, 0), dActionEntry (44, 0, 0, 78, 0, 0), dActionEntry (59, 0, 0, 77, 0, 0), dActionEntry (254, 0, 1, 10, 2, 37), 
			dActionEntry (37, 0, 1, 0, 1, 12), dActionEntry (42, 0, 1, 0, 1, 12), dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (44, 0, 1, 0, 1, 12), 
			dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), dActionEntry (59, 0, 1, 0, 1, 12), dActionEntry (60, 0, 1, 0, 1, 12), 
			dActionEntry (62, 0, 1, 0, 1, 12), dActionEntry (94, 0, 1, 0, 1, 12), dActionEntry (254, 0, 1, 0, 1, 12), dActionEntry (271, 0, 1, 0, 1, 12), 
			dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (254, 0, 1, 10, 2, 36), dActionEntry (37, 0, 1, 0, 1, 17), dActionEntry (42, 0, 1, 0, 1, 17), 
			dActionEntry (43, 0, 1, 0, 1, 17), dActionEntry (44, 0, 1, 0, 1, 17), dActionEntry (45, 0, 1, 0, 1, 17), dActionEntry (47, 0, 1, 0, 1, 17), 
			dActionEntry (59, 0, 1, 0, 1, 17), dActionEntry (60, 0, 1, 0, 1, 17), dActionEntry (62, 0, 1, 0, 1, 17), dActionEntry (94, 0, 1, 0, 1, 17), 
			dActionEntry (254, 0, 1, 0, 1, 17), dActionEntry (271, 0, 1, 0, 1, 17), dActionEntry (281, 0, 1, 0, 1, 17), dActionEntry (37, 0, 1, 0, 1, 18), 
			dActionEntry (42, 0, 1, 0, 1, 18), dActionEntry (43, 0, 1, 0, 1, 18), dActionEntry (44, 0, 1, 0, 1, 18), dActionEntry (45, 0, 1, 0, 1, 18), 
			dActionEntry (47, 0, 1, 0, 1, 18), dActionEntry (59, 0, 1, 0, 1, 18), dActionEntry (60, 0, 1, 0, 1, 18), dActionEntry (62, 0, 1, 0, 1, 18), 
			dActionEntry (94, 0, 1, 0, 1, 18), dActionEntry (254, 0, 1, 0, 1, 18), dActionEntry (271, 0, 1, 0, 1, 18), dActionEntry (281, 0, 1, 0, 1, 18), 
			dActionEntry (37, 0, 1, 0, 1, 16), dActionEntry (42, 0, 1, 0, 1, 16), dActionEntry (43, 0, 1, 0, 1, 16), dActionEntry (44, 0, 1, 0, 1, 16), 
			dActionEntry (45, 0, 1, 0, 1, 16), dActionEntry (47, 0, 1, 0, 1, 16), dActionEntry (59, 0, 1, 0, 1, 16), dActionEntry (60, 0, 1, 0, 1, 16), 
			dActionEntry (62, 0, 1, 0, 1, 16), dActionEntry (94, 0, 1, 0, 1, 16), dActionEntry (254, 0, 1, 0, 1, 16), dActionEntry (271, 0, 1, 0, 1, 16), 
			dActionEntry (281, 0, 1, 0, 1, 16), dActionEntry (37, 0, 1, 0, 1, 15), dActionEntry (42, 0, 1, 0, 1, 15), dActionEntry (43, 0, 1, 0, 1, 15), 
			dActionEntry (44, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), dActionEntry (59, 0, 1, 0, 1, 15), 
			dActionEntry (60, 0, 1, 0, 1, 15), dActionEntry (62, 0, 1, 0, 1, 15), dActionEntry (94, 0, 1, 0, 1, 15), dActionEntry (254, 0, 1, 0, 1, 15), 
			dActionEntry (271, 0, 1, 0, 1, 15), dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (290, 0, 0, 79, 0, 0), dActionEntry (44, 0, 0, 65, 0, 0), 
			dActionEntry (59, 0, 1, 7, 3, 29), dActionEntry (254, 0, 1, 7, 3, 29), dActionEntry (268, 0, 1, 7, 3, 29), dActionEntry (273, 0, 1, 7, 3, 29), 
			dActionEntry (290, 0, 1, 7, 3, 29), dActionEntry (37, 0, 1, 0, 1, 13), dActionEntry (41, 0, 1, 0, 1, 13), dActionEntry (42, 0, 1, 0, 1, 13), 
			dActionEntry (43, 0, 1, 0, 1, 13), dActionEntry (45, 0, 1, 0, 1, 13), dActionEntry (47, 0, 1, 0, 1, 13), dActionEntry (60, 0, 1, 0, 1, 13), 
			dActionEntry (62, 0, 1, 0, 1, 13), dActionEntry (94, 0, 1, 0, 1, 13), dActionEntry (271, 0, 1, 0, 1, 13), dActionEntry (281, 0, 1, 0, 1, 13), 
			dActionEntry (37, 0, 1, 0, 1, 14), dActionEntry (41, 0, 1, 0, 1, 14), dActionEntry (42, 0, 1, 0, 1, 14), dActionEntry (43, 0, 1, 0, 1, 14), 
			dActionEntry (45, 0, 1, 0, 1, 14), dActionEntry (47, 0, 1, 0, 1, 14), dActionEntry (60, 0, 1, 0, 1, 14), dActionEntry (62, 0, 1, 0, 1, 14), 
			dActionEntry (94, 0, 1, 0, 1, 14), dActionEntry (271, 0, 1, 0, 1, 14), dActionEntry (281, 0, 1, 0, 1, 14), dActionEntry (37, 0, 0, 88, 0, 0), 
			dActionEntry (41, 0, 0, 90, 0, 0), dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 0, 83, 0, 0), dActionEntry (45, 0, 0, 86, 0, 0), 
			dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 0, 89, 0, 0), dActionEntry (62, 0, 0, 87, 0, 0), dActionEntry (94, 0, 0, 84, 0, 0), 
			dActionEntry (271, 0, 0, 85, 0, 0), dActionEntry (281, 0, 0, 91, 0, 0), dActionEntry (37, 0, 1, 0, 1, 12), dActionEntry (41, 0, 1, 0, 1, 12), 
			dActionEntry (42, 0, 1, 0, 1, 12), dActionEntry (43, 0, 1, 0, 1, 12), dActionEntry (45, 0, 1, 0, 1, 12), dActionEntry (47, 0, 1, 0, 1, 12), 
			dActionEntry (60, 0, 1, 0, 1, 12), dActionEntry (62, 0, 1, 0, 1, 12), dActionEntry (94, 0, 1, 0, 1, 12), dActionEntry (271, 0, 1, 0, 1, 12), 
			dActionEntry (281, 0, 1, 0, 1, 12), dActionEntry (37, 0, 1, 0, 1, 17), dActionEntry (41, 0, 1, 0, 1, 17), dActionEntry (42, 0, 1, 0, 1, 17), 
			dActionEntry (43, 0, 1, 0, 1, 17), dActionEntry (45, 0, 1, 0, 1, 17), dActionEntry (47, 0, 1, 0, 1, 17), dActionEntry (60, 0, 1, 0, 1, 17), 
			dActionEntry (62, 0, 1, 0, 1, 17), dActionEntry (94, 0, 1, 0, 1, 17), dActionEntry (271, 0, 1, 0, 1, 17), dActionEntry (281, 0, 1, 0, 1, 17), 
			dActionEntry (37, 0, 1, 0, 1, 18), dActionEntry (41, 0, 1, 0, 1, 18), dActionEntry (42, 0, 1, 0, 1, 18), dActionEntry (43, 0, 1, 0, 1, 18), 
			dActionEntry (45, 0, 1, 0, 1, 18), dActionEntry (47, 0, 1, 0, 1, 18), dActionEntry (60, 0, 1, 0, 1, 18), dActionEntry (62, 0, 1, 0, 1, 18), 
			dActionEntry (94, 0, 1, 0, 1, 18), dActionEntry (271, 0, 1, 0, 1, 18), dActionEntry (281, 0, 1, 0, 1, 18), dActionEntry (37, 0, 1, 0, 1, 16), 
			dActionEntry (41, 0, 1, 0, 1, 16), dActionEntry (42, 0, 1, 0, 1, 16), dActionEntry (43, 0, 1, 0, 1, 16), dActionEntry (45, 0, 1, 0, 1, 16), 
			dActionEntry (47, 0, 1, 0, 1, 16), dActionEntry (60, 0, 1, 0, 1, 16), dActionEntry (62, 0, 1, 0, 1, 16), dActionEntry (94, 0, 1, 0, 1, 16), 
			dActionEntry (271, 0, 1, 0, 1, 16), dActionEntry (281, 0, 1, 0, 1, 16), dActionEntry (37, 0, 1, 0, 1, 15), dActionEntry (41, 0, 1, 0, 1, 15), 
			dActionEntry (42, 0, 1, 0, 1, 15), dActionEntry (43, 0, 1, 0, 1, 15), dActionEntry (45, 0, 1, 0, 1, 15), dActionEntry (47, 0, 1, 0, 1, 15), 
			dActionEntry (60, 0, 1, 0, 1, 15), dActionEntry (62, 0, 1, 0, 1, 15), dActionEntry (94, 0, 1, 0, 1, 15), dActionEntry (271, 0, 1, 0, 1, 15), 
			dActionEntry (281, 0, 1, 0, 1, 15), dActionEntry (40, 0, 0, 102, 0, 0), dActionEntry (262, 0, 0, 104, 0, 0), dActionEntry (269, 0, 0, 106, 0, 0), 
			dActionEntry (275, 0, 0, 103, 0, 0), dActionEntry (288, 0, 0, 108, 0, 0), dActionEntry (289, 0, 0, 110, 0, 0), dActionEntry (290, 0, 0, 109, 0, 0), 
			dActionEntry (291, 0, 0, 107, 0, 0), dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 0, 111, 0, 0), dActionEntry (42, 0, 0, 82, 0, 0), 
			dActionEntry (43, 0, 0, 83, 0, 0), dActionEntry (45, 0, 0, 86, 0, 0), dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 0, 89, 0, 0), 
			dActionEntry (62, 0, 0, 87, 0, 0), dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 0, 85, 0, 0), dActionEntry (281, 0, 0, 91, 0, 0), 
			dActionEntry (40, 0, 0, 33, 0, 0), dActionEntry (262, 0, 0, 35, 0, 0), dActionEntry (269, 0, 0, 38, 0, 0), dActionEntry (275, 0, 0, 34, 0, 0), 
			dActionEntry (288, 0, 0, 41, 0, 0), dActionEntry (289, 0, 0, 43, 0, 0), dActionEntry (290, 0, 0, 42, 0, 0), dActionEntry (291, 0, 0, 40, 0, 0), 
			dActionEntry (254, 0, 1, 10, 3, 38), dActionEntry (40, 0, 0, 122, 0, 0), dActionEntry (262, 0, 0, 124, 0, 0), dActionEntry (269, 0, 0, 126, 0, 0), 
			dActionEntry (275, 0, 0, 123, 0, 0), dActionEntry (288, 0, 0, 128, 0, 0), dActionEntry (289, 0, 0, 130, 0, 0), dActionEntry (290, 0, 0, 129, 0, 0), 
			dActionEntry (291, 0, 0, 127, 0, 0), dActionEntry (44, 0, 1, 2, 3, 22), dActionEntry (59, 0, 1, 2, 3, 22), dActionEntry (61, 0, 1, 2, 3, 22), 
			dActionEntry (254, 0, 1, 2, 3, 22), dActionEntry (268, 0, 1, 2, 3, 22), dActionEntry (273, 0, 1, 2, 3, 22), dActionEntry (290, 0, 1, 2, 3, 22), 
			dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 0, 131, 0, 0), dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 0, 83, 0, 0), 
			dActionEntry (45, 0, 0, 86, 0, 0), dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 0, 89, 0, 0), dActionEntry (62, 0, 0, 87, 0, 0), 
			dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 0, 85, 0, 0), dActionEntry (281, 0, 0, 91, 0, 0), dActionEntry (37, 0, 1, 0, 3, 11), 
			dActionEntry (42, 0, 1, 0, 3, 11), dActionEntry (43, 0, 1, 0, 3, 11), dActionEntry (44, 0, 1, 0, 3, 11), dActionEntry (45, 0, 1, 0, 3, 11), 
			dActionEntry (47, 0, 1, 0, 3, 11), dActionEntry (59, 0, 1, 0, 3, 11), dActionEntry (60, 0, 1, 0, 3, 11), dActionEntry (62, 0, 1, 0, 3, 11), 
			dActionEntry (94, 0, 1, 0, 3, 11), dActionEntry (254, 0, 1, 0, 3, 11), dActionEntry (268, 0, 1, 0, 3, 11), dActionEntry (271, 0, 1, 0, 3, 11), 
			dActionEntry (273, 0, 1, 0, 3, 11), dActionEntry (281, 0, 1, 0, 3, 11), dActionEntry (290, 0, 1, 0, 3, 11), dActionEntry (37, 0, 1, 0, 3, 4), 
			dActionEntry (42, 0, 1, 0, 3, 4), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), 
			dActionEntry (47, 0, 1, 0, 3, 4), dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (60, 0, 1, 0, 3, 4), dActionEntry (62, 0, 1, 0, 3, 4), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (268, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), 
			dActionEntry (273, 0, 1, 0, 3, 4), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (290, 0, 1, 0, 3, 4), dActionEntry (37, 0, 1, 0, 3, 3), 
			dActionEntry (42, 0, 1, 0, 3, 3), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 1, 0, 3, 3), dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (60, 0, 1, 0, 3, 3), dActionEntry (62, 0, 1, 0, 3, 3), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (268, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), 
			dActionEntry (273, 0, 1, 0, 3, 3), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (290, 0, 1, 0, 3, 3), dActionEntry (37, 0, 0, 62, 0, 0), 
			dActionEntry (42, 0, 0, 56, 0, 0), dActionEntry (43, 0, 1, 0, 3, 1), dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 1, 0, 3, 1), 
			dActionEntry (47, 0, 0, 55, 0, 0), dActionEntry (59, 0, 1, 0, 3, 1), dActionEntry (60, 0, 1, 0, 3, 1), dActionEntry (62, 0, 1, 0, 3, 1), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (268, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), 
			dActionEntry (273, 0, 1, 0, 3, 1), dActionEntry (281, 0, 1, 0, 3, 1), dActionEntry (290, 0, 1, 0, 3, 1), dActionEntry (37, 0, 1, 0, 3, 6), 
			dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (44, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), 
			dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (59, 0, 1, 0, 3, 6), dActionEntry (60, 0, 1, 0, 3, 6), dActionEntry (62, 0, 1, 0, 3, 6), 
			dActionEntry (94, 0, 1, 0, 3, 6), dActionEntry (254, 0, 1, 0, 3, 6), dActionEntry (268, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), 
			dActionEntry (273, 0, 1, 0, 3, 6), dActionEntry (281, 0, 1, 0, 3, 6), dActionEntry (290, 0, 1, 0, 3, 6), dActionEntry (37, 0, 0, 62, 0, 0), 
			dActionEntry (42, 0, 0, 56, 0, 0), dActionEntry (43, 0, 0, 57, 0, 0), dActionEntry (44, 0, 1, 0, 3, 10), dActionEntry (45, 0, 0, 60, 0, 0), 
			dActionEntry (47, 0, 0, 55, 0, 0), dActionEntry (59, 0, 1, 0, 3, 10), dActionEntry (60, 0, 0, 63, 0, 0), dActionEntry (62, 0, 0, 61, 0, 0), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 10), dActionEntry (268, 0, 1, 0, 3, 10), dActionEntry (271, 0, 1, 0, 3, 10), 
			dActionEntry (273, 0, 1, 0, 3, 10), dActionEntry (281, 0, 0, 64, 0, 0), dActionEntry (290, 0, 1, 0, 3, 10), dActionEntry (37, 0, 0, 62, 0, 0), 
			dActionEntry (42, 0, 0, 56, 0, 0), dActionEntry (43, 0, 1, 0, 3, 2), dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 1, 0, 3, 2), 
			dActionEntry (47, 0, 0, 55, 0, 0), dActionEntry (59, 0, 1, 0, 3, 2), dActionEntry (60, 0, 1, 0, 3, 2), dActionEntry (62, 0, 1, 0, 3, 2), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (268, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), 
			dActionEntry (273, 0, 1, 0, 3, 2), dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (290, 0, 1, 0, 3, 2), dActionEntry (37, 0, 0, 62, 0, 0), 
			dActionEntry (42, 0, 0, 56, 0, 0), dActionEntry (43, 0, 0, 57, 0, 0), dActionEntry (44, 0, 1, 0, 3, 7), dActionEntry (45, 0, 0, 60, 0, 0), 
			dActionEntry (47, 0, 0, 55, 0, 0), dActionEntry (59, 0, 1, 0, 3, 7), dActionEntry (60, 0, 1, 0, 3, 7), dActionEntry (62, 0, 1, 0, 3, 7), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (268, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), 
			dActionEntry (273, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (290, 0, 1, 0, 3, 7), dActionEntry (37, 0, 1, 0, 3, 5), 
			dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (44, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), 
			dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (59, 0, 1, 0, 3, 5), dActionEntry (60, 0, 1, 0, 3, 5), dActionEntry (62, 0, 1, 0, 3, 5), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (268, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), 
			dActionEntry (273, 0, 1, 0, 3, 5), dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (290, 0, 1, 0, 3, 5), dActionEntry (37, 0, 0, 62, 0, 0), 
			dActionEntry (42, 0, 0, 56, 0, 0), dActionEntry (43, 0, 0, 57, 0, 0), dActionEntry (44, 0, 1, 0, 3, 8), dActionEntry (45, 0, 0, 60, 0, 0), 
			dActionEntry (47, 0, 0, 55, 0, 0), dActionEntry (59, 0, 1, 0, 3, 8), dActionEntry (60, 0, 1, 0, 3, 8), dActionEntry (62, 0, 1, 0, 3, 8), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 8), dActionEntry (268, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), 
			dActionEntry (273, 0, 1, 0, 3, 8), dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (290, 0, 1, 0, 3, 8), dActionEntry (37, 0, 0, 62, 0, 0), 
			dActionEntry (42, 0, 0, 56, 0, 0), dActionEntry (43, 0, 0, 57, 0, 0), dActionEntry (44, 0, 1, 0, 3, 9), dActionEntry (45, 0, 0, 60, 0, 0), 
			dActionEntry (47, 0, 0, 55, 0, 0), dActionEntry (59, 0, 1, 0, 3, 9), dActionEntry (60, 0, 1, 0, 3, 9), dActionEntry (62, 0, 1, 0, 3, 9), 
			dActionEntry (94, 0, 0, 58, 0, 0), dActionEntry (254, 0, 1, 0, 3, 9), dActionEntry (268, 0, 1, 0, 3, 9), dActionEntry (271, 0, 1, 0, 3, 9), 
			dActionEntry (273, 0, 1, 0, 3, 9), dActionEntry (281, 0, 1, 0, 3, 9), dActionEntry (290, 0, 1, 0, 3, 9), dActionEntry (37, 0, 0, 150, 0, 0), 
			dActionEntry (42, 0, 0, 144, 0, 0), dActionEntry (43, 0, 0, 145, 0, 0), dActionEntry (44, 0, 1, 1, 3, 20), dActionEntry (45, 0, 0, 148, 0, 0), 
			dActionEntry (47, 0, 0, 143, 0, 0), dActionEntry (59, 0, 1, 1, 3, 20), dActionEntry (60, 0, 0, 151, 0, 0), dActionEntry (62, 0, 0, 149, 0, 0), 
			dActionEntry (94, 0, 0, 146, 0, 0), dActionEntry (254, 0, 1, 1, 3, 20), dActionEntry (268, 0, 1, 1, 3, 20), dActionEntry (271, 0, 0, 147, 0, 0), 
			dActionEntry (273, 0, 1, 1, 3, 20), dActionEntry (281, 0, 0, 152, 0, 0), dActionEntry (290, 0, 1, 1, 3, 20), dActionEntry (37, 0, 1, 0, 3, 11), 
			dActionEntry (42, 0, 1, 0, 3, 11), dActionEntry (43, 0, 1, 0, 3, 11), dActionEntry (44, 0, 1, 0, 3, 11), dActionEntry (45, 0, 1, 0, 3, 11), 
			dActionEntry (47, 0, 1, 0, 3, 11), dActionEntry (59, 0, 1, 0, 3, 11), dActionEntry (60, 0, 1, 0, 3, 11), dActionEntry (62, 0, 1, 0, 3, 11), 
			dActionEntry (94, 0, 1, 0, 3, 11), dActionEntry (254, 0, 1, 0, 3, 11), dActionEntry (271, 0, 1, 0, 3, 11), dActionEntry (281, 0, 1, 0, 3, 11), 
			dActionEntry (37, 0, 1, 0, 3, 4), dActionEntry (42, 0, 1, 0, 3, 4), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), 
			dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 1, 0, 3, 4), dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (60, 0, 1, 0, 3, 4), 
			dActionEntry (62, 0, 1, 0, 3, 4), dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), 
			dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (37, 0, 1, 0, 3, 3), dActionEntry (42, 0, 1, 0, 3, 3), dActionEntry (43, 0, 1, 0, 3, 3), 
			dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 1, 0, 3, 3), dActionEntry (59, 0, 1, 0, 3, 3), 
			dActionEntry (60, 0, 1, 0, 3, 3), dActionEntry (62, 0, 1, 0, 3, 3), dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 0, 3, 3), 
			dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (37, 0, 0, 74, 0, 0), dActionEntry (42, 0, 0, 68, 0, 0), 
			dActionEntry (43, 0, 1, 0, 3, 1), dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 1, 0, 3, 1), dActionEntry (47, 0, 0, 67, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 1), dActionEntry (60, 0, 1, 0, 3, 1), dActionEntry (62, 0, 1, 0, 3, 1), dActionEntry (94, 0, 0, 70, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (281, 0, 1, 0, 3, 1), dActionEntry (37, 0, 1, 0, 3, 6), 
			dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (44, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), 
			dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (59, 0, 1, 0, 3, 6), dActionEntry (60, 0, 1, 0, 3, 6), dActionEntry (62, 0, 1, 0, 3, 6), 
			dActionEntry (94, 0, 1, 0, 3, 6), dActionEntry (254, 0, 1, 0, 3, 6), dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (281, 0, 1, 0, 3, 6), 
			dActionEntry (37, 0, 0, 74, 0, 0), dActionEntry (42, 0, 0, 68, 0, 0), dActionEntry (43, 0, 0, 69, 0, 0), dActionEntry (44, 0, 1, 0, 3, 10), 
			dActionEntry (45, 0, 0, 72, 0, 0), dActionEntry (47, 0, 0, 67, 0, 0), dActionEntry (59, 0, 1, 0, 3, 10), dActionEntry (60, 0, 0, 75, 0, 0), 
			dActionEntry (62, 0, 0, 73, 0, 0), dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 0, 3, 10), dActionEntry (271, 0, 1, 0, 3, 10), 
			dActionEntry (281, 0, 0, 76, 0, 0), dActionEntry (37, 0, 0, 74, 0, 0), dActionEntry (42, 0, 0, 68, 0, 0), dActionEntry (43, 0, 1, 0, 3, 2), 
			dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 1, 0, 3, 2), dActionEntry (47, 0, 0, 67, 0, 0), dActionEntry (59, 0, 1, 0, 3, 2), 
			dActionEntry (60, 0, 1, 0, 3, 2), dActionEntry (62, 0, 1, 0, 3, 2), dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 0, 3, 2), 
			dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (37, 0, 0, 74, 0, 0), dActionEntry (42, 0, 0, 68, 0, 0), 
			dActionEntry (43, 0, 0, 69, 0, 0), dActionEntry (44, 0, 1, 0, 3, 7), dActionEntry (45, 0, 0, 72, 0, 0), dActionEntry (47, 0, 0, 67, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 7), dActionEntry (60, 0, 1, 0, 3, 7), dActionEntry (62, 0, 1, 0, 3, 7), dActionEntry (94, 0, 0, 70, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (37, 0, 1, 0, 3, 5), 
			dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (44, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), 
			dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (59, 0, 1, 0, 3, 5), dActionEntry (60, 0, 1, 0, 3, 5), dActionEntry (62, 0, 1, 0, 3, 5), 
			dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (281, 0, 1, 0, 3, 5), 
			dActionEntry (37, 0, 0, 74, 0, 0), dActionEntry (42, 0, 0, 68, 0, 0), dActionEntry (43, 0, 0, 69, 0, 0), dActionEntry (44, 0, 1, 0, 3, 8), 
			dActionEntry (45, 0, 0, 72, 0, 0), dActionEntry (47, 0, 0, 67, 0, 0), dActionEntry (59, 0, 1, 0, 3, 8), dActionEntry (60, 0, 1, 0, 3, 8), 
			dActionEntry (62, 0, 1, 0, 3, 8), dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), 
			dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (37, 0, 0, 74, 0, 0), dActionEntry (42, 0, 0, 68, 0, 0), dActionEntry (43, 0, 0, 69, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 9), dActionEntry (45, 0, 0, 72, 0, 0), dActionEntry (47, 0, 0, 67, 0, 0), dActionEntry (59, 0, 1, 0, 3, 9), 
			dActionEntry (60, 0, 1, 0, 3, 9), dActionEntry (62, 0, 1, 0, 3, 9), dActionEntry (94, 0, 0, 70, 0, 0), dActionEntry (254, 0, 1, 0, 3, 9), 
			dActionEntry (271, 0, 1, 0, 3, 9), dActionEntry (281, 0, 1, 0, 3, 9), dActionEntry (37, 0, 0, 161, 0, 0), dActionEntry (42, 0, 0, 155, 0, 0), 
			dActionEntry (43, 0, 0, 156, 0, 0), dActionEntry (44, 0, 1, 1, 3, 20), dActionEntry (45, 0, 0, 159, 0, 0), dActionEntry (47, 0, 0, 154, 0, 0), 
			dActionEntry (59, 0, 1, 1, 3, 20), dActionEntry (60, 0, 0, 162, 0, 0), dActionEntry (62, 0, 0, 160, 0, 0), dActionEntry (94, 0, 0, 157, 0, 0), 
			dActionEntry (254, 0, 1, 1, 3, 20), dActionEntry (271, 0, 0, 158, 0, 0), dActionEntry (281, 0, 0, 163, 0, 0), dActionEntry (37, 0, 1, 0, 3, 11), 
			dActionEntry (41, 0, 1, 0, 3, 11), dActionEntry (42, 0, 1, 0, 3, 11), dActionEntry (43, 0, 1, 0, 3, 11), dActionEntry (45, 0, 1, 0, 3, 11), 
			dActionEntry (47, 0, 1, 0, 3, 11), dActionEntry (60, 0, 1, 0, 3, 11), dActionEntry (62, 0, 1, 0, 3, 11), dActionEntry (94, 0, 1, 0, 3, 11), 
			dActionEntry (271, 0, 1, 0, 3, 11), dActionEntry (281, 0, 1, 0, 3, 11), dActionEntry (37, 0, 1, 0, 3, 4), dActionEntry (41, 0, 1, 0, 3, 4), 
			dActionEntry (42, 0, 1, 0, 3, 4), dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 1, 0, 3, 4), 
			dActionEntry (60, 0, 1, 0, 3, 4), dActionEntry (62, 0, 1, 0, 3, 4), dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 4), 
			dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (37, 0, 1, 0, 3, 3), dActionEntry (41, 0, 1, 0, 3, 3), dActionEntry (42, 0, 1, 0, 3, 3), 
			dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 1, 0, 3, 3), dActionEntry (60, 0, 1, 0, 3, 3), 
			dActionEntry (62, 0, 1, 0, 3, 3), dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (281, 0, 1, 0, 3, 3), 
			dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 1, 0, 3, 1), dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 1, 0, 3, 1), 
			dActionEntry (45, 0, 1, 0, 3, 1), dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 1, 0, 3, 1), dActionEntry (62, 0, 1, 0, 3, 1), 
			dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (281, 0, 1, 0, 3, 1), dActionEntry (37, 0, 1, 0, 3, 6), 
			dActionEntry (41, 0, 1, 0, 3, 6), dActionEntry (42, 0, 1, 0, 3, 6), dActionEntry (43, 0, 1, 0, 3, 6), dActionEntry (45, 0, 1, 0, 3, 6), 
			dActionEntry (47, 0, 1, 0, 3, 6), dActionEntry (60, 0, 1, 0, 3, 6), dActionEntry (62, 0, 1, 0, 3, 6), dActionEntry (94, 0, 1, 0, 3, 6), 
			dActionEntry (271, 0, 1, 0, 3, 6), dActionEntry (281, 0, 1, 0, 3, 6), dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 1, 0, 3, 10), 
			dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 0, 83, 0, 0), dActionEntry (45, 0, 0, 86, 0, 0), dActionEntry (47, 0, 0, 81, 0, 0), 
			dActionEntry (60, 0, 0, 89, 0, 0), dActionEntry (62, 0, 0, 87, 0, 0), dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 10), 
			dActionEntry (281, 0, 0, 91, 0, 0), dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 1, 0, 3, 2), dActionEntry (42, 0, 0, 82, 0, 0), 
			dActionEntry (43, 0, 1, 0, 3, 2), dActionEntry (45, 0, 1, 0, 3, 2), dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 1, 0, 3, 2), 
			dActionEntry (62, 0, 1, 0, 3, 2), dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (281, 0, 1, 0, 3, 2), 
			dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 1, 0, 3, 7), dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 0, 83, 0, 0), 
			dActionEntry (45, 0, 0, 86, 0, 0), dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 1, 0, 3, 7), dActionEntry (62, 0, 1, 0, 3, 7), 
			dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (37, 0, 1, 0, 3, 5), 
			dActionEntry (41, 0, 1, 0, 3, 5), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), 
			dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (60, 0, 1, 0, 3, 5), dActionEntry (62, 0, 1, 0, 3, 5), dActionEntry (94, 0, 0, 84, 0, 0), 
			dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 1, 0, 3, 8), 
			dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 0, 83, 0, 0), dActionEntry (45, 0, 0, 86, 0, 0), dActionEntry (47, 0, 0, 81, 0, 0), 
			dActionEntry (60, 0, 1, 0, 3, 8), dActionEntry (62, 0, 1, 0, 3, 8), dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 8), 
			dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 1, 0, 3, 9), dActionEntry (42, 0, 0, 82, 0, 0), 
			dActionEntry (43, 0, 0, 83, 0, 0), dActionEntry (45, 0, 0, 86, 0, 0), dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 1, 0, 3, 9), 
			dActionEntry (62, 0, 1, 0, 3, 9), dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 1, 0, 3, 9), dActionEntry (281, 0, 1, 0, 3, 9), 
			dActionEntry (37, 0, 0, 88, 0, 0), dActionEntry (41, 0, 0, 164, 0, 0), dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 0, 83, 0, 0), 
			dActionEntry (45, 0, 0, 86, 0, 0), dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 0, 89, 0, 0), dActionEntry (62, 0, 0, 87, 0, 0), 
			dActionEntry (94, 0, 0, 84, 0, 0), dActionEntry (271, 0, 0, 85, 0, 0), dActionEntry (281, 0, 0, 91, 0, 0), dActionEntry (37, 0, 0, 88, 0, 0), 
			dActionEntry (41, 0, 0, 175, 0, 0), dActionEntry (42, 0, 0, 82, 0, 0), dActionEntry (43, 0, 0, 83, 0, 0), dActionEntry (45, 0, 0, 86, 0, 0), 
			dActionEntry (47, 0, 0, 81, 0, 0), dActionEntry (60, 0, 0, 89, 0, 0), dActionEntry (62, 0, 0, 87, 0, 0), dActionEntry (94, 0, 0, 84, 0, 0), 
			dActionEntry (271, 0, 0, 85, 0, 0), dActionEntry (281, 0, 0, 91, 0, 0), dActionEntry (37, 0, 1, 0, 3, 4), dActionEntry (42, 0, 1, 0, 3, 4), 
			dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 1, 0, 3, 4), 
			dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (60, 0, 1, 0, 3, 4), dActionEntry (62, 0, 1, 0, 3, 4), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (268, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (273, 0, 1, 0, 3, 4), 
			dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (290, 0, 1, 0, 3, 4), dActionEntry (37, 0, 1, 0, 3, 3), dActionEntry (42, 0, 1, 0, 3, 3), 
			dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), dActionEntry (47, 0, 1, 0, 3, 3), 
			dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (60, 0, 1, 0, 3, 3), dActionEntry (62, 0, 1, 0, 3, 3), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (268, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (273, 0, 1, 0, 3, 3), 
			dActionEntry (281, 0, 1, 0, 3, 3), dActionEntry (290, 0, 1, 0, 3, 3), dActionEntry (37, 0, 0, 150, 0, 0), dActionEntry (42, 0, 0, 144, 0, 0), 
			dActionEntry (43, 0, 1, 0, 3, 1), dActionEntry (44, 0, 1, 0, 3, 1), dActionEntry (45, 0, 1, 0, 3, 1), dActionEntry (47, 0, 0, 143, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 1), dActionEntry (60, 0, 1, 0, 3, 1), dActionEntry (62, 0, 1, 0, 3, 1), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (268, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), dActionEntry (273, 0, 1, 0, 3, 1), 
			dActionEntry (281, 0, 1, 0, 3, 1), dActionEntry (290, 0, 1, 0, 3, 1), dActionEntry (37, 0, 0, 150, 0, 0), dActionEntry (42, 0, 0, 144, 0, 0), 
			dActionEntry (43, 0, 0, 145, 0, 0), dActionEntry (44, 0, 1, 0, 3, 10), dActionEntry (45, 0, 0, 148, 0, 0), dActionEntry (47, 0, 0, 143, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 10), dActionEntry (60, 0, 0, 151, 0, 0), dActionEntry (62, 0, 0, 149, 0, 0), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 10), dActionEntry (268, 0, 1, 0, 3, 10), dActionEntry (271, 0, 1, 0, 3, 10), dActionEntry (273, 0, 1, 0, 3, 10), 
			dActionEntry (281, 0, 0, 152, 0, 0), dActionEntry (290, 0, 1, 0, 3, 10), dActionEntry (37, 0, 0, 150, 0, 0), dActionEntry (42, 0, 0, 144, 0, 0), 
			dActionEntry (43, 0, 1, 0, 3, 2), dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 1, 0, 3, 2), dActionEntry (47, 0, 0, 143, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 2), dActionEntry (60, 0, 1, 0, 3, 2), dActionEntry (62, 0, 1, 0, 3, 2), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (268, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (273, 0, 1, 0, 3, 2), 
			dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (290, 0, 1, 0, 3, 2), dActionEntry (37, 0, 0, 150, 0, 0), dActionEntry (42, 0, 0, 144, 0, 0), 
			dActionEntry (43, 0, 0, 145, 0, 0), dActionEntry (44, 0, 1, 0, 3, 7), dActionEntry (45, 0, 0, 148, 0, 0), dActionEntry (47, 0, 0, 143, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 7), dActionEntry (60, 0, 1, 0, 3, 7), dActionEntry (62, 0, 1, 0, 3, 7), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (268, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (273, 0, 1, 0, 3, 7), 
			dActionEntry (281, 0, 1, 0, 3, 7), dActionEntry (290, 0, 1, 0, 3, 7), dActionEntry (37, 0, 1, 0, 3, 5), dActionEntry (42, 0, 1, 0, 3, 5), 
			dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (44, 0, 1, 0, 3, 5), dActionEntry (45, 0, 1, 0, 3, 5), dActionEntry (47, 0, 1, 0, 3, 5), 
			dActionEntry (59, 0, 1, 0, 3, 5), dActionEntry (60, 0, 1, 0, 3, 5), dActionEntry (62, 0, 1, 0, 3, 5), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (268, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), dActionEntry (273, 0, 1, 0, 3, 5), 
			dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (290, 0, 1, 0, 3, 5), dActionEntry (37, 0, 0, 150, 0, 0), dActionEntry (42, 0, 0, 144, 0, 0), 
			dActionEntry (43, 0, 0, 145, 0, 0), dActionEntry (44, 0, 1, 0, 3, 8), dActionEntry (45, 0, 0, 148, 0, 0), dActionEntry (47, 0, 0, 143, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 8), dActionEntry (60, 0, 1, 0, 3, 8), dActionEntry (62, 0, 1, 0, 3, 8), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 8), dActionEntry (268, 0, 1, 0, 3, 8), dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (273, 0, 1, 0, 3, 8), 
			dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (290, 0, 1, 0, 3, 8), dActionEntry (37, 0, 0, 150, 0, 0), dActionEntry (42, 0, 0, 144, 0, 0), 
			dActionEntry (43, 0, 0, 145, 0, 0), dActionEntry (44, 0, 1, 0, 3, 9), dActionEntry (45, 0, 0, 148, 0, 0), dActionEntry (47, 0, 0, 143, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 9), dActionEntry (60, 0, 1, 0, 3, 9), dActionEntry (62, 0, 1, 0, 3, 9), dActionEntry (94, 0, 0, 146, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 9), dActionEntry (268, 0, 1, 0, 3, 9), dActionEntry (271, 0, 1, 0, 3, 9), dActionEntry (273, 0, 1, 0, 3, 9), 
			dActionEntry (281, 0, 1, 0, 3, 9), dActionEntry (290, 0, 1, 0, 3, 9), dActionEntry (37, 0, 1, 0, 3, 4), dActionEntry (42, 0, 1, 0, 3, 4), 
			dActionEntry (43, 0, 1, 0, 3, 4), dActionEntry (44, 0, 1, 0, 3, 4), dActionEntry (45, 0, 1, 0, 3, 4), dActionEntry (47, 0, 1, 0, 3, 4), 
			dActionEntry (59, 0, 1, 0, 3, 4), dActionEntry (60, 0, 1, 0, 3, 4), dActionEntry (62, 0, 1, 0, 3, 4), dActionEntry (94, 0, 0, 157, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 4), dActionEntry (271, 0, 1, 0, 3, 4), dActionEntry (281, 0, 1, 0, 3, 4), dActionEntry (37, 0, 1, 0, 3, 3), 
			dActionEntry (42, 0, 1, 0, 3, 3), dActionEntry (43, 0, 1, 0, 3, 3), dActionEntry (44, 0, 1, 0, 3, 3), dActionEntry (45, 0, 1, 0, 3, 3), 
			dActionEntry (47, 0, 1, 0, 3, 3), dActionEntry (59, 0, 1, 0, 3, 3), dActionEntry (60, 0, 1, 0, 3, 3), dActionEntry (62, 0, 1, 0, 3, 3), 
			dActionEntry (94, 0, 0, 157, 0, 0), dActionEntry (254, 0, 1, 0, 3, 3), dActionEntry (271, 0, 1, 0, 3, 3), dActionEntry (281, 0, 1, 0, 3, 3), 
			dActionEntry (37, 0, 0, 161, 0, 0), dActionEntry (42, 0, 0, 155, 0, 0), dActionEntry (43, 0, 1, 0, 3, 1), dActionEntry (44, 0, 1, 0, 3, 1), 
			dActionEntry (45, 0, 1, 0, 3, 1), dActionEntry (47, 0, 0, 154, 0, 0), dActionEntry (59, 0, 1, 0, 3, 1), dActionEntry (60, 0, 1, 0, 3, 1), 
			dActionEntry (62, 0, 1, 0, 3, 1), dActionEntry (94, 0, 0, 157, 0, 0), dActionEntry (254, 0, 1, 0, 3, 1), dActionEntry (271, 0, 1, 0, 3, 1), 
			dActionEntry (281, 0, 1, 0, 3, 1), dActionEntry (37, 0, 0, 161, 0, 0), dActionEntry (42, 0, 0, 155, 0, 0), dActionEntry (43, 0, 0, 156, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 10), dActionEntry (45, 0, 0, 159, 0, 0), dActionEntry (47, 0, 0, 154, 0, 0), dActionEntry (59, 0, 1, 0, 3, 10), 
			dActionEntry (60, 0, 0, 162, 0, 0), dActionEntry (62, 0, 0, 160, 0, 0), dActionEntry (94, 0, 0, 157, 0, 0), dActionEntry (254, 0, 1, 0, 3, 10), 
			dActionEntry (271, 0, 1, 0, 3, 10), dActionEntry (281, 0, 0, 163, 0, 0), dActionEntry (37, 0, 0, 161, 0, 0), dActionEntry (42, 0, 0, 155, 0, 0), 
			dActionEntry (43, 0, 1, 0, 3, 2), dActionEntry (44, 0, 1, 0, 3, 2), dActionEntry (45, 0, 1, 0, 3, 2), dActionEntry (47, 0, 0, 154, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 2), dActionEntry (60, 0, 1, 0, 3, 2), dActionEntry (62, 0, 1, 0, 3, 2), dActionEntry (94, 0, 0, 157, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 2), dActionEntry (271, 0, 1, 0, 3, 2), dActionEntry (281, 0, 1, 0, 3, 2), dActionEntry (37, 0, 0, 161, 0, 0), 
			dActionEntry (42, 0, 0, 155, 0, 0), dActionEntry (43, 0, 0, 156, 0, 0), dActionEntry (44, 0, 1, 0, 3, 7), dActionEntry (45, 0, 0, 159, 0, 0), 
			dActionEntry (47, 0, 0, 154, 0, 0), dActionEntry (59, 0, 1, 0, 3, 7), dActionEntry (60, 0, 1, 0, 3, 7), dActionEntry (62, 0, 1, 0, 3, 7), 
			dActionEntry (94, 0, 0, 157, 0, 0), dActionEntry (254, 0, 1, 0, 3, 7), dActionEntry (271, 0, 1, 0, 3, 7), dActionEntry (281, 0, 1, 0, 3, 7), 
			dActionEntry (37, 0, 1, 0, 3, 5), dActionEntry (42, 0, 1, 0, 3, 5), dActionEntry (43, 0, 1, 0, 3, 5), dActionEntry (44, 0, 1, 0, 3, 5), 
			dActionEntry (45, 0, 1, 0, 3, 5), dActionEntry (47, 0, 1, 0, 3, 5), dActionEntry (59, 0, 1, 0, 3, 5), dActionEntry (60, 0, 1, 0, 3, 5), 
			dActionEntry (62, 0, 1, 0, 3, 5), dActionEntry (94, 0, 0, 157, 0, 0), dActionEntry (254, 0, 1, 0, 3, 5), dActionEntry (271, 0, 1, 0, 3, 5), 
			dActionEntry (281, 0, 1, 0, 3, 5), dActionEntry (37, 0, 0, 161, 0, 0), dActionEntry (42, 0, 0, 155, 0, 0), dActionEntry (43, 0, 0, 156, 0, 0), 
			dActionEntry (44, 0, 1, 0, 3, 8), dActionEntry (45, 0, 0, 159, 0, 0), dActionEntry (47, 0, 0, 154, 0, 0), dActionEntry (59, 0, 1, 0, 3, 8), 
			dActionEntry (60, 0, 1, 0, 3, 8), dActionEntry (62, 0, 1, 0, 3, 8), dActionEntry (94, 0, 0, 157, 0, 0), dActionEntry (254, 0, 1, 0, 3, 8), 
			dActionEntry (271, 0, 1, 0, 3, 8), dActionEntry (281, 0, 1, 0, 3, 8), dActionEntry (37, 0, 0, 161, 0, 0), dActionEntry (42, 0, 0, 155, 0, 0), 
			dActionEntry (43, 0, 0, 156, 0, 0), dActionEntry (44, 0, 1, 0, 3, 9), dActionEntry (45, 0, 0, 159, 0, 0), dActionEntry (47, 0, 0, 154, 0, 0), 
			dActionEntry (59, 0, 1, 0, 3, 9), dActionEntry (60, 0, 1, 0, 3, 9), dActionEntry (62, 0, 1, 0, 3, 9), dActionEntry (94, 0, 0, 157, 0, 0), 
			dActionEntry (254, 0, 1, 0, 3, 9), dActionEntry (271, 0, 1, 0, 3, 9), dActionEntry (281, 0, 1, 0, 3, 9)};

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
			9, 0, 0, 0, 7, 0, 0, 0, 1, 0, 0, 0, 0, 2, 1, 0, 0, 0, 2, 0, 0, 2, 1, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 
			0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 
			1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 
			1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static short gotoStart[] = {
			0, 9, 9, 9, 9, 16, 16, 16, 16, 17, 17, 17, 17, 17, 19, 20, 20, 20, 20, 22, 22, 22, 24, 25, 
			25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 
			27, 27, 27, 27, 27, 27, 27, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 38, 39, 40, 41, 42, 
			43, 44, 45, 46, 47, 48, 48, 49, 49, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 58, 59, 59, 59, 59, 
			59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 
			60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 
			62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 81, 81, 81, 
			81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81};
	static dGotoEntry gotoTable[] = {
			dGotoEntry (295, 3), dGotoEntry (296, 1), dGotoEntry (297, 9), dGotoEntry (298, 11), dGotoEntry (299, 5), 
			dGotoEntry (300, 7), dGotoEntry (301, 4), dGotoEntry (303, 2), dGotoEntry (304, 12), dGotoEntry (295, 3), 
			dGotoEntry (296, 15), dGotoEntry (297, 9), dGotoEntry (298, 11), dGotoEntry (299, 5), dGotoEntry (300, 17), 
			dGotoEntry (302, 16), dGotoEntry (294, 20), dGotoEntry (292, 25), dGotoEntry (293, 26), dGotoEntry (295, 32), 
			dGotoEntry (292, 36), dGotoEntry (293, 37), dGotoEntry (292, 25), dGotoEntry (293, 45), dGotoEntry (292, 49), 
			dGotoEntry (292, 66), dGotoEntry (292, 80), dGotoEntry (292, 92), dGotoEntry (292, 93), dGotoEntry (292, 94), 
			dGotoEntry (292, 95), dGotoEntry (292, 96), dGotoEntry (292, 97), dGotoEntry (292, 98), dGotoEntry (292, 99), 
			dGotoEntry (292, 100), dGotoEntry (292, 101), dGotoEntry (292, 105), dGotoEntry (292, 112), dGotoEntry (292, 113), 
			dGotoEntry (292, 114), dGotoEntry (292, 115), dGotoEntry (292, 116), dGotoEntry (292, 117), dGotoEntry (292, 118), 
			dGotoEntry (292, 119), dGotoEntry (292, 120), dGotoEntry (292, 121), dGotoEntry (292, 125), dGotoEntry (292, 132), 
			dGotoEntry (292, 133), dGotoEntry (292, 134), dGotoEntry (292, 135), dGotoEntry (292, 136), dGotoEntry (292, 137), 
			dGotoEntry (292, 138), dGotoEntry (292, 139), dGotoEntry (292, 140), dGotoEntry (292, 141), dGotoEntry (292, 142), 
			dGotoEntry (292, 153), dGotoEntry (292, 165), dGotoEntry (292, 166), dGotoEntry (292, 167), dGotoEntry (292, 168), 
			dGotoEntry (292, 169), dGotoEntry (292, 170), dGotoEntry (292, 171), dGotoEntry (292, 172), dGotoEntry (292, 173), 
			dGotoEntry (292, 174), dGotoEntry (292, 176), dGotoEntry (292, 177), dGotoEntry (292, 178), dGotoEntry (292, 179), 
			dGotoEntry (292, 180), dGotoEntry (292, 181), dGotoEntry (292, 182), dGotoEntry (292, 183), dGotoEntry (292, 184), 
			dGotoEntry (292, 185)};

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
						case 41:// chunk : block 
{MyModule->CloseFunctionDeclaration();}
break;

						case 24:// variableList : variable 
{entry.m_value = parameter[0].m_value;}
break;

						case 23:// variable : _LABEL 
{entry.m_value = parameter[0].m_value;}
break;

						case 28:// local : localDeclaration 
{entry.m_value = parameter[0].m_value;}
break;

						case 35:// returnStatement : _RETURN 
{entry.m_value = MyModule->EmitReturn(dUserVariable());}
break;

						case 21:// namelist : _LABEL 
{entry.m_value = parameter[0].m_value; entry.m_value.m_tokenList.Append (parameter[0].m_value.GetString());}
break;

						case 27:// localDeclaration : _LOCAL namelist 
{entry.m_value = MyModule->EmitLocalVariableDeclaration(parameter[1].m_value);}
break;

						case 13:// expression : _TRUE 
{dAssert(0);}
break;

						case 14:// expression : _FALSE 
{dAssert(0);}
break;

						case 19:// expressionList : expression 
{entry.m_value = parameter[0].m_value;}
break;

						case 26:// assigment : variableList = expressionList 
{entry.m_value = MyModule->EmitAssigmentStatement(parameter[0].m_value, parameter[2].m_value);}
break;

						case 12:// expression : _NIL 
{dAssert(0);}
break;

						case 17:// expression : _STRING 
{dAssert(0);}
break;

						case 18:// expression : _INTEGER 
{entry.m_value = MyModule->EmitLoadConstant(parameter[0].m_value);}
break;

						case 16:// expression : _LABEL 
{entry.m_value = MyModule->EmitLoadVariable(parameter[0].m_value);}
break;

						case 15:// expression : _FLOAT 
{dAssert(0);}
break;

						case 25:// variableList : variableList , variable 
{dAssert(0);}
break;

						case 37:// returnStatement : _RETURN expressionList 
{entry.m_value = MyModule->EmitReturn(parameter[1].m_value);}
break;

						case 36:// returnStatement : _RETURN ; 
{entry.m_value = MyModule->EmitReturn(dUserVariable());}
break;

						case 29:// local : localDeclaration = expressionList 
{entry.m_value = MyModule->EmitAssigmentStatement(parameter[0].m_value, parameter[2].m_value);}
break;

						case 38:// returnStatement : _RETURN expressionList ; 
{entry.m_value = MyModule->EmitReturn(parameter[1].m_value);}
break;

						case 22:// namelist : namelist , _LABEL 
{entry.m_value = parameter[0].m_value; entry.m_value.m_tokenList.Append (parameter[2].m_value.GetString());}
break;

						case 11:// expression : ( expression ) 
{dAssert(0);}
break;

						case 4:// expression : expression / expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 3:// expression : expression * expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 1:// expression : expression + expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 6:// expression : expression ^ expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 10:// expression : expression _OR expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 2:// expression : expression - expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 7:// expression : expression > expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 5:// expression : expression % expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 8:// expression : expression < expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 9:// expression : expression _IDENTICAL expression 
{entry.m_value = MyModule->EmitBinaryExpression(parameter[0].m_value, parameter[1].m_value, parameter[2].m_value);}
break;

						case 20:// expressionList : expressionList , expression 
{entry.m_value = parameter[0].m_value; entry.m_value.m_nodeList.Append (parameter[2].m_value.m_nodeList.GetFirst()->GetInfo());}
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



