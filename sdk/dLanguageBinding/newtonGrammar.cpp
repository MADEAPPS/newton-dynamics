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
// Auto generated Parser Generator class: newtonGrammar.cpp
//

#include <stdafx.h>
#include "lexical.h"
#include "newtonBinding.h"


#include "newtonGrammar.h"
#include <dList.h>

#define MAX_USER_PARAM	64

enum newtonGrammar::ActionType
{
	dSHIFT = 0,
	dREDUCE,
	dACCEPT,
	dERROR
};

class newtonGrammar::dActionEntry
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

class newtonGrammar::dGotoEntry
{
	public:
	dGotoEntry (short token, short nextState)
		:m_token(token), m_nextState(nextState)
	{
	}

	short  m_token;
	short  m_nextState;
};



class newtonGrammar::dStackPair
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


newtonGrammar::newtonGrammar()
{
}

newtonGrammar::~newtonGrammar()
{
}


const newtonGrammar::dActionEntry* newtonGrammar::FindAction (const dActionEntry* const actionList, int count, dToken token) const
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

const newtonGrammar::dGotoEntry* newtonGrammar::FindGoto (const dGotoEntry* const gotoList, int count, dToken token) const
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

	_ASSERT (0);
	return NULL;
}



const newtonGrammar::dActionEntry* newtonGrammar::GetNextAction (dList<dStackPair>& stack, dToken token, lexical& scanner) const
{
	static short actionsCount[] = {
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 3, 3, 3, 3, 3, 1, 1, 3, 4, 
			4, 4, 2, 3, 1, 2, 3, 2, 3, 1, 1, 3, 1, 1, 3, 2, 1, 1, 4, 3, 4, 4, 4, 2, 
			2, 1, 1, 3, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 2, 1, 1, 2, 1, 1, 1, 1, 2, 1, 
			1, 2, 1, 3, 4, 2, 2, 2, 2, 1, 4, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 
			1, 1, 1, 2, 1, 2, 1, 1, 2, 2, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 1, 2, 10, 2, 
			1, 1, 1, 1, 1, 1, 2, 1, 1, 9, 2, 2, 11, 5, 10, 2, 2, 1, 1, 2, 2, 11, 2, 11, 
			1, 3, 2, 2, 2, 1, 1, 1, 1, 10, 10, 1, 9, 1, 2, 2, 5, 2, 2, 1, 2, 2, 2, 1, 
			3, 2, 2, 2, 1, 11, 2, 2, 2, 11, 1, 1, 10, 1, 2, 2, 1, 1, 1, 1, 2, 2, 8, 2, 
			2, 1, 1, 2, 1, 10, 2, 2, 2, 2, 1, 1, 1, 2, 1, 2, 1, 1, 11, 2, 1, 2, 1, 2, 
			1, 4, 1, 1, 1, 1, 1, 1, 2, 1, 1, 9, 9, 1, 3, 2, 2, 2, 1, 2, 1, 1, 1, 11, 
			1, 2, 1, 2, 1, 1, 1, 2, 1, 2, 2, 2, 1, 1, 2, 1, 2, 1, 2, 1};
	static short actionsStart[] = {
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 17, 20, 23, 26, 29, 32, 33, 34, 37, 
			41, 41, 45, 47, 50, 51, 53, 56, 58, 61, 62, 63, 66, 67, 68, 71, 73, 74, 41, 75, 37, 41, 41, 78, 
			80, 82, 83, 84, 37, 41, 41, 87, 90, 93, 95, 97, 99, 100, 101, 103, 104, 105, 107, 108, 109, 110, 111, 113, 
			114, 115, 117, 118, 41, 121, 123, 125, 127, 117, 41, 129, 130, 131, 132, 133, 134, 135, 136, 138, 140, 141, 142, 143, 
			144, 145, 146, 147, 149, 150, 152, 153, 154, 156, 158, 159, 161, 162, 163, 164, 165, 166, 168, 170, 171, 172, 174, 184, 
			186, 187, 188, 189, 190, 191, 192, 194, 195, 196, 205, 207, 209, 220, 174, 225, 227, 229, 230, 231, 233, 235, 246, 248, 
			259, 260, 263, 265, 267, 269, 270, 271, 272, 273, 283, 293, 294, 303, 304, 306, 308, 313, 315, 317, 318, 320, 322, 324, 
			325, 328, 330, 332, 334, 335, 346, 348, 350, 352, 363, 364, 365, 375, 376, 378, 380, 381, 382, 383, 384, 386, 388, 396, 
			398, 400, 401, 402, 404, 405, 415, 417, 419, 421, 423, 424, 425, 426, 428, 429, 431, 432, 433, 444, 446, 447, 449, 450, 
			452, 453, 457, 458, 459, 460, 461, 462, 463, 465, 466, 467, 476, 485, 486, 489, 491, 493, 495, 496, 498, 499, 500, 501, 
			512, 513, 515, 516, 518, 519, 520, 521, 523, 524, 526, 528, 530, 531, 532, 534, 535, 537, 538, 540};
	static dActionEntry actionTable[] = {
			dActionEntry (271, 0, 0, 2, 0, 0), dActionEntry (254, 0, 2, 0, 0, 0), dActionEntry (277, 0, 0, 3, 0, 0), dActionEntry (274, 0, 0, 4, 0, 0), 
			dActionEntry (277, 0, 0, 7, 0, 0), dActionEntry (274, 0, 0, 8, 0, 0), dActionEntry (273, 0, 0, 11, 0, 0), dActionEntry (274, 0, 1, 2, 2, 3), 
			dActionEntry (277, 0, 0, 12, 0, 0), dActionEntry (274, 0, 0, 13, 0, 0), dActionEntry (270, 0, 0, 15, 0, 0), dActionEntry (271, 0, 0, 17, 0, 0), 
			dActionEntry (254, 0, 1, 0, 4, 1), dActionEntry (276, 0, 0, 20, 0, 0), dActionEntry (277, 0, 0, 21, 0, 0), dActionEntry (270, 0, 1, 3, 2, 4), 
			dActionEntry (271, 0, 1, 3, 2, 4), dActionEntry (263, 0, 1, 8, 1, 9), dActionEntry (265, 0, 1, 8, 1, 9), dActionEntry (277, 0, 1, 8, 1, 9), 
			dActionEntry (270, 0, 0, 15, 0, 0), dActionEntry (271, 0, 0, 17, 0, 0), dActionEntry (273, 0, 1, 1, 3, 2), dActionEntry (263, 0, 1, 8, 1, 10), 
			dActionEntry (265, 0, 1, 8, 1, 10), dActionEntry (277, 0, 1, 8, 1, 10), dActionEntry (263, 0, 0, 24, 0, 0), dActionEntry (265, 0, 0, 25, 0, 0), 
			dActionEntry (277, 0, 0, 23, 0, 0), dActionEntry (270, 0, 1, 4, 1, 7), dActionEntry (271, 0, 1, 4, 1, 7), dActionEntry (273, 0, 1, 4, 1, 7), 
			dActionEntry (274, 0, 1, 5, 3, 5), dActionEntry (276, 0, 0, 26, 0, 0), dActionEntry (270, 0, 1, 4, 2, 8), dActionEntry (271, 0, 1, 4, 2, 8), 
			dActionEntry (273, 0, 1, 4, 2, 8), dActionEntry (268, 0, 0, 28, 0, 0), dActionEntry (270, 0, 0, 15, 0, 0), dActionEntry (271, 0, 0, 17, 0, 0), 
			dActionEntry (274, 0, 0, 27, 0, 0), dActionEntry (268, 0, 0, 33, 0, 0), dActionEntry (270, 0, 0, 15, 0, 0), dActionEntry (271, 0, 0, 17, 0, 0), 
			dActionEntry (274, 0, 0, 32, 0, 0), dActionEntry (270, 0, 1, 6, 3, 6), dActionEntry (271, 0, 1, 6, 3, 6), dActionEntry (263, 0, 0, 39, 0, 0), 
			dActionEntry (265, 0, 0, 40, 0, 0), dActionEntry (277, 0, 0, 38, 0, 0), dActionEntry (278, 0, 0, 41, 0, 0), dActionEntry (272, 0, 0, 42, 0, 0), 
			dActionEntry (273, 0, 0, 43, 0, 0), dActionEntry (263, 0, 0, 45, 0, 0), dActionEntry (265, 0, 0, 46, 0, 0), dActionEntry (277, 0, 0, 44, 0, 0), 
			dActionEntry (272, 0, 1, 9, 1, 23), dActionEntry (273, 0, 1, 9, 1, 23), dActionEntry (263, 0, 0, 48, 0, 0), dActionEntry (265, 0, 0, 49, 0, 0), 
			dActionEntry (277, 0, 0, 47, 0, 0), dActionEntry (278, 0, 0, 50, 0, 0), dActionEntry (273, 0, 0, 51, 0, 0), dActionEntry (263, 0, 0, 53, 0, 0), 
			dActionEntry (265, 0, 0, 54, 0, 0), dActionEntry (277, 0, 0, 52, 0, 0), dActionEntry (273, 0, 1, 9, 1, 23), dActionEntry (273, 0, 0, 55, 0, 0), 
			dActionEntry (272, 0, 1, 9, 2, 15), dActionEntry (273, 0, 1, 9, 2, 15), dActionEntry (277, 0, 0, 56, 0, 0), dActionEntry (262, 0, 0, 58, 0, 0), 
			dActionEntry (264, 0, 0, 57, 0, 0), dActionEntry (264, 0, 0, 59, 0, 0), dActionEntry (123, 0, 0, 60, 0, 0), dActionEntry (270, 0, 1, 7, 4, 13), 
			dActionEntry (271, 0, 1, 7, 4, 13), dActionEntry (273, 0, 1, 7, 4, 13), dActionEntry (273, 0, 1, 9, 2, 15), dActionEntry (277, 0, 0, 65, 0, 0), 
			dActionEntry (262, 0, 0, 67, 0, 0), dActionEntry (264, 0, 0, 66, 0, 0), dActionEntry (264, 0, 0, 68, 0, 0), dActionEntry (123, 0, 0, 69, 0, 0), 
			dActionEntry (270, 0, 1, 7, 4, 11), dActionEntry (271, 0, 1, 7, 4, 11), dActionEntry (273, 0, 1, 7, 4, 11), dActionEntry (270, 0, 1, 7, 4, 12), 
			dActionEntry (271, 0, 1, 7, 4, 12), dActionEntry (273, 0, 1, 7, 4, 12), dActionEntry (40, 0, 0, 73, 0, 0), dActionEntry (272, 0, 1, 9, 3, 19), 
			dActionEntry (273, 0, 1, 9, 3, 19), dActionEntry (272, 0, 1, 9, 3, 17), dActionEntry (273, 0, 1, 9, 3, 17), dActionEntry (272, 0, 1, 9, 3, 16), 
			dActionEntry (273, 0, 1, 9, 3, 16), dActionEntry (272, 0, 1, 9, 3, 18), dActionEntry (273, 0, 1, 9, 3, 18), dActionEntry (273, 0, 0, 74, 0, 0), 
			dActionEntry (273, 0, 0, 75, 0, 0), dActionEntry (272, 0, 0, 76, 0, 0), dActionEntry (273, 0, 0, 77, 0, 0), dActionEntry (273, 0, 0, 78, 0, 0), 
			dActionEntry (273, 0, 0, 79, 0, 0), dActionEntry (40, 0, 0, 80, 0, 0), dActionEntry (273, 0, 1, 9, 3, 19), dActionEntry (273, 0, 1, 9, 3, 17), 
			dActionEntry (273, 0, 1, 9, 3, 16), dActionEntry (273, 0, 1, 9, 3, 18), dActionEntry (273, 0, 0, 81, 0, 0), dActionEntry (272, 0, 0, 82, 0, 0), 
			dActionEntry (273, 0, 0, 83, 0, 0), dActionEntry (273, 0, 0, 84, 0, 0), dActionEntry (273, 0, 0, 85, 0, 0), dActionEntry (40, 0, 0, 86, 0, 0), 
			dActionEntry (277, 0, 0, 87, 0, 0), dActionEntry (274, 0, 0, 88, 0, 0), dActionEntry (270, 0, 1, 7, 6, 14), dActionEntry (271, 0, 1, 7, 6, 14), 
			dActionEntry (273, 0, 1, 7, 6, 14), dActionEntry (272, 0, 1, 7, 4, 13), dActionEntry (273, 0, 1, 7, 4, 13), dActionEntry (272, 0, 1, 7, 4, 11), 
			dActionEntry (273, 0, 1, 7, 4, 11), dActionEntry (272, 0, 1, 7, 4, 12), dActionEntry (273, 0, 1, 7, 4, 12), dActionEntry (40, 0, 0, 94, 0, 0), 
			dActionEntry (277, 0, 0, 95, 0, 0), dActionEntry (273, 0, 1, 7, 4, 13), dActionEntry (273, 0, 1, 7, 4, 11), dActionEntry (273, 0, 1, 7, 4, 12), 
			dActionEntry (277, 0, 0, 98, 0, 0), dActionEntry (41, 0, 0, 99, 0, 0), dActionEntry (277, 0, 0, 100, 0, 0), dActionEntry (269, 0, 0, 102, 0, 0), 
			dActionEntry (269, 0, 1, 11, 1, 87), dActionEntry (274, 0, 0, 88, 0, 0), dActionEntry (269, 0, 1, 41, 1, 81), dActionEntry (274, 0, 1, 41, 1, 81), 
			dActionEntry (270, 0, 0, 106, 0, 0), dActionEntry (273, 0, 0, 107, 0, 0), dActionEntry (277, 0, 0, 108, 0, 0), dActionEntry (41, 0, 0, 109, 0, 0), 
			dActionEntry (270, 0, 0, 110, 0, 0), dActionEntry (273, 0, 0, 111, 0, 0), dActionEntry (40, 0, 0, 112, 0, 0), dActionEntry (272, 0, 1, 9, 6, 21), 
			dActionEntry (273, 0, 1, 9, 6, 21), dActionEntry (276, 0, 0, 113, 0, 0), dActionEntry (266, 0, 1, 12, 1, 88), dActionEntry (269, 0, 0, 102, 0, 0), 
			dActionEntry (266, 0, 0, 115, 0, 0), dActionEntry (266, 0, 0, 116, 0, 0), dActionEntry (266, 0, 1, 42, 1, 83), dActionEntry (269, 0, 1, 42, 1, 83), 
			dActionEntry (269, 0, 1, 41, 2, 82), dActionEntry (274, 0, 1, 41, 2, 82), dActionEntry (277, 0, 0, 120, 0, 0), dActionEntry (272, 0, 1, 7, 6, 14), 
			dActionEntry (273, 0, 1, 7, 6, 14), dActionEntry (40, 0, 0, 121, 0, 0), dActionEntry (273, 0, 1, 9, 6, 21), dActionEntry (277, 0, 0, 122, 0, 0), 
			dActionEntry (273, 0, 1, 7, 6, 14), dActionEntry (278, 0, 0, 123, 0, 0), dActionEntry (269, 0, 1, 16, 3, 25), dActionEntry (274, 0, 1, 16, 3, 25), 
			dActionEntry (266, 0, 1, 42, 2, 84), dActionEntry (269, 0, 1, 42, 2, 84), dActionEntry (277, 0, 0, 124, 0, 0), dActionEntry (277, 0, 0, 125, 0, 0), 
			dActionEntry (266, 0, 0, 116, 0, 0), dActionEntry (269, 0, 0, 129, 0, 0), dActionEntry (256, 0, 0, 135, 0, 0), dActionEntry (257, 0, 0, 142, 0, 0), 
			dActionEntry (258, 0, 0, 139, 0, 0), dActionEntry (259, 0, 0, 137, 0, 0), dActionEntry (260, 0, 0, 145, 0, 0), dActionEntry (261, 0, 0, 133, 0, 0), 
			dActionEntry (263, 0, 0, 140, 0, 0), dActionEntry (265, 0, 0, 146, 0, 0), dActionEntry (267, 0, 0, 138, 0, 0), dActionEntry (277, 0, 0, 136, 0, 0), 
			dActionEntry (266, 0, 1, 13, 1, 27), dActionEntry (269, 0, 1, 13, 1, 27), dActionEntry (125, 0, 0, 148, 0, 0), dActionEntry (278, 0, 0, 149, 0, 0), 
			dActionEntry (125, 0, 0, 150, 0, 0), dActionEntry (41, 0, 0, 151, 0, 0), dActionEntry (123, 0, 0, 152, 0, 0), dActionEntry (123, 0, 0, 153, 0, 0), 
			dActionEntry (269, 0, 1, 39, 1, 77), dActionEntry (277, 0, 1, 39, 1, 77), dActionEntry (40, 0, 0, 154, 0, 0), dActionEntry (277, 0, 0, 156, 0, 0), 
			dActionEntry (256, 0, 0, 161, 0, 0), dActionEntry (257, 0, 0, 166, 0, 0), dActionEntry (258, 0, 0, 164, 0, 0), dActionEntry (259, 0, 0, 163, 0, 0), 
			dActionEntry (260, 0, 0, 168, 0, 0), dActionEntry (261, 0, 0, 160, 0, 0), dActionEntry (263, 0, 0, 165, 0, 0), dActionEntry (265, 0, 0, 169, 0, 0), 
			dActionEntry (277, 0, 0, 162, 0, 0), dActionEntry (269, 0, 0, 129, 0, 0), dActionEntry (277, 0, 1, 14, 1, 85), dActionEntry (266, 0, 1, 13, 2, 28), 
			dActionEntry (269, 0, 1, 13, 2, 28), dActionEntry (125, 0, 0, 172, 0, 0), dActionEntry (256, 0, 0, 135, 0, 0), dActionEntry (257, 0, 0, 142, 0, 0), 
			dActionEntry (258, 0, 0, 139, 0, 0), dActionEntry (259, 0, 0, 137, 0, 0), dActionEntry (260, 0, 0, 145, 0, 0), dActionEntry (261, 0, 0, 133, 0, 0), 
			dActionEntry (263, 0, 0, 140, 0, 0), dActionEntry (265, 0, 0, 146, 0, 0), dActionEntry (267, 0, 0, 138, 0, 0), dActionEntry (277, 0, 0, 136, 0, 0), 
			dActionEntry (42, 0, 1, 21, 1, 37), dActionEntry (256, 0, 0, 174, 0, 0), dActionEntry (257, 0, 0, 175, 0, 0), dActionEntry (260, 0, 0, 176, 0, 0), 
			dActionEntry (277, 0, 1, 21, 1, 37), dActionEntry (42, 0, 1, 21, 1, 31), dActionEntry (277, 0, 1, 21, 1, 31), dActionEntry (42, 0, 0, 178, 0, 0), 
			dActionEntry (277, 0, 1, 22, 1, 44), dActionEntry (277, 0, 0, 179, 0, 0), dActionEntry (123, 0, 0, 180, 0, 0), dActionEntry (42, 0, 1, 21, 1, 33), 
			dActionEntry (277, 0, 1, 21, 1, 33), dActionEntry (42, 0, 1, 21, 1, 35), dActionEntry (277, 0, 1, 21, 1, 35), dActionEntry (125, 0, 1, 28, 1, 55), 
			dActionEntry (256, 0, 1, 28, 1, 55), dActionEntry (257, 0, 1, 28, 1, 55), dActionEntry (258, 0, 1, 28, 1, 55), dActionEntry (259, 0, 1, 28, 1, 55), 
			dActionEntry (260, 0, 1, 28, 1, 55), dActionEntry (261, 0, 1, 28, 1, 55), dActionEntry (263, 0, 1, 28, 1, 55), dActionEntry (265, 0, 1, 28, 1, 55), 
			dActionEntry (267, 0, 1, 28, 1, 55), dActionEntry (277, 0, 1, 28, 1, 55), dActionEntry (42, 0, 1, 21, 1, 32), dActionEntry (277, 0, 1, 21, 1, 32), 
			dActionEntry (125, 0, 1, 20, 1, 56), dActionEntry (256, 0, 1, 20, 1, 56), dActionEntry (257, 0, 1, 20, 1, 56), dActionEntry (258, 0, 1, 20, 1, 56), 
			dActionEntry (259, 0, 1, 20, 1, 56), dActionEntry (260, 0, 1, 20, 1, 56), dActionEntry (261, 0, 1, 20, 1, 56), dActionEntry (263, 0, 1, 20, 1, 56), 
			dActionEntry (265, 0, 1, 20, 1, 56), dActionEntry (267, 0, 1, 20, 1, 56), dActionEntry (277, 0, 1, 20, 1, 56), dActionEntry (277, 0, 0, 182, 0, 0), 
			dActionEntry (42, 0, 1, 21, 1, 34), dActionEntry (256, 0, 0, 183, 0, 0), dActionEntry (277, 0, 1, 21, 1, 34), dActionEntry (42, 0, 1, 21, 1, 36), 
			dActionEntry (277, 0, 1, 21, 1, 36), dActionEntry (42, 0, 0, 184, 0, 0), dActionEntry (277, 0, 1, 22, 1, 42), dActionEntry (272, 0, 1, 9, 8, 20), 
			dActionEntry (273, 0, 1, 9, 8, 20), dActionEntry (41, 0, 0, 185, 0, 0), dActionEntry (273, 0, 1, 9, 8, 20), dActionEntry (41, 0, 0, 186, 0, 0), 
			dActionEntry (125, 0, 0, 187, 0, 0), dActionEntry (256, 0, 1, 19, 3, 30), dActionEntry (257, 0, 1, 19, 3, 30), dActionEntry (258, 0, 1, 19, 3, 30), 
			dActionEntry (259, 0, 1, 19, 3, 30), dActionEntry (260, 0, 1, 19, 3, 30), dActionEntry (261, 0, 1, 19, 3, 30), dActionEntry (263, 0, 1, 19, 3, 30), 
			dActionEntry (265, 0, 1, 19, 3, 30), dActionEntry (267, 0, 1, 19, 3, 30), dActionEntry (277, 0, 1, 19, 3, 30), dActionEntry (41, 0, 0, 194, 0, 0), 
			dActionEntry (256, 0, 0, 135, 0, 0), dActionEntry (257, 0, 0, 142, 0, 0), dActionEntry (258, 0, 0, 139, 0, 0), dActionEntry (259, 0, 0, 190, 0, 0), 
			dActionEntry (260, 0, 0, 145, 0, 0), dActionEntry (261, 0, 0, 133, 0, 0), dActionEntry (263, 0, 0, 140, 0, 0), dActionEntry (265, 0, 0, 146, 0, 0), 
			dActionEntry (277, 0, 0, 189, 0, 0), dActionEntry (270, 0, 1, 10, 5, 24), dActionEntry (256, 0, 0, 135, 0, 0), dActionEntry (257, 0, 0, 142, 0, 0), 
			dActionEntry (258, 0, 0, 139, 0, 0), dActionEntry (259, 0, 0, 137, 0, 0), dActionEntry (260, 0, 0, 145, 0, 0), dActionEntry (261, 0, 0, 133, 0, 0), 
			dActionEntry (263, 0, 0, 140, 0, 0), dActionEntry (265, 0, 0, 146, 0, 0), dActionEntry (277, 0, 0, 136, 0, 0), dActionEntry (40, 0, 0, 197, 0, 0), 
			dActionEntry (270, 0, 1, 40, 1, 79), dActionEntry (277, 0, 1, 40, 1, 79), dActionEntry (270, 0, 1, 15, 1, 86), dActionEntry (277, 0, 0, 156, 0, 0), 
			dActionEntry (40, 0, 1, 21, 1, 37), dActionEntry (42, 0, 1, 21, 1, 37), dActionEntry (256, 0, 0, 199, 0, 0), dActionEntry (257, 0, 0, 200, 0, 0), 
			dActionEntry (260, 0, 0, 201, 0, 0), dActionEntry (40, 0, 1, 21, 1, 31), dActionEntry (42, 0, 1, 21, 1, 31), dActionEntry (40, 0, 1, 22, 1, 44), 
			dActionEntry (42, 0, 0, 202, 0, 0), dActionEntry (277, 0, 0, 203, 0, 0), dActionEntry (40, 0, 1, 21, 1, 33), dActionEntry (42, 0, 1, 21, 1, 33), 
			dActionEntry (40, 0, 1, 21, 1, 35), dActionEntry (42, 0, 1, 21, 1, 35), dActionEntry (40, 0, 1, 21, 1, 32), dActionEntry (42, 0, 1, 21, 1, 32), 
			dActionEntry (40, 0, 0, 204, 0, 0), dActionEntry (40, 0, 1, 21, 1, 34), dActionEntry (42, 0, 1, 21, 1, 34), dActionEntry (256, 0, 0, 205, 0, 0), 
			dActionEntry (40, 0, 1, 21, 1, 36), dActionEntry (42, 0, 1, 21, 1, 36), dActionEntry (40, 0, 1, 22, 1, 42), dActionEntry (42, 0, 0, 206, 0, 0), 
			dActionEntry (269, 0, 1, 39, 2, 78), dActionEntry (277, 0, 1, 39, 2, 78), dActionEntry (59, 0, 0, 207, 0, 0), dActionEntry (125, 0, 1, 20, 2, 57), 
			dActionEntry (256, 0, 1, 20, 2, 57), dActionEntry (257, 0, 1, 20, 2, 57), dActionEntry (258, 0, 1, 20, 2, 57), dActionEntry (259, 0, 1, 20, 2, 57), 
			dActionEntry (260, 0, 1, 20, 2, 57), dActionEntry (261, 0, 1, 20, 2, 57), dActionEntry (263, 0, 1, 20, 2, 57), dActionEntry (265, 0, 1, 20, 2, 57), 
			dActionEntry (267, 0, 1, 20, 2, 57), dActionEntry (277, 0, 1, 20, 2, 57), dActionEntry (42, 0, 1, 21, 2, 38), dActionEntry (277, 0, 1, 21, 2, 38), 
			dActionEntry (42, 0, 1, 21, 2, 39), dActionEntry (277, 0, 1, 21, 2, 39), dActionEntry (42, 0, 1, 21, 2, 40), dActionEntry (277, 0, 1, 21, 2, 40), 
			dActionEntry (125, 0, 0, 208, 0, 0), dActionEntry (256, 0, 0, 135, 0, 0), dActionEntry (257, 0, 0, 142, 0, 0), dActionEntry (258, 0, 0, 139, 0, 0), 
			dActionEntry (259, 0, 0, 137, 0, 0), dActionEntry (260, 0, 0, 145, 0, 0), dActionEntry (261, 0, 0, 133, 0, 0), dActionEntry (263, 0, 0, 140, 0, 0), 
			dActionEntry (265, 0, 0, 146, 0, 0), dActionEntry (267, 0, 0, 138, 0, 0), dActionEntry (277, 0, 0, 136, 0, 0), dActionEntry (277, 0, 1, 22, 2, 45), 
			dActionEntry (42, 0, 0, 209, 0, 0), dActionEntry (256, 0, 1, 26, 2, 52), dActionEntry (257, 0, 1, 26, 2, 52), dActionEntry (258, 0, 1, 26, 2, 52), 
			dActionEntry (259, 0, 1, 26, 2, 52), dActionEntry (260, 0, 1, 26, 2, 52), dActionEntry (261, 0, 1, 26, 2, 52), dActionEntry (263, 0, 1, 26, 2, 52), 
			dActionEntry (265, 0, 1, 26, 2, 52), dActionEntry (267, 0, 1, 26, 2, 52), dActionEntry (277, 0, 1, 26, 2, 52), dActionEntry (59, 0, 0, 210, 0, 0), 
			dActionEntry (59, 0, 1, 25, 1, 50), dActionEntry (91, 0, 0, 212, 0, 0), dActionEntry (42, 0, 1, 21, 2, 41), dActionEntry (277, 0, 1, 21, 2, 41), 
			dActionEntry (277, 0, 1, 22, 2, 43), dActionEntry (41, 0, 0, 214, 0, 0), dActionEntry (41, 0, 0, 215, 0, 0), dActionEntry (277, 0, 0, 216, 0, 0), 
			dActionEntry (41, 0, 1, 34, 1, 69), dActionEntry (44, 0, 1, 34, 1, 69), dActionEntry (42, 0, 1, 29, 1, 59), dActionEntry (277, 0, 1, 29, 1, 59), 
			dActionEntry (256, 0, 0, 218, 0, 0), dActionEntry (257, 0, 0, 222, 0, 0), dActionEntry (258, 0, 0, 220, 0, 0), dActionEntry (260, 0, 0, 224, 0, 0), 
			dActionEntry (261, 0, 0, 217, 0, 0), dActionEntry (263, 0, 0, 221, 0, 0), dActionEntry (265, 0, 0, 225, 0, 0), dActionEntry (277, 0, 0, 219, 0, 0), 
			dActionEntry (41, 0, 0, 229, 0, 0), dActionEntry (44, 0, 0, 228, 0, 0), dActionEntry (42, 0, 0, 230, 0, 0), dActionEntry (277, 0, 1, 31, 1, 62), 
			dActionEntry (277, 0, 0, 232, 0, 0), dActionEntry (59, 0, 0, 233, 0, 0), dActionEntry (42, 0, 1, 29, 1, 58), dActionEntry (277, 0, 1, 29, 1, 58), 
			dActionEntry (277, 0, 0, 234, 0, 0), dActionEntry (41, 0, 0, 236, 0, 0), dActionEntry (256, 0, 0, 135, 0, 0), dActionEntry (257, 0, 0, 142, 0, 0), 
			dActionEntry (258, 0, 0, 139, 0, 0), dActionEntry (259, 0, 0, 190, 0, 0), dActionEntry (260, 0, 0, 145, 0, 0), dActionEntry (261, 0, 0, 133, 0, 0), 
			dActionEntry (263, 0, 0, 140, 0, 0), dActionEntry (265, 0, 0, 146, 0, 0), dActionEntry (277, 0, 0, 189, 0, 0), dActionEntry (270, 0, 1, 40, 2, 80), 
			dActionEntry (277, 0, 1, 40, 2, 80), dActionEntry (40, 0, 1, 21, 2, 38), dActionEntry (42, 0, 1, 21, 2, 38), dActionEntry (40, 0, 1, 21, 2, 39), 
			dActionEntry (42, 0, 1, 21, 2, 39), dActionEntry (40, 0, 1, 21, 2, 40), dActionEntry (42, 0, 1, 21, 2, 40), dActionEntry (40, 0, 1, 22, 2, 45), 
			dActionEntry (42, 0, 0, 237, 0, 0), dActionEntry (42, 0, 0, 238, 0, 0), dActionEntry (40, 0, 1, 21, 2, 41), dActionEntry (42, 0, 1, 21, 2, 41), 
			dActionEntry (40, 0, 1, 22, 2, 43), dActionEntry (266, 0, 1, 18, 4, 29), dActionEntry (269, 0, 1, 18, 4, 29), dActionEntry (59, 0, 0, 239, 0, 0), 
			dActionEntry (277, 0, 1, 22, 3, 46), dActionEntry (125, 0, 1, 28, 3, 54), dActionEntry (256, 0, 1, 28, 3, 54), dActionEntry (257, 0, 1, 28, 3, 54), 
			dActionEntry (258, 0, 1, 28, 3, 54), dActionEntry (259, 0, 1, 28, 3, 54), dActionEntry (260, 0, 1, 28, 3, 54), dActionEntry (261, 0, 1, 28, 3, 54), 
			dActionEntry (263, 0, 1, 28, 3, 54), dActionEntry (265, 0, 1, 28, 3, 54), dActionEntry (267, 0, 1, 28, 3, 54), dActionEntry (277, 0, 1, 28, 3, 54), 
			dActionEntry (59, 0, 1, 24, 1, 48), dActionEntry (91, 0, 1, 24, 1, 48), dActionEntry (276, 0, 0, 240, 0, 0), dActionEntry (59, 0, 1, 25, 2, 51), 
			dActionEntry (91, 0, 0, 212, 0, 0), dActionEntry (41, 0, 0, 242, 0, 0), dActionEntry (272, 0, 1, 9, 11, 22), dActionEntry (273, 0, 1, 9, 11, 22), 
			dActionEntry (59, 0, 0, 243, 0, 0), dActionEntry (42, 0, 1, 21, 1, 37), dActionEntry (256, 0, 0, 244, 0, 0), dActionEntry (257, 0, 0, 245, 0, 0), 
			dActionEntry (260, 0, 0, 246, 0, 0), dActionEntry (42, 0, 1, 21, 1, 31), dActionEntry (42, 0, 1, 29, 1, 59), dActionEntry (42, 0, 1, 21, 1, 33), 
			dActionEntry (42, 0, 1, 21, 1, 35), dActionEntry (42, 0, 1, 21, 1, 32), dActionEntry (42, 0, 0, 230, 0, 0), dActionEntry (42, 0, 1, 21, 1, 34), 
			dActionEntry (256, 0, 0, 248, 0, 0), dActionEntry (42, 0, 1, 21, 1, 36), dActionEntry (42, 0, 1, 29, 1, 58), dActionEntry (256, 0, 0, 135, 0, 0), 
			dActionEntry (257, 0, 0, 142, 0, 0), dActionEntry (258, 0, 0, 139, 0, 0), dActionEntry (259, 0, 0, 190, 0, 0), dActionEntry (260, 0, 0, 145, 0, 0), 
			dActionEntry (261, 0, 0, 133, 0, 0), dActionEntry (263, 0, 0, 140, 0, 0), dActionEntry (265, 0, 0, 146, 0, 0), dActionEntry (277, 0, 0, 189, 0, 0), 
			dActionEntry (256, 0, 1, 33, 1, 68), dActionEntry (257, 0, 1, 33, 1, 68), dActionEntry (258, 0, 1, 33, 1, 68), dActionEntry (259, 0, 1, 33, 1, 68), 
			dActionEntry (260, 0, 1, 33, 1, 68), dActionEntry (261, 0, 1, 33, 1, 68), dActionEntry (263, 0, 1, 33, 1, 68), dActionEntry (265, 0, 1, 33, 1, 68), 
			dActionEntry (277, 0, 1, 33, 1, 68), dActionEntry (59, 0, 0, 250, 0, 0), dActionEntry (42, 0, 0, 251, 0, 0), dActionEntry (259, 0, 1, 30, 1, 60), 
			dActionEntry (277, 0, 1, 30, 1, 60), dActionEntry (259, 0, 0, 252, 0, 0), dActionEntry (277, 0, 1, 31, 2, 63), dActionEntry (41, 0, 1, 32, 2, 67), 
			dActionEntry (44, 0, 1, 32, 2, 67), dActionEntry (269, 0, 1, 36, 4, 73), dActionEntry (277, 0, 1, 36, 4, 73), dActionEntry (40, 0, 1, 37, 3, 74), 
			dActionEntry (41, 0, 0, 253, 0, 0), dActionEntry (44, 0, 0, 228, 0, 0), dActionEntry (59, 0, 0, 254, 0, 0), dActionEntry (40, 0, 1, 22, 3, 46), 
			dActionEntry (277, 0, 0, 255, 0, 0), dActionEntry (125, 0, 1, 27, 4, 53), dActionEntry (256, 0, 1, 27, 4, 53), dActionEntry (257, 0, 1, 27, 4, 53), 
			dActionEntry (258, 0, 1, 27, 4, 53), dActionEntry (259, 0, 1, 27, 4, 53), dActionEntry (260, 0, 1, 27, 4, 53), dActionEntry (261, 0, 1, 27, 4, 53), 
			dActionEntry (263, 0, 1, 27, 4, 53), dActionEntry (265, 0, 1, 27, 4, 53), dActionEntry (267, 0, 1, 27, 4, 53), dActionEntry (277, 0, 1, 27, 4, 53), 
			dActionEntry (93, 0, 0, 256, 0, 0), dActionEntry (59, 0, 1, 24, 2, 49), dActionEntry (91, 0, 1, 24, 2, 49), dActionEntry (273, 0, 1, 9, 11, 22), 
			dActionEntry (266, 0, 1, 17, 7, 26), dActionEntry (269, 0, 1, 17, 7, 26), dActionEntry (42, 0, 1, 21, 2, 38), dActionEntry (42, 0, 1, 21, 2, 39), 
			dActionEntry (42, 0, 1, 21, 2, 40), dActionEntry (259, 0, 0, 257, 0, 0), dActionEntry (277, 0, 1, 31, 3, 65), dActionEntry (42, 0, 1, 21, 2, 41), 
			dActionEntry (41, 0, 1, 34, 3, 70), dActionEntry (44, 0, 1, 34, 3, 70), dActionEntry (269, 0, 1, 36, 5, 72), dActionEntry (277, 0, 1, 36, 5, 72), 
			dActionEntry (259, 0, 1, 30, 2, 61), dActionEntry (277, 0, 1, 30, 2, 61), dActionEntry (277, 0, 1, 31, 3, 64), dActionEntry (59, 0, 0, 258, 0, 0), 
			dActionEntry (270, 0, 1, 38, 4, 76), dActionEntry (277, 0, 1, 38, 4, 76), dActionEntry (41, 0, 0, 259, 0, 0), dActionEntry (59, 0, 1, 23, 3, 47), 
			dActionEntry (91, 0, 1, 23, 3, 47), dActionEntry (277, 0, 1, 31, 4, 66), dActionEntry (270, 0, 1, 38, 5, 75), dActionEntry (277, 0, 1, 38, 5, 75), 
			dActionEntry (40, 0, 1, 35, 6, 71)};

	bool errorMode = false;
	const dStackPair& stackTop = stack.GetLast()->GetInfo();
	int state = stackTop.m_state;
	int start = actionsStart[state];
	int count = actionsCount[state];

	const dActionEntry* const table = &actionTable[start];
	const dActionEntry* action = FindAction (table, count, token);
	while (!action && (stack.GetCount() > 1)) {
		errorMode = true; 

		// we found a syntax error in go into error recovering mode, and find the token mark by a ". error" rule
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


bool newtonGrammar::Parse(lexical& scanner)
{
	static short gotoCount[] = {
			1, 0, 0, 2, 0, 2, 0, 0, 0, 1, 3, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 3, 
			3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 3, 3, 0, 
			0, 0, 0, 0, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 4, 0, 3, 0, 0, 0, 0, 4, 3, 0, 0, 0, 0, 0, 0, 3, 1, 0, 0, 0, 0, 0, 
			0, 0, 0, 0, 0, 1, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 6, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 4, 2, 2, 0, 5, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 2, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 1, 
			1, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static short gotoStart[] = {
			0, 1, 1, 1, 3, 3, 5, 5, 5, 5, 6, 9, 9, 9, 9, 9, 9, 11, 11, 11, 11, 11, 11, 11, 
			14, 17, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 23, 23, 26, 29, 32, 
			32, 32, 32, 32, 32, 35, 38, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 
			41, 41, 41, 45, 45, 48, 48, 48, 48, 48, 52, 55, 55, 55, 55, 55, 55, 55, 58, 59, 59, 59, 59, 59, 
			59, 59, 59, 59, 59, 59, 60, 60, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 69, 75, 
			75, 75, 75, 75, 75, 75, 75, 75, 75, 79, 81, 83, 83, 88, 88, 94, 94, 94, 94, 94, 94, 94, 94, 94, 
			94, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 100, 100, 102, 102, 102, 104, 104, 104, 104, 104, 104, 104, 104, 
			104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 109, 109, 109, 109, 109, 111, 111, 111, 111, 111, 111, 111, 111, 113, 
			114, 115, 115, 115, 115, 115, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 121, 121, 
			121, 121, 121, 121, 121, 121, 121, 121, 122, 122, 122, 122, 126, 126, 126, 126, 126, 126, 126, 126, 127, 127, 127, 127, 
			127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127};
	static dGotoEntry gotoTable[] = {
			dGotoEntry (279, 1), dGotoEntry (280, 6), dGotoEntry (281, 5), dGotoEntry (282, 10), dGotoEntry (284, 9), 
			dGotoEntry (285, 14), dGotoEntry (283, 16), dGotoEntry (286, 19), dGotoEntry (287, 18), dGotoEntry (286, 22), 
			dGotoEntry (287, 18), dGotoEntry (286, 31), dGotoEntry (287, 30), dGotoEntry (288, 29), dGotoEntry (286, 36), 
			dGotoEntry (287, 35), dGotoEntry (288, 34), dGotoEntry (286, 36), dGotoEntry (287, 35), dGotoEntry (288, 37), 
			dGotoEntry (286, 36), dGotoEntry (287, 35), dGotoEntry (288, 61), dGotoEntry (286, 31), dGotoEntry (287, 30), 
			dGotoEntry (288, 62), dGotoEntry (286, 36), dGotoEntry (287, 35), dGotoEntry (288, 63), dGotoEntry (286, 36), 
			dGotoEntry (287, 35), dGotoEntry (288, 64), dGotoEntry (286, 31), dGotoEntry (287, 30), dGotoEntry (288, 70), 
			dGotoEntry (286, 36), dGotoEntry (287, 35), dGotoEntry (288, 71), dGotoEntry (286, 36), dGotoEntry (287, 35), 
			dGotoEntry (288, 72), dGotoEntry (289, 92), dGotoEntry (290, 89), dGotoEntry (295, 91), dGotoEntry (320, 90), 
			dGotoEntry (286, 36), dGotoEntry (287, 35), dGotoEntry (288, 93), dGotoEntry (289, 96), dGotoEntry (290, 89), 
			dGotoEntry (295, 91), dGotoEntry (320, 90), dGotoEntry (286, 36), dGotoEntry (287, 35), dGotoEntry (288, 97), 
			dGotoEntry (291, 103), dGotoEntry (296, 104), dGotoEntry (321, 101), dGotoEntry (295, 105), dGotoEntry (296, 114), 
			dGotoEntry (292, 117), dGotoEntry (297, 119), dGotoEntry (298, 118), dGotoEntry (293, 128), dGotoEntry (297, 131), 
			dGotoEntry (298, 118), dGotoEntry (314, 127), dGotoEntry (315, 126), dGotoEntry (318, 130), dGotoEntry (299, 132), 
			dGotoEntry (300, 147), dGotoEntry (301, 144), dGotoEntry (305, 134), dGotoEntry (306, 141), dGotoEntry (307, 143), 
			dGotoEntry (294, 155), dGotoEntry (316, 157), dGotoEntry (317, 158), dGotoEntry (319, 159), dGotoEntry (300, 170), 
			dGotoEntry (301, 167), dGotoEntry (314, 127), dGotoEntry (315, 171), dGotoEntry (300, 147), dGotoEntry (301, 144), 
			dGotoEntry (305, 134), dGotoEntry (306, 141), dGotoEntry (307, 173), dGotoEntry (299, 177), dGotoEntry (300, 147), 
			dGotoEntry (301, 144), dGotoEntry (305, 134), dGotoEntry (306, 141), dGotoEntry (307, 143), dGotoEntry (304, 181), 
			dGotoEntry (300, 195), dGotoEntry (308, 192), dGotoEntry (310, 193), dGotoEntry (311, 188), dGotoEntry (313, 191), 
			dGotoEntry (300, 147), dGotoEntry (301, 196), dGotoEntry (316, 157), dGotoEntry (317, 198), dGotoEntry (300, 147), 
			dGotoEntry (301, 144), dGotoEntry (305, 134), dGotoEntry (306, 141), dGotoEntry (307, 173), dGotoEntry (302, 211), 
			dGotoEntry (303, 213), dGotoEntry (300, 226), dGotoEntry (308, 223), dGotoEntry (312, 227), dGotoEntry (309, 231), 
			dGotoEntry (300, 195), dGotoEntry (308, 192), dGotoEntry (310, 193), dGotoEntry (311, 188), dGotoEntry (313, 235), 
			dGotoEntry (302, 241), dGotoEntry (309, 247), dGotoEntry (300, 195), dGotoEntry (308, 192), dGotoEntry (310, 193), 
			dGotoEntry (311, 249), dGotoEntry (312, 227)};

	dList<dStackPair> stack;
	const int lastToken = 279;
	
	stack.Append ();
	m_grammarError = false;
	dToken token = dToken (scanner.NextToken());
	for (bool terminate = false; !terminate;) {

		const dActionEntry* const action = GetNextAction (stack, token, scanner);
		if (!action) {
			terminate = true;
			fprintf (stderr, "unrecoverable parser error\n");
			DTRACE (("unrecoverable parser error\n"));
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
					_ASSERTE (reduceCount < sizeof (parameter) / sizeof (parameter[0]));

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
						case 4:// engineVersion : engineMayorVersion engineMinorVersion 
{
	((NewtonBinding*)this)->EngineVersion (parameter[0].m_value.GetString(), parameter[1].m_value.GetString()); 
	((NewtonBinding*)this)->BlockEnd();
	((NewtonBinding*)this)->DeclareDataType  ("float", "double"); 
	((NewtonBinding*)this)->BlockEnd();
}
break;

						case 5:// engineMayorVersion : _DEFINE _LITERAL_IDENTIFIER _NUMERIC_CONSTANT 
{entry.m_value = parameter[2].m_value;}
break;

						case 6:// engineMinorVersion : _DEFINE _LITERAL_IDENTIFIER _NUMERIC_CONSTANT 
{entry.m_value = parameter[2].m_value;}
break;

						case 87:// constantDefinitionList : recursiveConstantDefinitionList 
{((NewtonBinding*)this)->BlockEnd();}
break;

						case 88:// internalEngineStructList : recursiveInternalEngineStructList 
{((NewtonBinding*)this)->BlockEnd();}
break;

						case 25:// constantDefinition : _DEFINE _LITERAL_IDENTIFIER _NUMERIC_CONSTANT 
{ ((NewtonBinding*)this)->ConstantDefinition (parameter[1].m_value.GetString(), parameter[2].m_value.GetString()); }
break;

						case 85:// engineCallbackList : recursiveEngineCallbackList 
{((NewtonBinding*)this)->BlockEnd();}
break;

						case 37:// intrinsicType : _UNSIGNED 
{entry.m_value = parameter[0].m_value;}
break;

						case 31:// intrinsicType : _INT 
{entry.m_value = parameter[0].m_value;}
break;

						case 44:// genericDataType : _LITERAL_IDENTIFIER 
{entry.m_value = parameter[0].m_value;}
break;

						case 33:// intrinsicType : _VOID 
{entry.m_value = parameter[0].m_value;}
break;

						case 35:// intrinsicType : _DFLOAT 
{entry.m_value = parameter[0].m_value;}
break;

						case 32:// intrinsicType : _CHAR 
{entry.m_value = parameter[0].m_value;}
break;

						case 34:// intrinsicType : _SHORT 
{entry.m_value = parameter[0].m_value;}
break;

						case 36:// intrinsicType : _DFLOAT64 
{entry.m_value = parameter[0].m_value;}
break;

						case 42:// genericDataType : intrinsicType 
{entry.m_value = parameter[0].m_value;}
break;

						case 30:// engineStructBegin : _STRUCT _LITERAL_IDENTIFIER { 
{((NewtonBinding*)this)->StructDeclareStart (parameter[1].m_value.GetString());}
break;

						case 86:// enginefunctionsList : recursiveEnginefunctionsList 
{((NewtonBinding*)this)->BlockEnd();}
break;

						case 38:// intrinsicType : _UNSIGNED _INT 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 39:// intrinsicType : _UNSIGNED _CHAR 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 40:// intrinsicType : _UNSIGNED _SHORT 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 45:// genericDataType : _LITERAL_IDENTIFIER * 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 52:// unionBegin : _UNION { 
{((NewtonBinding*)this)->StructNameLessUnion ();}
break;

						case 50:// structDataName : _LITERAL_IDENTIFIER 
{entry.m_value = parameter[0].m_value;}
break;

						case 41:// intrinsicType : _SHORT _INT 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 43:// genericDataType : intrinsicType * 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 59:// argumentTypeName : _LITERAL_IDENTIFIER 
{entry.m_value = parameter[0].m_value;}
break;

						case 62:// functionArgumentType : argumentTypeName 
{entry.m_value = parameter[0].m_value;}
break;

						case 58:// argumentTypeName : intrinsicType 
{entry.m_value = parameter[0].m_value;}
break;

						case 29:// engineStruct : engineStructBegin structDataDeclarationList } ; 
{((NewtonBinding*)this)->StructDeclareEnd(); ((NewtonBinding*)this)->BlockEnd();}
break;

						case 46:// genericDataType : _CONST _LITERAL_IDENTIFIER * 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + ' ' + parameter[1].m_value.m_data + parameter[2].m_value.m_data;}
break;

						case 54:// structDataDeclaration : genericDataType structDataName ; 
{((NewtonBinding*)this)->StructAddDataType (parameter[0].m_value.GetString(), parameter[1].m_value.GetString());}
break;

						case 48:// arrayTypeList : arrayType 
{entry.m_value = parameter[0].m_value;}
break;

						case 51:// structDataName : _LITERAL_IDENTIFIER arrayTypeList 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 68:// callbackArgumentSeparator : , 
{((NewtonBinding*)this)->FunctionArgumentSeparator();}
break;

						case 60:// dereference : * 
{entry.m_value = parameter[0].m_value;}
break;

						case 63:// functionArgumentType : argumentTypeName dereference 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 67:// callbackArgument : functionArgumentType _LITERAL_IDENTIFIER 
{((NewtonBinding*)this)->FunctionArgument (parameter[0].m_value.m_data, parameter[1].m_value.m_data);}
break;

						case 73:// engineCallback : engineCallbackBegin ( ) ; 
{((NewtonBinding*)this)->FunctionDeclarationEnd ();}
break;

						case 74:// enginefunctionBegin : _LITERAL_IDENTIFIER genericDataType _LITERAL_IDENTIFIER 
{((NewtonBinding*)this)->FunctionDeclaration (parameter[1].m_value.m_data, parameter[2].m_value.m_data);}
break;

						case 53:// unionType : unionBegin structDataDeclarationList } ; 
{((NewtonBinding*)this)->StructDeclareEnd();}
break;

						case 49:// arrayTypeList : arrayTypeList arrayType 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 26:// internalEngineStruct : _TYPEDEF _STRUCT _LITERAL_IDENTIFIER { } _LITERAL_IDENTIFIER ; 
{((NewtonBinding*)this)->InternalEngineStruct (parameter[2].m_value.GetString());}
break;

						case 65:// functionArgumentType : _CONST argumentTypeName dereference 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + " " + parameter[1].m_value.m_data + parameter[2].m_value.m_data;}
break;

						case 72:// engineCallback : engineCallbackBegin ( functionsArgumentList ) ; 
{((NewtonBinding*)this)->FunctionDeclarationEnd ();}
break;

						case 61:// dereference : * * 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data;}
break;

						case 64:// functionArgumentType : argumentTypeName dereference _CONST 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data + " " + parameter[2].m_value.m_data;}
break;

						case 76:// enginefunction : enginefunctionBegin ( ) ; 
{((NewtonBinding*)this)->FunctionDeclarationEnd ();}
break;

						case 47:// arrayType : [ _NUMERIC_CONSTANT ] 
{ entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + parameter[1].m_value.m_data + parameter[2].m_value.m_data;}
break;

						case 66:// functionArgumentType : _CONST argumentTypeName dereference _CONST 
{entry.m_value = parameter[0].m_value; entry.m_value.m_data = parameter[0].m_value.m_data + " " + parameter[1].m_value.m_data + parameter[2].m_value.m_data + " " + parameter[3].m_value.m_data;}
break;

						case 75:// enginefunction : enginefunctionBegin ( functionsArgumentList ) ; 
{((NewtonBinding*)this)->FunctionDeclarationEnd ();}
break;

						case 71:// engineCallbackBegin : _TYPEDEF genericDataType ( * _LITERAL_IDENTIFIER ) 
{((NewtonBinding*)this)->FunctionCallbackDeclaration (parameter[1].m_value.m_data, parameter[4].m_value.m_data);}
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
					_ASSERTE (0);
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





