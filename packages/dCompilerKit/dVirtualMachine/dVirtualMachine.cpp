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


#include "dVirtualMachine.h"


dVirtualMachine::dNemonic dVirtualMachine::m_nemonics[] = 
{
/*
	{mov,		"mov"},

	{addi,		"addi"},		
	{add,		"add"},
	{sub,		"sub"},		
	{mul,		"mul"},
	{div,		"div"}, 
	{abs,		"abs"},
	{neg,		"neg"},

	{and,		"and"},		
	{or,		"or"},		
	{xor,		"xor"},
	{not,		"xor"},

	{sll,		"sll"},		
	{srl,		"srl"},		

	{loadb,		"loadb"},		
	{loadw,		"loadw"},		
	{loadd,		"loadd"},		

	{storeb,	"storeb"},	
	{storew,	"storew"},	
	{stored,	"stored"},	

	{beq,		"beq"},		
	{bne,		"bne"},		
	{blt,		"blt"},		
	{ble,		"ble"},		
	{bgt,		"bgt"},		
	{bget,		"bget"},		

	{syscall,	"syscall"},		
	{jump,		"jump"},		

	{enter,		"enter"},		
	{exit,		"exit"},		
	{push,		"push"},		 
	{pop,		"pop"},		
*/
	{nop,		"nop"},
};

dVirtualMachine::dVirtualMachine(void)
	:m_codeSegementSize(0)
	,m_codeSegment(NULL)
{	
	dAssert (nop <= 1 << D_OPCODE_FIELD);
}

dVirtualMachine::~dVirtualMachine(void)
{
	if (m_codeSegment) {
		delete m_codeSegment;
	}
}

