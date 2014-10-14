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
	{ m_nop, "nop" },
	{ m_enter, "enter" },
	{ m_leave, "leave" },

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

	{push,		"push"},		 
	{pop,		"pop"},		
*/
	
};

dVirtualMachine::dVirtualMachine(void)
	:m_functionTable()
	,m_codeSegmentSize(0)
	,m_codeSegment(NULL)
{	
//	m_codeSegment = new dOpCode[m_codeMaxSize];
	dAssert(m_count <= (1 << D_OPCODE_FIELD));
}

dVirtualMachine::~dVirtualMachine(void)
{
	if (m_codeSegment) {
		delete[] m_codeSegment;
	}
}

dVirtualMachine::dOpCode* dVirtualMachine::AllocCodeSegement(int sizeInWords)
{
/*
	while ((m_codeSegmentSize + sizeInWords) > m_codeMaxSize) {
		m_codeMaxSize *= 2;
		dOpCode* const codeSegment = new dOpCode[m_codeMaxSize];
		memcpy(codeSegment, m_codeSegment, m_codeSegmentSize * sizeof (dOpCode));
		delete[] m_codeSegment;
		m_codeSegment = codeSegment;
	}
*/	
	return new dOpCode[sizeInWords];
}

void dVirtualMachine::AddFunction(const dString& name, int byteCodeOffset)
{
	//AllocCodeSegement(codeSize);
	m_functionTable.Insert(byteCodeOffset, name);

	//memcpy(&m_codeSegment[m_codeSegmentSize], byteCode, codeSize * sizeof (dOpCode));
	//m_codeSegmentSize += codeSize;
}