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
	_ASSERTE (nop <= 1 << D_OPCODE_FIELD);
}

dVirtualMachine::~dVirtualMachine(void)
{
	if (m_codeSegment) {
		delete m_codeSegment;
	}
}

