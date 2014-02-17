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

#ifndef __dCIL_H_
#define __dCIL_H_

#include "dCILstdafx.h"




/*
#define D_USE_COMPLEX_ADRESSING_MODE
#define D_TEMPRARY_SYMBOL			"t"
#define D_LOOP_HEADER_SYMBOL		"loopHeader"
#define D_LOOP_TAIL_SYMBOL			"loopTail"


inline dString GetTemporaryVariableName(int index)
{
	char tmp[256];
	sprintf (tmp, "%s%d", D_TEMPRARY_SYMBOL, index);
	return tmp;
}

inline dString GetReturnVariableName()
{
	return GetTemporaryVariableName(0);
}

class dDataFlowGraph; 
*/

using  namespace llvm;

class dCIL: public LLVMTargetMachine
{
	public:
	enum dReturnType
	{
		m_voidNone,
		m_intRegister,
		m_floatRegister,
	};

	enum dIntrisicType
	{
		m_void,
		m_bool,
		m_byte,
		m_short,
		m_int,
		m_long,
		m_float,
		m_double,
		m_classPointer,
	};

	class dReturnValue
	{
		public:
		dReturnValue ()
			:m_type(m_int)
		{
			m_f = 0.0;
		}

		dIntrisicType m_type;
		union {;
			dMachineIntRegister m_i;
			dMachineFloatRegister m_f;
		};
	};

	
	dCIL (const Target &T, StringRef TargetTriple, StringRef CPU, StringRef FS, TargetOptions Options, Reloc::Model RM, CodeModel::Model CM, CodeGenOpt::Level OL);
	virtual ~dCIL(void);



	static dCIL* CreateTargetMachine();
//	void Trace();
//	dString NewTemp (); 
//	dString NewLabel (); 
//	void ResetTemporaries();
//	dListNode* NewStatement();
	
//	void Optimize (dListNode* const functionNode, int argumentInRegisters, dReturnType returnType);

	private:
/*
	bool RemoveNop(dListNode* const functionNode);
	bool RemoveRedundantJumps(dListNode* const functionNode);

	int m_mark;
	int m_tempIndex;
	int m_labelIndex;
	bool m_commutativeOperator[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_conditionals[dTreeAdressStmt::m_operatorsCount];
	dTreeAdressStmt::dOperator m_operatorComplement[dTreeAdressStmt::m_operatorsCount];
	friend dDataFlowGraph;
*/



	private:
	static void RegisterTarget(); 
	static bool getArchMatch(Triple::ArchType Arch);
//	SparcTargetMachine(const Module &M, const std::string &FS);

//	virtual const SparcInstrInfo *getInstrInfo() const {return &InstrInfo; }
//	virtual const TargetFrameInfo *getFrameInfo() const {return &FrameInfo; }
//	virtual const TargetSubtarget *getSubtargetImpl() const{return &Subtarget; }
//	virtual const TargetRegisterInfo *getRegisterInfo() const {
//		return &InstrInfo.getRegisterInfo();
//	}
//	virtual const DataLayout *getDataLayout() const { return &DataLayout; }
//	static unsigned getModuleMatchQuality(const Module &M);

	// Pass Pipeline Configuration
//	virtual bool addInstSelector(PassManagerBase &PM, bool Fast);
//	virtual bool addPreEmitPass(PassManagerBase &PM, bool Fast);


//	protected:
//	virtual const TargetAsmInfo *createTargetAsmInfo() const;


//	const DataLayout DataLayout;       // Calculates type size & alignment
//	SparcSubtarget Subtarget;
//	SparcInstrInfo InstrInfo;
//	TargetFrameInfo FrameInfo;

	static Target m_target;
};


#endif