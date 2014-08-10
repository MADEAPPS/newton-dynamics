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


#ifndef _dNVMAsmPrinter_H_
#define _dNVMAsmPrinter_H_

#include "dCILstdafx.h"
//#include "SparcInstrInfo.h"
//#include "SparcSubtarget.h"
//#include "llvm/Target/TargetMachine.h"


class NVMAsmPrinter : public llvm::AsmPrinter 
{
/*
	SparcTargetStreamer &getTargetStreamer() 
	{
		return static_cast<SparcTargetStreamer &>(*OutStreamer.getTargetStreamer());
	}
*/
	public:
	NVMAsmPrinter (llvm::TargetMachine &TM, llvm::MCStreamer &Streamer);
	const char *getPassName() const override;
	static const char *getRegisterName(unsigned RegNo);

/*
	void printOperand(const MachineInstr *MI, int opNum, raw_ostream &OS);
	void printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &OS, const char *Modifier = nullptr);
	void printCCOperand(const MachineInstr *MI, int opNum, raw_ostream &OS);
	void EmitFunctionBodyStart() override;
	void EmitInstruction(const MachineInstr *MI) override;
	void EmitEndOfAsmFile(Module &M) override;
	bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo, unsigned AsmVariant, const char *ExtraCode, raw_ostream &O) override;
	bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo, unsigned AsmVariant, const char *ExtraCode, raw_ostream &O) override;
	void LowerGETPCXAndEmitMCInsts(const MachineInstr *MI, const MCSubtargetInfo &STI);
*/
};





#endif
