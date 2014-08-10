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


#ifndef _dNVMAsmParser_H_
#define _dNVMAsmParser_H_

#include "dCILstdafx.h"


class NVMAsmParser : public llvm::MCTargetAsmParser 
{
	public:
	NVMAsmParser (llvm::MCSubtargetInfo &sti, llvm::MCAsmParser &parser, const llvm::MCInstrInfo &MII, const llvm::MCTargetOptions &Options);

	
	bool parseDirectiveWord (unsigned Size, llvm::SMLoc L);
	bool ParseDirective (llvm::AsmToken DirectiveID) override;
	bool mnemonicIsValid (llvm::StringRef Mnemonic, unsigned VariantID);
	void convertToMapAndConstraints(unsigned Kind, const llvm::OperandVector &Operands);
	bool ParseRegister (unsigned &RegNo, llvm::SMLoc &StartLoc, llvm::SMLoc &EndLoc) override;
	
	bool ParseInstruction (llvm::ParseInstructionInfo &Info, llvm::StringRef Name, llvm::SMLoc NameLoc, llvm::OperandVector &Operands) override;
	bool MatchAndEmitInstruction (llvm::SMLoc IDLoc, unsigned &Opcode, llvm::OperandVector &Operands, llvm::MCStreamer &Out, unsigned &ErrorInfo, bool MatchingInlineAsm) override;

//	MCSubtargetInfo &STI;
//	MCAsmParser &Parser;

	/*
	/// @name Auto-generated Match Functions
	/// {
	#define GET_ASSEMBLER_HEADER
	#include "SparcGenAsmMatcher.inc"

	/// }
	*/
/*
	// public interface of the MCTargetAsmParser.
	
	
	
	

	unsigned validateTargetOperandClass(MCParsedAsmOperand &Op, unsigned Kind) override;

	// Custom parse functions for Sparc specific operands.
	OperandMatchResultTy parseMEMOperand(OperandVector &Operands);

	OperandMatchResultTy parseOperand(OperandVector &Operands, StringRef Name);

	OperandMatchResultTy parseSparcAsmOperand(std::unique_ptr<SparcOperand> &Operand, bool isCall = false);

	OperandMatchResultTy parseBranchModifiers(OperandVector &Operands);

	// returns true if Tok is matched to a register and returns register in RegNo.
	bool matchRegisterName(const AsmToken &Tok, unsigned &RegNo, unsigned &RegKind);

	bool matchSparcAsmModifiers(const MCExpr *&EVal, SMLoc &EndLoc);
	

	bool is64Bit() const { return STI.getTargetTriple().startswith("sparcv9"); }
*/
};








#endif
