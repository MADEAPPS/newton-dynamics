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


#include "dCILstdafx.h"
#include "dNVMAsmParser.h"
#include "dNVMTargetMachine.h"


NVMAsmParser::NVMAsmParser (llvm::MCSubtargetInfo &sti, llvm::MCAsmParser &parser, const llvm::MCInstrInfo &MII, const llvm::MCTargetOptions &Options)
	:llvm::MCTargetAsmParser()
//	,STI(sti)
//	,Parser(parser) 
{
	dAssert (0);
		// Initialize the set of available features.
//		setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
}

	/*
	/// @name Auto-generated Match Functions
	/// {
	#define GET_ASSEMBLER_HEADER
	#include "SparcGenAsmMatcher.inc"

	/// }
	*/



bool NVMAsmParser::ParseRegister(unsigned &RegNo, llvm::SMLoc &StartLoc, llvm::SMLoc &EndLoc)
{
	dAssert (0);
	return false;
}

bool NVMAsmParser::ParseInstruction (llvm::ParseInstructionInfo &Info, llvm::StringRef Name, llvm::SMLoc NameLoc, llvm::OperandVector &Operands)
{
	dAssert (0);
	return false;
}


bool NVMAsmParser::parseDirectiveWord(unsigned Size, llvm::SMLoc L)
{
	dAssert (0);
	return false;
}


bool NVMAsmParser::ParseDirective(llvm::AsmToken DirectiveID)
{
	dAssert (0);
	return false;
}

bool NVMAsmParser::MatchAndEmitInstruction(llvm::SMLoc IDLoc, unsigned &Opcode, llvm::OperandVector &Operands, llvm::MCStreamer &Out, unsigned &ErrorInfo, bool MatchingInlineAsm)
{
	dAssert (0);
	return false;
}


bool NVMAsmParser::mnemonicIsValid(llvm::StringRef Mnemonic, unsigned VariantID)
{
	dAssert (0);
	return false;
}

void NVMAsmParser::convertToMapAndConstraints(unsigned Kind, const llvm::OperandVector &Operands)
{
	dAssert (0);
}

/*
	// public interface of the MCTargetAsmParser.
	bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode, OperandVector &Operands, MCStreamer &Out, unsigned &ErrorInfo, bool MatchingInlineAsm) override;
	
	bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name, SMLoc NameLoc, OperandVector &Operands) override;
	bool ParseDirective(AsmToken DirectiveID) override;

	unsigned validateTargetOperandClass(MCParsedAsmOperand &Op, unsigned Kind) override;

	// Custom parse functions for Sparc specific operands.
	OperandMatchResultTy parseMEMOperand(OperandVector &Operands);

	OperandMatchResultTy parseOperand(OperandVector &Operands, StringRef Name);

	OperandMatchResultTy parseSparcAsmOperand(std::unique_ptr<SparcOperand> &Operand, bool isCall = false);

	OperandMatchResultTy parseBranchModifiers(OperandVector &Operands);

	// returns true if Tok is matched to a register and returns register in RegNo.
	bool matchRegisterName(const AsmToken &Tok, unsigned &RegNo, unsigned &RegKind);

	bool matchSparcAsmModifiers(const MCExpr *&EVal, SMLoc &EndLoc);
	bool parseDirectiveWord(unsigned Size, SMLoc L);

	bool is64Bit() const { return STI.getTargetTriple().startswith("sparcv9"); }
*/
