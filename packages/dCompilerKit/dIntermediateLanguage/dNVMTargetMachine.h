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


#ifndef _dNVMTargetMachine_H_
#define _dNVMTargetMachine_H_

#include "dCILstdafx.h"
#include "dNVMSubtarget.h"


class dNVMTargetMachine : public llvm::LLVMTargetMachine 
{
	public:
	dNVMTargetMachine (const llvm::Target &T, llvm::StringRef TT, llvm::StringRef CPU, llvm::StringRef FS, const llvm::TargetOptions &options, llvm::Reloc::Model RM, llvm::CodeModel::Model CM, llvm::CodeGenOpt::Level OL);
	~dNVMTargetMachine();

	const llvm::TargetSubtargetInfo *getSubtargetImpl() const;
	const llvm::TargetIntrinsicInfo *getIntrinsicInfo() const;


	// Pass Pipeline Configuration
//	TargetPassConfig *createPassConfig(PassManagerBase &PM) override;
//	bool addCodeEmitter(PassManagerBase &PM, JITCodeEmitter &JCE) override;

	dNVMSubTarget m_subtarget;
	
};




#endif
