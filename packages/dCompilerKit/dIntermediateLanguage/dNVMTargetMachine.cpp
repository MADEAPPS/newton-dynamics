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
#include "dNVMTargetMachine.h"

dNVMTargetMachine::dNVMTargetMachine(const llvm::Target &T, llvm::StringRef TT, llvm::StringRef CPU, llvm::StringRef FS, const llvm::TargetOptions &options, llvm::Reloc::Model RM, llvm::CodeModel::Model CM, llvm::CodeGenOpt::Level OL)
  : LLVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL)
//  ,Subtarget(TT, CPU, FS, *this, is64bit) 
{
//  initAsmInfo();
}

/*
namespace {
/// Sparc Code Generator Pass Configuration Options.
class SparcPassConfig : public TargetPassConfig {
public:
  SparcPassConfig(dNVMTargetMachine *TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  dNVMTargetMachine &getdNVMTargetMachine() const {
    return getTM<dNVMTargetMachine>();
  }

  bool addInstSelector() override;
  bool addPreEmitPass() override;
};
} // namespace

TargetPassConfig *dNVMTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new SparcPassConfig(this, PM);
}

bool SparcPassConfig::addInstSelector() {
  addPass(createSparcISelDag(getdNVMTargetMachine()));
  return false;
}

bool dNVMTargetMachine::addCodeEmitter(PassManagerBase &PM,
                                        JITCodeEmitter &JCE) {
  // Machine code emitter pass for Sparc.
  PM.add(createSparcJITCodeEmitterPass(*this, JCE));
  return false;
}

/// addPreEmitPass - This pass may be implemented by targets that want to run
/// passes immediately before machine code is emitted.  This should return
/// true if -print-machineinstrs should print out the code after the passes.
bool SparcPassConfig::addPreEmitPass(){
  addPass(createSparcDelaySlotFillerPass(getdNVMTargetMachine()));
  return true;
}

void SparcV8TargetMachine::anchor() { }

SparcV8TargetMachine::SparcV8TargetMachine(const Target &T,
                                           StringRef TT, StringRef CPU,
                                           StringRef FS,
                                           const TargetOptions &Options,
                                           Reloc::Model RM,
                                           CodeModel::Model CM,
                                           CodeGenOpt::Level OL)
  : dNVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, false) {
}

void SparcV9TargetMachine::anchor() { }

SparcV9TargetMachine::SparcV9TargetMachine(const Target &T,
                                           StringRef TT,  StringRef CPU,
                                           StringRef FS,
                                           const TargetOptions &Options,
                                           Reloc::Model RM,
                                           CodeModel::Model CM,
                                           CodeGenOpt::Level OL)
  : dNVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, true) {
}
*/