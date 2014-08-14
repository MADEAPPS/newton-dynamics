/* Copyright (c) <2009> <nvm Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

//===----------------------------------------------------------------------===//
// This is the top level entry point for the NVM target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

//include "llvm/Target/Target.td"
include "../dIntermediateLanguage/Target.td"

/*
//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

//include "NVMRegisterInfo.td"
//include "../dIntermediateLanguage/dNVMRegisterInfo.td"
//include "NVMSchedule.td"
//include "NVMInstrInfo.td"
//include "NVMCallingConv.td"

def NVMInstrInfo : InstrInfo;


//===----------------------------------------------------------------------===//
// NVM Subtarget features                                                    //
//===----------------------------------------------------------------------===//

def FeatureO32         : SubtargetFeature<"o32", "NVMABI", "O32", "Enable o32 ABI">;
def FeatureNVM1        : SubtargetFeature<"NVM1", "NVMArchVersion", "NVM1", "NVM I ISA Support [highly experimental]">;


def FeatureNoABICalls  : SubtargetFeature<"noabicalls", "NoABICalls", "true", "Disable SVR4-style position-independent code.">;
def FeatureGP64Bit     : SubtargetFeature<"gp64", "IsGP64bit", "true", "General Purpose Registers are 64-bit wide.">;
def FeatureFP64Bit     : SubtargetFeature<"fp64", "IsFP64bit", "true", "Support 64-bit FP registers.">;
def FeatureFPXX        : SubtargetFeature<"fpxx", "IsFPXX", "true",  "Support for FPXX.">;
def FeatureNaN2008     : SubtargetFeature<"nan2008", "IsNaN2008bit", "true", "IEEE 754-2008 NaN encoding.">;
def FeatureSingleFloat : SubtargetFeature<"single-float", "IsSingleFloat", "true", "Only supports single precision float">;

def FeatureN32         : SubtargetFeature<"n32", "NVMABI", "N32", "Enable n32 ABI">;
def FeatureN64         : SubtargetFeature<"n64", "NVMABI", "N64", "Enable n64 ABI">;
def FeatureEABI        : SubtargetFeature<"eabi", "NVMABI", "EABI","Enable eabi ABI">;
def FeatureNoOddSPReg  : SubtargetFeature<"nooddspreg", "UseOddSPReg", "false", "Disable odd numbered single-precision " "registers">;
def FeatureVFPU        : SubtargetFeature<"vfpu", "HasVFPU", "true", "Enable vector FPU instructions.">;

def FeatureNVM2       : SubtargetFeature<"NVM2", "NVMArchVersion", "NVM2", "NVM II ISA Support [highly experimental]", [FeatureNVM1]>;
def FeatureNVM3_32    : SubtargetFeature<"NVM3_32", "HasNVM3_32", "true", "Subset of NVM-III that is also in NVM32 " "[highly experimental]">;
def FeatureNVM3_32r2  : SubtargetFeature<"NVM3_32r2", "HasNVM3_32r2", "true", "Subset of NVM-III that is also in NVM32r2 " "[highly experimental]">;
def FeatureNVM3       : SubtargetFeature<"NVM3", "NVMArchVersion", "NVM3", "NVM III ISA Support [highly experimental]", [FeatureNVM2, FeatureNVM3_32, FeatureNVM3_32r2, FeatureGP64Bit, FeatureFP64Bit]>;
def FeatureNVM4_32    : SubtargetFeature<"NVM4_32", "HasNVM4_32", "true","Subset of NVM-IV that is also in NVM32 " "[highly experimental]">;
def FeatureNVM4_32r2  : SubtargetFeature<"NVM4_32r2", "HasNVM4_32r2", "true", "Subset of NVM-IV that is also in NVM32r2 "  "[highly experimental]">;
def FeatureNVM4       : SubtargetFeature<"NVM4", "NVMArchVersion", "NVM4", "NVM IV ISA Support", [FeatureNVM3, FeatureNVM4_32, FeatureNVM4_32r2]>;
def FeatureNVM5_32r2  : SubtargetFeature<"NVM5_32r2", "HasNVM5_32r2", "true", "Subset of NVM-V that is also in NVM32r2 " "[highly experimental]">;
def FeatureNVM5       : SubtargetFeature<"NVM5", "NVMArchVersion", "NVM5", "NVM V ISA Support [highly experimental]", [FeatureNVM4, FeatureNVM5_32r2]>;
def FeatureNVM32      : SubtargetFeature<"NVM32", "NVMArchVersion", "NVM32", "NVM32 ISA Support", [FeatureNVM2, FeatureNVM3_32, FeatureNVM4_32]>;
def FeatureNVM32r2    : SubtargetFeature<"NVM32r2", "NVMArchVersion", "NVM32r2", "NVM32r2 ISA Support", [FeatureNVM3_32r2, FeatureNVM4_32r2, FeatureNVM5_32r2, FeatureNVM32]>;
def FeatureNVM32r6    : SubtargetFeature<"NVM32r6", "NVMArchVersion", "NVM32r6", "NVM32r6 ISA Support [experimental]", [FeatureNVM32r2, FeatureFP64Bit, FeatureNaN2008]>;
def FeatureNVM64      : SubtargetFeature<"NVM64", "NVMArchVersion", "NVM64", "NVM64 ISA Support", [FeatureNVM5, FeatureNVM32]>;
def FeatureNVM64r2    : SubtargetFeature<"NVM64r2", "NVMArchVersion", "NVM64r2", "NVM64r2 ISA Support", [FeatureNVM64, FeatureNVM32r2]>;
def FeatureNVM64r6    : SubtargetFeature<"NVM64r6", "NVMArchVersion", "NVM64r6", "NVM64r6 ISA Support [experimental]", [FeatureNVM32r6, FeatureNVM64r2, FeatureNaN2008]>;
def FeatureNVM16      : SubtargetFeature<"NVM16", "InNVM16Mode", "true",  "NVM16 mode">;
def FeatureDSP        : SubtargetFeature<"dsp", "HasDSP", "true", "NVM DSP ASE">;
def FeatureDSPR2      : SubtargetFeature<"dspr2", "HasDSPR2", "true", "NVM DSP-R2 ASE", [FeatureDSP]>;
def FeatureMSA        : SubtargetFeature<"msa", "HasMSA", "true", "NVM MSA ASE">;
def FeatureMicroNVM   : SubtargetFeature<"microNVM", "InMicroNVMMode", "true", "microNVM mode">;
def FeatureCnNVM      : SubtargetFeature<"cnNVM", "HasCnNVM",  "true", "Octeon cnNVM Support",  [FeatureNVM64r2]>;


//===----------------------------------------------------------------------===//
// NVM processors supported.
//===----------------------------------------------------------------------===//

class Proc<string Name, list<SubtargetFeature> Features> : Processor<Name, NVMGenericItineraries, Features>;

def : Proc<"NVM1", [FeatureNVM1, FeatureO32]>;
def : Proc<"NVM2", [FeatureNVM2, FeatureO32]>;
def : Proc<"NVM32", [FeatureNVM32, FeatureO32]>;
def : Proc<"NVM32r2", [FeatureNVM32r2, FeatureO32]>;
def : Proc<"NVM32r6", [FeatureNVM32r6, FeatureO32]>;

def : Proc<"NVM3", [FeatureNVM3, FeatureN64]>;
def : Proc<"NVM4", [FeatureNVM4, FeatureN64]>;
def : Proc<"NVM5", [FeatureNVM5, FeatureN64]>;
def : Proc<"NVM64", [FeatureNVM64, FeatureN64]>;
def : Proc<"NVM64r2", [FeatureNVM64r2, FeatureN64]>;
def : Proc<"NVM64r6", [FeatureNVM64r6, FeatureN64]>;
def : Proc<"NVM16", [FeatureNVM16, FeatureO32]>;
def : Proc<"octeon", [FeatureNVM64r2, FeatureN64, FeatureCnNVM]>;


def NVMAsmParser : AsmParser 
{
  let ShouldEmitMatchRegisterName = 0;
  let MnemonicContainsDot = 1;
}

def NVMAsmParserVariant : AsmParserVariant 
{
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def NVM : Target 
{
  let InstructionSet = NVMInstrInfo;
  let AssemblyParsers = [NVMAsmParser];
  let AssemblyParserVariants = [NVMAsmParserVariant];
}
*/



include "../dIntermediateLanguage/dNVMCallingConvention.td"

//===----------------------------------------------------------------------===//
// NVM Subtarget features                                                     //
//===----------------------------------------------------------------------===//

def FeatureCmp      : SubtargetFeature<"cmp", "HasCmp", "true", "Enable 'cmp' instructions.">;
def FeatureSlt      : SubtargetFeature<"slt", "HasSlt", "true", "Enable 'slt' instructions.">;
def FeatureNVM		: SubtargetFeature<"NVM", "Cpu0ArchVersion", "NVM", "NVM ISA Support (slt)", [FeatureCmp, FeatureSlt]>;

//===----------------------------------------------------------------------===//
// Cpu0 processors supported.
//===----------------------------------------------------------------------===//

def GenericItineraries : ProcessorItineraries<[], [], []>; 

  
//class Proc<string Name, list<SubtargetFeature> Features> : Processor<Name, NVMGenericItineraries, Features>;
class Proc<string Name, list<SubtargetFeature> Features> : Processor<Name, GenericItineraries, Features>;

def : Proc<"NVM", [FeatureNVM]>;



def dNVM : Target 
{
  // Pull in Instruction Info:
//  let InstructionSet = ARMInstrInfo;
}

