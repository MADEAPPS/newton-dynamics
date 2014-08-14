/*===- TableGen'erated file -------------------------------------*- C++ -*-===*\
|*                                                                            *|
|*Subtarget Enumeration Source Fragment                                       *|
|*                                                                            *|
|* Automatically generated file, do not edit!                                 *|
|*                                                                            *|
\*===----------------------------------------------------------------------===*/


#ifdef GET_SUBTARGETINFO_ENUM
#undef GET_SUBTARGETINFO_ENUM
namespace llvm {
namespace dNVM {
enum {
  FeatureCmp =  1ULL << 0,
  FeatureNVM =  1ULL << 1,
  FeatureSlt =  1ULL << 2
};
}
} // End llvm namespace 
#endif // GET_SUBTARGETINFO_ENUM


#ifdef GET_SUBTARGETINFO_MC_DESC
#undef GET_SUBTARGETINFO_MC_DESC
namespace llvm {
// Sorted (by key) array of values for CPU features.
extern const llvm::SubtargetFeatureKV dNVMFeatureKV[] = {
  { "NVM", "NVM ISA Support (slt)", dNVM::FeatureNVM, dNVM::FeatureCmp | dNVM::FeatureSlt },
  { "cmp", "Enable 'cmp' instructions.", dNVM::FeatureCmp, 0ULL },
  { "slt", "Enable 'slt' instructions.", dNVM::FeatureSlt, 0ULL }
};

// Sorted (by key) array of values for CPU subtype.
extern const llvm::SubtargetFeatureKV dNVMSubTypeKV[] = {
  { "NVM", "Select the NVM processor", dNVM::FeatureNVM, 0ULL }
};

#ifdef DBGFIELD
#error "<target>GenSubtargetInfo.inc requires a DBGFIELD macro"
#endif
#ifndef NDEBUG
#define DBGFIELD(x) x,
#else
#define DBGFIELD(x)
#endif

// ===============================================================
// Data tables for the new per-operand machine model.

// {ProcResourceIdx, Cycles}
extern const llvm::MCWriteProcResEntry dNVMWriteProcResTable[] = {
  { 0,  0}, // Invalid
}; // dNVMWriteProcResTable

// {Cycles, WriteResourceID}
extern const llvm::MCWriteLatencyEntry dNVMWriteLatencyTable[] = {
  { 0,  0}, // Invalid
}; // dNVMWriteLatencyTable

// {UseIdx, WriteResourceID, Cycles}
extern const llvm::MCReadAdvanceEntry dNVMReadAdvanceTable[] = {
  {0,  0,  0}, // Invalid
}; // dNVMReadAdvanceTable

static const llvm::MCSchedModel NoSchedModel(
  MCSchedModel::DefaultIssueWidth,
  MCSchedModel::DefaultMicroOpBufferSize,
  MCSchedModel::DefaultLoopMicroOpBufferSize,
  MCSchedModel::DefaultLoadLatency,
  MCSchedModel::DefaultHighLatency,
  MCSchedModel::DefaultMispredictPenalty,
  0, // PostRAScheduler
  1, // CompleteModel
  0, // Processor ID
  0, 0, 0, 0, // No instruction-level machine model.
  0); // No Itinerary

// Sorted (by key) array of itineraries for CPU subtype.
extern const llvm::SubtargetInfoKV dNVMProcSchedKV[] = {
  { "NVM", (const void *)&NoSchedModel }
};
#undef DBGFIELD
static inline void InitdNVMMCSubtargetInfo(MCSubtargetInfo *II, StringRef TT, StringRef CPU, StringRef FS) {
  II->InitMCSubtargetInfo(TT, CPU, FS, dNVMFeatureKV, dNVMSubTypeKV, 
                      dNVMProcSchedKV, dNVMWriteProcResTable, dNVMWriteLatencyTable, dNVMReadAdvanceTable, 0, 0, 0);
}

} // End llvm namespace 
#endif // GET_SUBTARGETINFO_MC_DESC


#ifdef GET_SUBTARGETINFO_TARGET_DESC
#undef GET_SUBTARGETINFO_TARGET_DESC
#include "llvm/Support/Debug.h"
// ParseSubtargetFeatures - Parses features string setting specified
// subtarget options.
void llvm::dNVMSubtarget::ParseSubtargetFeatures(StringRef CPU, StringRef FS) {
  DEBUG(dbgs() << "\nFeatures:" << FS);
  DEBUG(dbgs() << "\nCPU:" << CPU << "\n\n");
  InitMCProcessorInfo(CPU, FS);
  uint64_t Bits = getFeatureBits();
  if ((Bits & dNVM::FeatureCmp) != 0) HasCmp = true;
  if ((Bits & dNVM::FeatureNVM) != 0 && Cpu0ArchVersion < NVM) Cpu0ArchVersion = NVM;
  if ((Bits & dNVM::FeatureSlt) != 0) HasSlt = true;
}
#endif // GET_SUBTARGETINFO_TARGET_DESC


#ifdef GET_SUBTARGETINFO_HEADER
#undef GET_SUBTARGETINFO_HEADER
namespace llvm {
class DFAPacketizer;
struct dNVMGenSubtargetInfo : public TargetSubtargetInfo {
  explicit dNVMGenSubtargetInfo(StringRef TT, StringRef CPU, StringRef FS);
public:
  unsigned resolveSchedClass(unsigned SchedClass, const MachineInstr *DefMI, const TargetSchedModel *SchedModel) const override;
  DFAPacketizer *createDFAPacketizer(const InstrItineraryData *IID) const;
};
} // End llvm namespace 
#endif // GET_SUBTARGETINFO_HEADER


#ifdef GET_SUBTARGETINFO_CTOR
#undef GET_SUBTARGETINFO_CTOR
#include "llvm/CodeGen/TargetSchedule.h"
namespace llvm {
extern const llvm::SubtargetFeatureKV dNVMFeatureKV[];
extern const llvm::SubtargetFeatureKV dNVMSubTypeKV[];
extern const llvm::SubtargetInfoKV dNVMProcSchedKV[];
extern const llvm::MCWriteProcResEntry dNVMWriteProcResTable[];
extern const llvm::MCWriteLatencyEntry dNVMWriteLatencyTable[];
extern const llvm::MCReadAdvanceEntry dNVMReadAdvanceTable[];
dNVMGenSubtargetInfo::dNVMGenSubtargetInfo(StringRef TT, StringRef CPU, StringRef FS)
  : TargetSubtargetInfo() {
  InitMCSubtargetInfo(TT, CPU, FS, makeArrayRef(dNVMFeatureKV, 3), makeArrayRef(dNVMSubTypeKV, 1), 
                      dNVMProcSchedKV, dNVMWriteProcResTable, dNVMWriteLatencyTable, dNVMReadAdvanceTable, 
                      0, 0, 0);
}

unsigned dNVMGenSubtargetInfo
::resolveSchedClass(unsigned SchedClass, const MachineInstr *MI, const TargetSchedModel *SchedModel) const {
  report_fatal_error("Expected a variant SchedClass");
} // dNVMGenSubtargetInfo::resolveSchedClass
} // End llvm namespace 
#endif // GET_SUBTARGETINFO_CTOR

