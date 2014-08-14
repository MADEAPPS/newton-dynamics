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


#ifndef _dNVMSubTarget_H_
#define _dNVMSubTarget_H_

#include "dCILstdafx.h"



class dNVMSubTarget: public llvm::TargetSubtargetInfo
{
	public:
	dNVMSubTarget (llvm::StringRef TT, llvm::StringRef CPU, llvm::StringRef FS, const llvm::TargetMachine &targetMachine, const llvm::TargetOptions &options);
	~dNVMSubTarget();


	const llvm::DataLayout *getDataLayout() const; 
	bool enableMachineScheduler() const;


	const llvm::TargetInstrInfo *getInstrInfo() const 
	{ 
		dAssert (0);
		return nullptr; 
	}

	const llvm::TargetFrameLowering *getFrameLowering() const 
	{
		dAssert (0);
		return nullptr;
	}

	const llvm::TargetLowering *getTargetLowering() const 
	{ 
		dAssert (0);
		return nullptr; 
	}

	const llvm::TargetSelectionDAGInfo *getSelectionDAGInfo() const 
	{
		dAssert (0);
		return nullptr;
	}


	const llvm::TargetRegisterInfo *getRegisterInfo() const 
	{ 
		dAssert (0);
		return nullptr; 
	}

	llvm::TargetJITInfo *getJITInfo() 
	{ 
		dAssert (0);
		return nullptr; 
	}

	const llvm::InstrItineraryData *getInstrItineraryData() const 
	{
		dAssert (0);
		return nullptr;
	}

	unsigned resolveSchedClass(unsigned SchedClass, const llvm::MachineInstr *MI, const llvm::TargetSchedModel* SchedModel) const 
	{
		dAssert (0);
		return 0;
	}

	

	bool enablePostMachineScheduler() const
	{
		dAssert (0);
		return 0;
	}

	bool enableAtomicExpandLoadLinked() const
	{
		dAssert (0);
		return 0;
	}


	void overrideSchedPolicy (llvm::MachineSchedPolicy &Policy, llvm::MachineInstr *begin, llvm::MachineInstr *end, unsigned NumRegionInstrs) const 
	{
		dAssert (0);
	}

	void adjustSchedDependency (llvm::SUnit *def, llvm::SUnit *use, llvm::SDep& dep) const 
	{ 
	}

	AntiDepBreakMode getAntiDepBreakMode() const 
	{
		dAssert (0);
		return ANTIDEP_NONE;
	}

	void getCriticalPathRCs(RegClassVector &CriticalPathRCs) const 
	{
		dAssert (0);
		return CriticalPathRCs.clear();
	}

	llvm::CodeGenOpt::Level getOptLevelToEnablePostRAScheduler() const 
	{
		dAssert (0);
		return llvm::CodeGenOpt::Default;
	}

	virtual bool enableRALocalReassignment (llvm::CodeGenOpt::Level OptLevel) const
	{
		dAssert (0);
		return 0;
	}

	virtual bool useAA() const
	{
		dAssert (0);
		return 0;
	}

	bool enableEarlyIfConversion() const 
	{ 
		dAssert (0);
		return false; 
	}

	void resetSubtargetFeatures(const llvm::MachineFunction *MF) 
	{ 
		dAssert (0);
	}


	llvm::DataLayout m_dataLayout;
};




#endif
