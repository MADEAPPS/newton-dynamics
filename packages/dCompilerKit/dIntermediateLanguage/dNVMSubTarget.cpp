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
#include "dNVMSubTarget.h"


dNVMSubTarget::dNVMSubTarget(llvm::StringRef TT, llvm::StringRef CPU, llvm::StringRef FS, const llvm::TargetMachine &targetMachine, const llvm::TargetOptions &options)
	:llvm::TargetSubtargetInfo()
	,m_dataLayout ("e-p:32:32:32-i8:8:32-i16:16:32-i32:32:32-i64:64:64-f32:32:64-f64:64:64")
{
}

dNVMSubTarget::~dNVMSubTarget()
{
}


const llvm::DataLayout* dNVMSubTarget::getDataLayout() const 
{ 
	return &m_dataLayout; 
}

bool dNVMSubTarget::enableMachineScheduler() const
{
	return TargetSubtargetInfo::enableMachineScheduler();
}

