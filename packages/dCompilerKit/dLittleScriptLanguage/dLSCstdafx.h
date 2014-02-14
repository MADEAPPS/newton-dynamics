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


#ifndef __dLSCstdafx_H_
#define __dLSCstdafx_H_

#include <io.h>
#include <direct.h>



#include <dCIL.h>
#include <dCRC.h>
#include <dTree.h>
#include <dList.h>
#include <dRtti.h>
#include <dString.h>
#include <dCILstdafx.h>
#include <dRefCounter.h>
//#include <dDataFlowGraph.h>
#include <dVirtualMachine.h>
//#include <dTreeAdressStmt.h>
#include <dContainersStdAfx.h>


#ifdef _MSC_VER
#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

//#pragma warning (disable: 4201)	// warning C4201: nonstandard extension used : nameless struct/union
#pragma warning (disable: 4100)		// 'O' : unreferenced formal parameter
#pragma warning (disable: 4244)		//'argument' : conversion from 'unsigned int' to 'unsigned short', possible loss of data
#pragma warning (disable: 4480)		// nonstandard extension used: specifying underlying type for enum ''
#pragma warning (disable: 4355)		//'this' : used in base member initializer list
#pragma warning (disable: 4800)		//'unsigned int' : forcing value to bool 'true' or 'false' (performance warning)
#pragma warning (disable: 4512)		//'llvm::Type' : assignment operator could not be generated

//4275
//4146
//4180
//4267
//4345
//4351
//4503
//4624
//4291
#endif

#include "llvm/IR/Verifier.h"
#include "llvm/ExecutionEngine/GenericValue.h"
#include "llvm/ExecutionEngine/Interpreter.h"
#include "llvm/ExecutionEngine/JIT.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/raw_ostream.h"



#define D_DEBUG_PARCEL


#endif