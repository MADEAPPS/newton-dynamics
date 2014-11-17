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

#ifndef _dConstantPropagationSolver_H_
#define _dConstantPropagationSolver_H_


#include "dCILstdafx.h"
#include "dCIL.h"
#include "dCILInstr.h"
#include "dBasicBlocksGraph.h"


class dConstantPropagationSolver
{
	public:
	class dInstructionMap: public dTree<int, dCILInstr*>
	{
		public:
		dInstructionMap ()
			:dTree<int, dCILInstr*>()
		{
		}
	};

	class dVariable
	{
		public:
		enum dValueTypes
		{
			m_constant,
			m_undefined,
			m_variableValue,
		};

		dVariable(dCILInstr* const instruction, dCILInstr::dArg* const variable)
			:m_type(m_undefined)
			,m_constValue ("")
			,m_instruction(instruction)
			,m_variable(variable)
		{
		}

		dVariable& operator= (const dVariable& copy)
		{
			dAssert(0);
			return *this;
		}

		dValueTypes m_type;
		dString m_constValue;
		dCILInstr::dArg* m_variable;
		dCILInstr* const m_instruction;
	};

	class dBlockEdgeKey
	{
		public:
		dBlockEdgeKey(const dBasicBlock* const blockHigh, const dBasicBlock* const blockLow)
			:m_blockHigh(blockHigh)
			, m_blockLow(blockLow)
		{
		}

		bool operator< (const dBlockEdgeKey& src) const
		{
			if (m_blockHigh < src.m_blockHigh) {
				return true;
			}
			return m_blockLow < src.m_blockLow;
		}

		bool operator>(const dBlockEdgeKey& src) const
		{
			if (m_blockHigh > src.m_blockHigh) {
				return true;
			}
			return m_blockLow > src.m_blockLow;
		}

		const dBasicBlock* m_blockHigh;
		const dBasicBlock* m_blockLow;
	};


	dConstantPropagationSolver (dBasicBlocksGraph* const graph);
	bool Solve();
	void UpdateLatice (const dCILInstr::dArg& arg, const dString& value, dVariable::dValueTypes type);

	dBasicBlocksGraph* m_graph; 
	dTree<int, dBasicBlock*> m_visited;
	dTree<dInstructionMap, dString> m_uses;
	dInstructionMap m_instructionsWorklist;
	dTree<int, dBasicBlock*> m_blockWorklist;
	dTree<dVariable, dString> m_variablesList;
	dTree<int, dBlockEdgeKey> m_executableEdges;
};

#endif