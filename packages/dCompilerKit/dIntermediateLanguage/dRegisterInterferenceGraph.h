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

#ifndef _dRegisterInterferenceGraph_H_
#define _dRegisterInterferenceGraph_H_


#include "dCILstdafx.h"
#include "dTreeAdressStmt.h"


class dDataFlowGraph;
class dRegisterInterferenceNode;
class dRegisterInterferenceNodeEdge
{
	public:
	dRegisterInterferenceNodeEdge (dTree<dRegisterInterferenceNode, dString>::dTreeNode* const targetNode)
		:m_isMov(false)
		,m_mark (0)
		,m_twin(NULL)
		,m_targetNode(targetNode)
	{
	}

	bool m_isMov;
	int m_mark;
	dRegisterInterferenceNodeEdge* m_twin;
	dTree<dRegisterInterferenceNode, dString>::dTreeNode* m_targetNode;
};

class dRegisterInterferenceNode
{
	public: 
	dRegisterInterferenceNode()
		:m_inSet (false)
		,m_registerIndex (-1)
	{
	}

	bool m_inSet;
	int m_registerIndex;
	dString m_name;
	dList<dRegisterInterferenceNodeEdge> m_interferanceEdge;
};


	class dRegisterInterferenceGraph: public dTree<dRegisterInterferenceNode, dString>
	{
		public: 
		dRegisterInterferenceGraph (dDataFlowGraph* const flowGraph, int registerCount);

		private:
		void Build();
		dTreeNode* GetBestNode();
		void EmitSpillStatements();
		void AllocateRegisters ();
		int ColorGraph (int registerCount);
		

		void SaveRegisterToTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument);
		void LoadRegisterFromTemp(dCIL::dListNode* const node, dTreeAdressStmt::dArg& argument);

		

		int GetRegisterIndex (const dString& varName) const;
		dString GetRegisterName (const dString& varName) const;

		dString MakeSpilledTemporary ();
		dString MakeLocalVariable (int index);
		
/*
		void SortRegistersByFrequency (int totalRegisters, int registerCount);
		void AllocatedSpilledRegister(int totalRegisters, int registerCount);

		void RemapRegister(dTreeAdressStmt::dArg& arg, int totalRegisters);
		void SortRegisters(int totalRegisters);
		int FindRegister (int regIndex, int totalRegisters) const;

		
		void SaveRegisterToTemp(dCIL::dListNode* const node, const dString& reg, const dString& local);

		
		void LoadRegisterFromTemp(dCIL::dListNode* const node, const dString& reg, const dString& local);

		void LoadSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument, int registerBase);
		void SaveSpillRegister(dCIL::dListNode* const node, const dString& alliasRegister, dTreeAdressStmt::dArg& argument, int registerBase);
		static int Compare (const void* p1, const void* p2);
		

		dString m_reg0;
		dString m_reg1;
		dString m_reg2;
		dString m_local0;
		dString m_local1;
		dString m_local2;
*/
	dDataFlowGraph* m_flowGraph;
	int m_spillCount;
	int m_localLayer;
	int m_registerCount;
//		int m_registenFrequency[1024][2];
};


#endif