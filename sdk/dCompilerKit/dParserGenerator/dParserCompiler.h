/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __dParserCompiler_h__
#define __dParserCompiler_h__

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <tchar.h>
#include <crtdbg.h>
#include <sys/types.h>
#include <sys/stat.h>


#include <dTree.h>
#include <dList.h>
#include <dArray.h>
#include <dCRC.h>
#include <dString.h>
#include <dContainersStdAfx.h>

//#define D_WRITE_STATE_TRANSITION_GRAPH

class dParserLexical;

class dParserCompiler
{
	public:
	enum dToken
	{
		OR = 256,
		COLOM,
		SIMICOLOM,
		PREC,
		TOKEN,
		UNION,
		LEFT,
		RIGHT,
		START,
		LITERAL,
		EXPECT,
		INTEGER,
		CODE_BLOCK,
		SEMANTIC_ACTION,
		GRAMMAR_SEGMENT,
		PARCEL_ERROR,
	};

	enum dTokenType;
	enum ActionType;
	
//	class dItem;
	class dState;
	class dGoto;
	class dAction;
//	class dSymbol;
//	class dRuleInfo;
	class dTokenInfo;
	class dGotoEntry;
	class ActionEntry;
	class dTransition;
	class dActionEntry;
	class dSentenceSymbol;
//	class dProductionRule;
	class dTokenStringPair;
	class dOperatorsPrecedence;
	class dOperatorsAssociation;

	template<class T>
	class dCache: public dArray<T*>
	{
		public:
		dCache()
			:dArray<T*>(1024)
		{
			m_count = 0;
		}

		int GetCount() const
		{
			return m_count;
		}

		void Push(T* const node)
		{
			(*this)[m_count] = node;
			m_count++;
		}

		T* Pop()
		{
			m_count--;
			return (*this)[m_count];
		}

		int m_count;
	};


	class dSymbolName
	{
		class dDictionary: public dTree<int, dString>
		{
			public:
			dDictionary()
				:dTree<int, dString>()
			{
				m_empty = FindWord(dString(""));
			}

			dTreeNode* FindWord(const dString& word)
			{
				dDictionary::dTreeNode* node = Find(word);
				if (!node) {
					node = Insert(0, word);
				}
				return node;
			}
			dTreeNode* m_empty;
		};

		public:
		dSymbolName()
			:m_name(GetDictionary().m_empty)
		{
		}

		dSymbolName(const char* const word)
			:m_name(GetDictionary().FindWord(dString(word)))
		{
		}

		dSymbolName(const dString& word)
			:m_name(GetDictionary().FindWord(word))
		{
		}

		const dString& GetString() const
		{
			dAssert(m_name);
			return m_name->GetKey();
		}

		const char* GetStr() const
		{
			return GetString().GetStr();
		}

		bool operator== (const dSymbolName& src) const
		{
			return m_name == src.m_name;
		}

		bool operator!= (const dSymbolName& src) const
		{
			return m_name != src.m_name;
		}

		bool operator< (const dSymbolName& src) const
		{
			return m_name < src.m_name;
		}

		bool operator>(const dSymbolName& src) const
		{
			return m_name > src.m_name;
		}

		private:
		dDictionary& GetDictionary()
		{
			static dDictionary dictionary;
			return dictionary;
		}

		const dDictionary::dTreeNode* m_name;
	};

	class dSymbol
	{
		public:
		dTokenType m_type;
		dToken m_token;
		dSymbolName m_name;
		dString m_operatorPrecendeceOverrride;
	};

	class dRuleInfo: public dSymbol, public dList<dSymbol>
	{
		public:
		dRuleInfo()
			:dSymbol()
			,dList<dSymbol>()
			,m_ruleId()
			,m_ruleNumber(0)
			,m_ruleReduced(false)
			,m_shiftReduceErrorMark(false)
			,m_semanticActionCode("")
		{
		}

		dListNode* GetSymbolNodeByIndex(int index) const
		{
			int i = 0;
			for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
				if (index == i) {
					return node;
				}
				i++;
			}
			return NULL;
		}

		dSymbolName m_ruleId;
		int m_ruleNumber;
		bool m_ruleReduced;
		bool m_shiftReduceErrorMark;
		dString m_semanticActionCode;
	};


	class dProductionRule: public dList<dRuleInfo>
	{
		public:
		dListNode* Find(dSymbolName name) const
		{
			for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
				if (node->GetInfo().m_name == name) {
					return node;
				}
			}

			dAssert(0);
			return NULL;
		}
	};


	class dItem
	{
		public:
		dItem()
			:m_indexMarker(0)
			,m_lookAheadSymbol()
			,m_lastOperatorSymbol()
			,m_ruleNode(NULL)
		{
		}

		int m_indexMarker;
		dSymbolName m_lookAheadSymbol;
		dSymbolName m_lastOperatorSymbol;
		dProductionRule::dListNode* m_ruleNode;
	};

	class dSymbolNameMap: public dTree<int, dSymbolName>
	{
		public:
		~dSymbolNameMap();
		dTreeNode* Insert(int, dSymbolName);
		private:
		static dCache<dTreeNode> m_cache;
	};

	class dItemList: public dList<dItem>
	{
		public:

		~dItemList();
		dListNode* Append();
		dListNode* Append(const dItem &element);

		private:
		static dCache<dListNode> m_cache;
	};


	dParserCompiler(const dString& inputRules, const char* const outputFileName, const char* const scannerClassName);
	~dParserCompiler();

	protected:
	dString GetClassName(const char* const fileName) const;
	void LoadTemplateFile(const char* const fileName, dString& output) const;
	void SaveFile(const char* const fileName, const char* const extention, const dString& input) const;

	void ReplaceMacro (dString& data, const dString& newName, const dString& macro) const;
	void ReplaceAllMacros (dString& data, const dString& newName, const dString& macro) const;

	void ScanGrammarFile(const dString& inputRules, dProductionRule& rules, dTree<dTokenInfo, dSymbolName>& symbolList, dOperatorsPrecedence& operatorPrecence,
						 dString& userCodeBlock, dString& userVariableClass, dString& endUserCode, int& lastTokenEnum);
	dToken ScanGrammarRule(dParserLexical& lexical, dProductionRule& rules, dTree<dTokenInfo, dSymbolName>& symbolList, int& ruleNumber, int& tokenEnumeration, const dOperatorsPrecedence& operatorPrecence);
	bool DoesSymbolDeriveEmpty (dSymbolName symbol, const dTree<dList<void*>, dSymbolName>& ruleMap) const ;
	void First (dSymbolName symbol, dSymbolNameMap& symbolListMark, const dTree<dTokenInfo, dSymbolName>& symbolList, const dTree<dList<void*>, dSymbolName>& ruleMap, dSymbolNameMap& firstSetOut) const;
	void First (const dList<dSymbolName>& symbolSet, const dTree<dTokenInfo, dSymbolName>& symbolList, const dTree<dList<void*>, dSymbolName>& ruleMap, dSymbolNameMap& firstSetOut) const;
	dState* Goto (const dState* const state, const dSymbolName& symbol, const dTree<dTokenInfo, dSymbolName>& symbolList, const dTree<dList<void*>, dSymbolName>& ruleMap) const;
	dState* Closure (const dItemList& itemSet, const dTree<dTokenInfo, dSymbolName>& symbolList, const dTree<dList<void*>, dSymbolName>& ruleMap) const;
	void BuildParsingTable (const dTree<dState*, dSymbolName>& stateList, const dSymbolName& startSymbol, const dOperatorsPrecedence& operatorPrecence) const;
	void CanonicalItemSets (dTree<dState*, dSymbolName>& states, const dProductionRule& rules, const dTree<dTokenInfo, dSymbolName>& symbolList, const dOperatorsPrecedence& operatorPrecence, const dString& fileName);

	void GenerateHeaderFile (const dString& className, const dString& scannerClassName, const char* const outputFileName, 
							 const dTree<dTokenInfo, dSymbolName>& symbolList, const dString& userVariableClass);
	void GenerateParserCode (const dString& className, const dString& scannerClassName, const char* const outputFileName, 
							 const dTree<dTokenInfo, dSymbolName>& symbolList, dTree<dState*, dSymbolName>& stateList, const dString& userCode, const dString& endUserCode, int lastTerminalTokenEnum);

	void DisplayError (const char* format, ...) const;

	mutable int m_shiftReduceExpectedWarnings;
};


#endif

// TODO: reference additional headers your program requires here

