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

// dParserCompiler.cpp : Defines the entry point for the console application.
//

#include "dParserCompiler.h"
#include "dParserLexical.h"

#define DACCEPT_SYMBOL "$$$"
#define DACCEPTING_TOKEN 254	

#define DDEBUG_STATES

#define DERROR_SYMBOL	"error"
#define DERROR_TOKEN	255



//The Parcel input file consists of three sections, separated by a line containing only `%%'. 
//
//	definitions
//	%%
//	production rules
//	%%
//	user code

enum dParserCompiler::dTokenType
{
	TERMINAL,
	NONTERMINAL
};


enum dParserCompiler::ActionType
{
	dSHIFT = 0,
	dREDUCE,
	dACCEPT,
	dERROR
};

class dParserCompiler::dTokenInfo
{
	public:
	dTokenInfo (int tokenId, dTokenType type, const dString& name)
		:m_tokenId (tokenId), m_type(type), m_name(name)
	{
	}

	int m_tokenId;
	dTokenType m_type;
	dString m_name;
};

class dParserCompiler::dGotoEntry
{
	public:
	dGotoEntry ()
	{
	}
	short  m_token;
	short  m_nextState;
};


class dParserCompiler::dActionEntry
{
	public:
	dActionEntry ()
	{
	}

	short m_token;
	char m_errorRule;
	char m_stateType;// 0 = shift, 1 = reduce, 2 = accept
	short m_nextState;
	short m_ruleSymbols;
	short m_ruleIndex;
};



class dParserCompiler::dSymbol
{
	public:
	dTokenType m_type;
	dToken m_token;
	dString m_name;
	dString m_operatorPrecendeceOverright;
	dCRCTYPE m_nameCRC;
};


class dParserCompiler::dRuleInfo: public dParserCompiler::dSymbol, public dList<dParserCompiler::dSymbol>
{
	public:
	dRuleInfo()
		:dSymbol()
		,dList<dSymbol>()
		,m_ruleId(0)
		,m_ruleNumber(0)
		,m_ruleReduced(false)
		,m_shiftReduceErrorMark(false)
		,m_semanticActionCode("")
	{
	}

	dListNode* GetSymbolNodeByIndex (int index) const
	{
		int i = 0;
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			if (index == i) {
				return node;
			}
			i ++;
		}
		return NULL;
	}

	dCRCTYPE m_ruleId;
	int m_ruleNumber;
	bool m_ruleReduced;
	bool m_shiftReduceErrorMark;
	dString m_semanticActionCode;

};


class dParserCompiler::dProductionRule: public dList<dParserCompiler::dRuleInfo>
{
	public:

	dListNode* Find (dCRCTYPE nameCRC) const
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			if (node->GetInfo().m_nameCRC == nameCRC) {
				return node;
			}
		}

		_ASSERTE (0);
		return NULL;
	}
};

class dParserCompiler::dTransition
{
	public:
	dString m_name;
	dCRCTYPE m_symbol;
	dTokenType m_type;
	dState* m_targetState;
};

class dParserCompiler::dItem
{
	public:
	dItem ()
		:m_indexMarker(0), m_lookAheadSymbolCRC(0), m_lastOperatorSymbolCRC(0), m_lookAheadSymbolName(""), m_lastOperatorSymbolName (""), m_ruleNode(NULL)
	{
	}

	int m_indexMarker;
	
	dCRCTYPE m_lookAheadSymbolCRC;
	dCRCTYPE m_lastOperatorSymbolCRC;
	dString m_lookAheadSymbolName;
	dString m_lastOperatorSymbolName;
	dProductionRule::dListNode* m_ruleNode;
};

class dParserCompiler::dAction 
{
	public:
	ActionType m_type;
	int m_nextState;
	dItem* m_myItem;
	dProductionRule::dListNode* m_reduceRuleNode;
};

class dParserCompiler::dTokenStringPair 
{
	public:
	dToken m_token;
	dString m_info;
};


class dParserCompiler::dState: public dList<dParserCompiler::dItem>
{
	public:
	class dItemKey
	{
		public:
		dItemKey (const dItem& item)
			:m_lookAheadSymbolCRC (item.m_lookAheadSymbolCRC), m_rule (item.m_ruleNode)
		{
		}

		dItemKey (dCRCTYPE symbol, dProductionRule::dListNode* rule)
			:m_lookAheadSymbolCRC (symbol), m_rule (rule)
		{
		}

		bool operator< (const dItemKey& key) const
		{	
			if (m_lookAheadSymbolCRC < key.m_lookAheadSymbolCRC) {
				return true;	
			} else if (m_lookAheadSymbolCRC == key.m_lookAheadSymbolCRC){
				if (m_rule < key.m_rule) {
					return true;	
				} 
			}
			return false;
		}

		bool operator> (const dItemKey& key) const
		{
			if (m_lookAheadSymbolCRC > key.m_lookAheadSymbolCRC) {
				return true;	
			} else if (m_lookAheadSymbolCRC == key.m_lookAheadSymbolCRC){
				if (m_rule > key.m_rule) {
					return true;	
				} 
			}
			return false;
		}

		dCRCTYPE m_lookAheadSymbolCRC;
		dProductionRule::dListNode* m_rule;
	};

	dState (const dList<dItem>& itemSet)
		:m_key(0), m_number(0), m_hasErroItem(false), m_goto(), m_actions(), m_transitions()
	{
		for (dListNode* node = itemSet.GetFirst(); node; node = node->GetNext()) {
			AddItem (node->GetInfo());
		}
	}

	void AddItem (const dItem& item)
	{
		dState::dListNode* const node = Append(item);

		static dCRCTYPE errorCRC = dCRC64 (DERROR_SYMBOL);

		dItemKey key (item);
		dTree<dList<dState::dListNode*>, dItemKey>::dTreeNode* mapNode = m_itemMap.Find (key);
		if (!mapNode) {
			mapNode = m_itemMap.Insert (key);
		}

		if (item.m_indexMarker == 0) {
			_ASSERTE (item.m_ruleNode);
			const dRuleInfo& ruleInfo = item.m_ruleNode->GetInfo();
			if (ruleInfo.GetCount()) {
				const dSymbol& symbol = ruleInfo.GetFirst()->GetInfo();
				if (symbol.m_nameCRC == errorCRC) {
					m_hasErroItem = true;
				}
			}

		}

		dList<dState::dListNode*>& bucket = mapNode->GetInfo();
		// check if bucket is not too big
		_ASSERTE (bucket.GetCount() < 16);
		bucket.Append(node);
	}

	dCRCTYPE GetKey() const
	{
		return m_key;
	}

	void CalculateKey ()
	{
		dCRCTYPE key = 0;
		for (dState::dListNode* itemNode = GetFirst(); itemNode; itemNode = itemNode->GetNext()) {
			dItem& item = itemNode->GetInfo();
			int index = 0;
			key = dCRC64 (item.m_lookAheadSymbolName.GetStr(), key);
			for (dRuleInfo::dListNode* node = item.m_ruleNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
				if (index == item.m_indexMarker) {
					key = dCRC64 (".", key);
				}
				const dSymbol& info = node->GetInfo();
				key = dCRC64 (info.m_name.GetStr(), key);
				index ++;
			}
			if (item.m_indexMarker == item.m_ruleNode->GetInfo().GetCount()) {
				key = dCRC64 (".", key);
			}
		}
		m_key = key;
	}

	dListNode* FindItem (dProductionRule::dListNode* const rule, int marker, dCRCTYPE lookAheadSymbol) const
	{
		dItemKey key (lookAheadSymbol, rule);
		dTree<dList<dState::dListNode*>, dItemKey>::dTreeNode* const mapNode = m_itemMap.Find (key);
		if (mapNode) {
			dList<dState::dListNode*>& bucket = mapNode->GetInfo();
				for (dList<dState::dListNode*>::dListNode* node = bucket.GetFirst(); node; node = node->GetNext()) {
					dState::dListNode* const nodeItem = node->GetInfo();
					const dItem& item = nodeItem->GetInfo();
				if (item.m_indexMarker == marker) {
					_ASSERTE (item.m_ruleNode == rule);
						_ASSERTE (item.m_lookAheadSymbolCRC == lookAheadSymbol);
						return nodeItem;
					}
				}
			}
		return NULL;
	}


	void Trace(FILE* const debugFile) const
	{
		fprintf (debugFile, "state %d:\n", m_number);
		for (dState::dListNode* itemNode = GetFirst(); itemNode; itemNode = itemNode->GetNext()) {
			dItem& item = itemNode->GetInfo();
			fprintf (debugFile, "%s -> ", item.m_ruleNode->GetInfo().m_name.GetStr());

			int index = 0;
			bool hasIndex = false;
			dRuleInfo::dListNode* node = item.m_ruleNode->GetInfo().GetFirst();
			for (; node; node = node->GetNext()) {

				if (index == item.m_indexMarker) {
					fprintf (debugFile, ". ");
					hasIndex = true;
					break;
				}
				const dSymbol& info = node->GetInfo();
				fprintf (debugFile, "%s ", info.m_name.GetStr());
				index ++;
			}

			for (; node; node = node->GetNext()) {
				const dSymbol& info = node->GetInfo();
				fprintf (debugFile, "%s ", info.m_name.GetStr());
			}
			if (!hasIndex) {
				fprintf (debugFile, ". ");
			}
			fprintf (debugFile, ":: %s\n", item.m_lookAheadSymbolName.GetStr());
		}
		fprintf (debugFile, "\n");
	}

	dCRCTYPE m_key;
	int m_number;
	bool m_hasErroItem;
	dTree<dState*, dCRCTYPE> m_goto; 
	dTree<dAction, dCRCTYPE> m_actions; 
	dList<dTransition> m_transitions;
	dTree<dList<dState::dListNode*>, dItemKey> m_itemMap;
};

class dParserCompiler::dOperatorsAssociation: public dList <dCRCTYPE>
{
	public:
	enum dAssoctivity
	{
		m_left,
		m_right
	};

	bool FindOperator(dCRCTYPE symbol) const
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			if (symbol == node->GetInfo()) {
				return true;
			}
		}
		return false;
	}

	int m_prioprity;
	dAssoctivity m_associativity;
};

class dParserCompiler::dOperatorsPrecedence: public dList <dOperatorsAssociation>
{
	public:
	const dOperatorsAssociation* FindAssociation (dCRCTYPE symbol)	const
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			const dOperatorsAssociation& association = node->GetInfo();
			if (association.FindOperator(symbol)) {
				return &node->GetInfo();
			}
		}

		return NULL;
	}

	void SaveLastOperationSymbol (const dState* const state) const
	{
		for (dState::dListNode* node = state->GetFirst(); node; node = node->GetNext()) {
			dItem& item = node->GetInfo();
			const dRuleInfo& ruleInfo = item.m_ruleNode->GetInfo();
			for (dRuleInfo::dListNode* infoSymbolNode = ruleInfo.GetFirst(); infoSymbolNode; infoSymbolNode = infoSymbolNode->GetNext()) {
				const dSymbol& infoSymbol = infoSymbolNode->GetInfo();
				if (infoSymbol.m_operatorPrecendeceOverright.Size()) {
					dCRCTYPE crc = dCRC64(infoSymbol.m_operatorPrecendeceOverright.GetStr());
					if (FindAssociation(crc)) {
						item.m_lastOperatorSymbolCRC = crc;
						item.m_lastOperatorSymbolName = infoSymbol.m_operatorPrecendeceOverright;
					}

				} else {
					if (FindAssociation(infoSymbol.m_nameCRC)) {
						item.m_lastOperatorSymbolCRC = infoSymbol.m_nameCRC;
						item.m_lastOperatorSymbolName = infoSymbol.m_name;
					}
				}
			}
		}
	}

};

dParserCompiler::dParserCompiler(const dString& inputRules, const char* const outputFileName, const char* const scannerClassName)
	:m_shiftReduceExpectedWarnings(0)
{
	// scan the grammar into a list of rules.
	int lastTerminalToken;
	dProductionRule ruleList;
	dOperatorsPrecedence operatorPrecedence;
	dTree<dTokenInfo, dCRCTYPE> symbolList;
	dString userCodeBlock;
	dString userVariableClass ("");
	dString endUserCode ("\n");

	// scan grammar to a set of LR(1) rules
	symbolList.Insert(dTokenInfo (DACCEPTING_TOKEN, TERMINAL, DACCEPT_SYMBOL), dCRC64 (DACCEPT_SYMBOL));
	symbolList.Insert(dTokenInfo (DERROR_TOKEN, TERMINAL, DERROR_SYMBOL), dCRC64 (DERROR_SYMBOL));
	ScanGrammarFile(inputRules, ruleList, symbolList, operatorPrecedence, userCodeBlock, userVariableClass, endUserCode, lastTerminalToken);

	// convert the rules into a NFA.
	dString debugFileName (outputFileName);
	debugFileName += ".txt";
	FILE* const debugFile = fopen (debugFileName.GetStr(), "w");

	dTree<dState*, dCRCTYPE> stateList;
	CanonicalItemSets (stateList, ruleList, symbolList, operatorPrecedence, debugFile);
	fclose (debugFile);


	// create a LR(1) parsing table from the NFA graphs
	const dString& startSymbol = ruleList.GetFirst()->GetInfo().m_name;
	BuildParsingTable (stateList, dCRC64 (startSymbol.GetStr()), operatorPrecedence);


	//Write Parser class and header file
	dString className (GetClassName(outputFileName));
	GenerateHeaderFile (className, scannerClassName, outputFileName, symbolList, userVariableClass);
	GenerateParserCode (className, scannerClassName, outputFileName, symbolList, stateList, userCodeBlock, endUserCode, lastTerminalToken);

	dTree<dState*, dCRCTYPE>::Iterator iter(stateList);
	for (iter.Begin(); iter; iter ++) {
		dState* const state = iter.GetNode()->GetInfo();
		delete state;
	}
}


dParserCompiler::~dParserCompiler()
{
}


void dParserCompiler::DisplayError (const char* format, ...) const
{
	va_list v_args;
	char* const text = (char*) malloc (strlen (format) + 2048);

	text[0] = 0;
	va_start (v_args, format);     
	vsprintf(text, format, v_args);
	va_end (v_args);            

	fprintf (stderr, text);
#ifdef _MSC_VER  
	OutputDebugStringA (text);
#endif	

	free (text);
}


dString dParserCompiler::GetClassName(const char* const fileName) const
{
	char className[256];
	const char* ptr = strrchr (fileName, '/');
	if (ptr) {
		ptr ++;
	} else {
		ptr = strrchr (fileName, '\\');
		if (ptr) {
			ptr ++;
		} else {
			ptr = fileName;
		}
	}
	strcpy (className, ptr);
	strtok (className, ".");
	return dString (className);
}




void dParserCompiler::LoadTemplateFile(const char* const templateName, dString& templateOuput) const
{
	char path[2048];

	// in windows
	GetModuleFileName(NULL, path, sizeof(path)); 

	char* const ptr = strrchr (path, '\\') + 1;
	sprintf (ptr, templateName);

	FILE* const templateFile = fopen (path, "rb");
	_ASSERTE (templateFile);

	templateOuput.LoadFile(templateFile);
	fclose (templateFile);	
}

void dParserCompiler::SaveFile(const char* const fileName, const char* const extention, const dString& input) const
{
	char path[2048];

	strcpy (path, fileName);
	char* const ptr1 = strrchr (path, '.');
	if (ptr1) {
		*ptr1 = 0;
	}
	strcat (path, extention);
	FILE* const headerFile = fopen (path, "wb");
	_ASSERTE (headerFile);
	fprintf (headerFile, "%s", input.GetStr());
	fclose (headerFile);
}



void dParserCompiler::ScanGrammarFile(
	const dString& inputRules, 
	dProductionRule& ruleList, 
	dTree<dTokenInfo, dCRCTYPE>& symbolList, 
	dOperatorsPrecedence& operatorPrecedence,
	dString& userCodeBlock,
	dString& userVariableClass,
	dString& endUserCode,
	int& lastTokenEnum)
{
	dString startSymbol ("");
	int tokenEnumeration = 256;
	int operatorPrecedencePriority = 0;

	dParserLexical lexical (inputRules.GetStr());
	LoadTemplateFile("dParserUserVariableTemplate_cpp.txt", userVariableClass);

	// scan the definition segment
	for (dToken token = dToken(lexical.NextToken()); token != GRAMMAR_SEGMENT; ) {
		switch (int (token)) 
		{
			case START:
			{
				token = dToken(lexical.NextToken());
				startSymbol = lexical.GetTokenString();
				token = dToken(lexical.NextToken());
				break;
			}

			case TOKEN:
			{
				for (token = dToken(lexical.NextToken()); token == LITERAL; token = dToken(lexical.NextToken())) {
					const char* const name = lexical.GetTokenString();
					symbolList.Insert(dTokenInfo (tokenEnumeration, TERMINAL, name), dCRC64 (name));
					tokenEnumeration ++;
				}
				break;
			}

			case LEFT:
			case RIGHT:
			{
				dOperatorsAssociation& association = operatorPrecedence.Append()->GetInfo();
				association.m_prioprity = operatorPrecedencePriority;
				operatorPrecedencePriority ++;
				switch (int (token))
				{
					case LEFT:
						association.m_associativity = dOperatorsAssociation::m_left;
						break;
					case RIGHT:
						association.m_associativity = dOperatorsAssociation::m_right;
						break;
				}

				for (token = dToken(lexical.NextToken()); (token == LITERAL) || ((token < 256) && !isalnum (token)); token = dToken(lexical.NextToken())) {
					association.Append(dCRC64 (lexical.GetTokenString()));
				}
				break;
			}


			case UNION:
			{
				token = dToken(lexical.NextToken());
				_ASSERTE (token == SEMANTIC_ACTION);
				userVariableClass = lexical.GetTokenString() + 1;
				userVariableClass.Replace(userVariableClass.Size() - 1, 1, "");
				token = dToken(lexical.NextToken());
				break;
			}


			case CODE_BLOCK:
			{
				userCodeBlock += lexical.GetTokenString();
				token = dToken(lexical.NextToken());
				break;
			}

			case EXPECT:
			{
				token = dToken(lexical.NextToken());
				_ASSERTE (token == INTEGER);
				m_shiftReduceExpectedWarnings = atoi (lexical.GetTokenString());
				token = dToken(lexical.NextToken());
				break;
			}

			default:;
			{
				_ASSERTE (0);
				token = dToken(lexical.NextToken());
			}
		}
	}


	int ruleNumber = 1;
	lastTokenEnum = tokenEnumeration;

	// scan the production rules segment
	dToken token1 = dToken(lexical.NextToken());
	for (; (token1 != GRAMMAR_SEGMENT) && (token1 != -1); token1 = dToken(lexical.NextToken())) {
		//dTrace (("%s\n", lexical.GetTokenString()));
		switch (int (token1)) 
		{
			case LITERAL:
			{
				// add the first Rule;
				dRuleInfo& rule = ruleList.Append()->GetInfo();
				rule.m_token = token1;
				rule.m_type = NONTERMINAL;
				rule.m_name = lexical.GetTokenString();
				rule.m_nameCRC = dCRC64 (lexical.GetTokenString());

				dTree<dTokenInfo, dCRCTYPE>::dTreeNode* nonTerminalIdNode = symbolList.Find(rule.m_nameCRC);
				if (!nonTerminalIdNode) {
					nonTerminalIdNode = symbolList.Insert(dTokenInfo (tokenEnumeration, NONTERMINAL, rule.m_name), rule.m_nameCRC);
					tokenEnumeration ++;
				}
				rule.m_ruleId = nonTerminalIdNode->GetInfo().m_tokenId;
				rule.m_ruleNumber = ruleNumber;
				ruleNumber ++;

				token1 = ScanGrammarRule(lexical, ruleList, symbolList, ruleNumber, tokenEnumeration, operatorPrecedence); 
				break;
			}
			default:
				_ASSERTE (0);
		}
	}

	dProductionRule::dListNode* firtRuleNode = ruleList.GetFirst();	
	if (startSymbol != "") {
		firtRuleNode = ruleList.Find (dCRC64 (startSymbol.GetStr()));	
	}

	//Expand the Grammar Rule by adding an empty start Rule;
	const dRuleInfo& firstRule = firtRuleNode->GetInfo();

	dRuleInfo& rule = ruleList.Addtop()->GetInfo();
	rule.m_ruleNumber = 0;
	rule.m_ruleId = tokenEnumeration;
	rule.m_token = firstRule.m_token;
	rule.m_type = NONTERMINAL;
	rule.m_name = firstRule.m_name + dString("__");
	rule.m_nameCRC = dCRC64 (rule.m_name.GetStr());
	symbolList.Insert(dTokenInfo (tokenEnumeration, rule.m_type, rule.m_name), rule.m_nameCRC);
	tokenEnumeration ++;
	
	dSymbol& symbol = rule.Append()->GetInfo();
	symbol.m_token = firstRule.m_token;
	symbol.m_type = firstRule.m_type;
	symbol.m_name = firstRule.m_name;
	symbol.m_nameCRC = firstRule.m_nameCRC;

	// scan literal use code
	if (token1 == GRAMMAR_SEGMENT) {
		endUserCode = lexical.GetNextBuffer();
		//endUserCode += "\n";
	}
}


dParserCompiler::dToken dParserCompiler::ScanGrammarRule(
	dParserLexical& lexical, 
	dProductionRule& rules, 
	dTree<dTokenInfo, dCRCTYPE>& symbolList, 
	int& ruleNumber,
	int& tokenEnumeration,
	const dOperatorsPrecedence& operatorPrecedence)
{
	dRuleInfo* currentRule = &rules.GetLast()->GetInfo();
	dToken token = dToken(lexical.NextToken());
	do {
		
		dList<dTokenStringPair> ruleTokens;
		for (token = dToken(lexical.NextToken()); !((token == SIMICOLOM) || (token == OR)); token = dToken(lexical.NextToken())) {
			_ASSERTE (token != -1);

			dTokenStringPair& pair = ruleTokens.Append()->GetInfo();
			pair.m_token = token;
			pair.m_info = lexical.GetTokenString();
		}
		
		dList<dTokenStringPair>::dListNode* lastNode = ruleTokens.GetLast();
		if (lastNode) {
			if (lastNode->GetInfo().m_token != SEMANTIC_ACTION) {
				lastNode = NULL;
			} else {
				currentRule->m_semanticActionCode = lastNode->GetInfo().m_info;
			}
		}
		for (dList<dTokenStringPair>::dListNode* node = ruleTokens.GetFirst(); node != lastNode; node = node->GetNext()) {
			dTokenStringPair& pair = node->GetInfo();

			if (pair.m_token == LITERAL) {
				dSymbol& symbol = currentRule->Append()->GetInfo();
				symbol.m_token = pair.m_token;
				symbol.m_name = pair.m_info;
				symbol.m_nameCRC = dCRC64 (symbol.m_name.GetStr());

				dTree<dTokenInfo, dCRCTYPE>::dTreeNode* symbolNode = symbolList.Find(symbol.m_nameCRC);
				if (!symbolNode) {
					symbolNode = symbolList.Insert(dTokenInfo (tokenEnumeration, NONTERMINAL, symbol.m_name), symbol.m_nameCRC);
					tokenEnumeration ++;
				}
				symbol.m_type = symbolNode->GetInfo().m_type;

			} else if (pair.m_token < 256) {
				_ASSERTE (pair.m_info.Size() == 1);
				dSymbol& symbol = currentRule->Append()->GetInfo();
				symbol.m_name = pair.m_info;
				symbol.m_nameCRC = dCRC64 (symbol.m_name.GetStr());

				symbol.m_type = TERMINAL;
				symbol.m_token = LITERAL;
				symbolList.Insert(dTokenInfo (pair.m_token, TERMINAL, symbol.m_name), symbol.m_nameCRC);

			} else if (pair.m_token == PREC) {
				node = node->GetNext();
				for (dRuleInfo::dListNode* ruleNode = currentRule->GetLast(); ruleNode; ruleNode = ruleNode->GetPrev()) {
					dSymbol& symbol = ruleNode->GetInfo();
					if (operatorPrecedence.FindAssociation (symbol.m_nameCRC)) {
						dTokenStringPair& pair = node->GetInfo();		
						symbol.m_operatorPrecendeceOverright = pair.m_info;
						break;
					}
				}
//			} else if (pair.m_token != SEMANTIC_ACTION) {
//				// no user action allowed in the middle of a sentence
//				_ASSERTE (pair.m_token == SEMANTIC_ACTION);
//			} else {
//				_ASSERTE (0);
			}
		}

		if (token == OR) {
			// this is a rule with multiples sentences alternates, add new rule with the same name Non terminal
			dRuleInfo& rule = rules.Append()->GetInfo();
			rule.m_ruleNumber = ruleNumber;
			ruleNumber ++;
			rule.m_ruleId = currentRule->m_ruleId;
			rule.m_token = currentRule->m_token;
			rule.m_type = NONTERMINAL;
			//rule.m_name += currentRule->m_name;
			rule.m_name = currentRule->m_name;
			rule.m_nameCRC = currentRule->m_nameCRC;
			currentRule = &rule;
		}

	} while (token != SIMICOLOM);

	return token;
}


// generates the canonical Items set for a LR(1) grammar
void dParserCompiler::CanonicalItemSets (
	dTree<dState*, dCRCTYPE>& stateMap, 
	const dProductionRule& ruleList, 
	const dTree<dTokenInfo, dCRCTYPE>& symbolList, 
	const dOperatorsPrecedence& operatorPrecedence,
	FILE* const debugFile)
{
	dList<dItem> itemSet;
	dList<dState*> stateList;

	// start by building an item set with only the first rule
	dItem& item = itemSet.Append()->GetInfo();
	item.m_indexMarker = 0;
	item.m_lookAheadSymbolCRC = dCRC64 (DACCEPT_SYMBOL);
	item.m_lookAheadSymbolName = DACCEPT_SYMBOL;
	item.m_ruleNode = ruleList.GetFirst();

	// build a rule info map
	dTree<dList<void*>, dCRCTYPE> ruleMap;
	for (dProductionRule::dListNode* ruleNode = ruleList.GetFirst(); ruleNode; ruleNode = ruleNode->GetNext()) {
		dRuleInfo& info = ruleNode->GetInfo();

		dTree<dList<void*>, dCRCTYPE>::dTreeNode* node = ruleMap.Find(info.m_nameCRC);
		if (!node) {
			node = ruleMap.Insert(info.m_nameCRC);
		}
		dList<void*>& entry = node->GetInfo();
		entry.Append(ruleNode);
	}


	// find the closure for the first this item set with only the first rule
	dState* const state = Closure (itemSet, symbolList, ruleMap);
	operatorPrecedence.SaveLastOperationSymbol (state);

	stateMap.Insert(state, state->GetKey());
	stateList.Append(state);

	state->Trace(debugFile);

	// now for each state found 
	int stateNumber = 1;
	for (dList<dState*>::dListNode* node = stateList.GetFirst(); node; node = node->GetNext()) {
		dState* const state = node->GetInfo();

		dTree<dTokenInfo, dCRCTYPE>::Iterator iter (symbolList);
		for (iter.Begin(); iter; iter ++) {

			dCRCTYPE symbol = iter.GetKey();
			dState* const newState = Goto (state, symbol, symbolList, ruleMap);

			if (newState->GetCount()) {
				const dTokenInfo& tokenInfo = iter.GetNode()->GetInfo();
				dTransition& transition = state->m_transitions.Append()->GetInfo();
				transition.m_symbol = symbol;
				transition.m_name = tokenInfo.m_name; 
				transition.m_type = tokenInfo.m_type;
				_ASSERTE (transition.m_symbol == dCRC64(transition.m_name.GetStr()));
				transition.m_targetState = newState;

				dTree<dState*, dCRCTYPE>::dTreeNode* const targetStateNode = stateMap.Find(newState->GetKey());
				if (!targetStateNode) {
					newState->m_number = stateNumber;

					stateNumber ++;
					stateMap.Insert(newState, newState->GetKey());
					newState->Trace(debugFile);
					stateList.Append(newState);

					operatorPrecedence.SaveLastOperationSymbol (newState);

				} else {
					transition.m_targetState = targetStateNode->GetInfo();
					delete newState;
				}
			} else {
				delete newState;
			}
		}

		dTrace (("state#:%d   items: %d   transitions: %d\n", state->m_number, state->GetCount(), state->m_transitions.GetCount()));
	}
}


// Generate the closure for a Set of Item  
dParserCompiler::dState* dParserCompiler::Closure (
	const dList<dItem>& itemSet, 
	const dTree<dTokenInfo, dCRCTYPE>& symbolList,
	const dTree<dList<void*>, dCRCTYPE>& ruleMap) const
{
	dState* const state = new dState (itemSet);
	for (dState::dListNode* itemNode = state->GetFirst(); itemNode; itemNode = itemNode->GetNext()) {
		dItem& item = itemNode->GetInfo();

		dRuleInfo::dListNode* const symbolNode = item.m_ruleNode->GetInfo().GetSymbolNodeByIndex (item.m_indexMarker);
		if (symbolNode) {
			// get Beta token dString
			const dRuleInfo& rule = item.m_ruleNode->GetInfo();
			dRuleInfo::dListNode* ruleNode = rule.GetFirst();
			for (int i = 0; i < item.m_indexMarker; i ++) {
				ruleNode = ruleNode->GetNext();
			}

			dList<dCRCTYPE> firstSymbolList;
			for (ruleNode = ruleNode->GetNext(); ruleNode; ruleNode = ruleNode->GetNext()) {
				const dSymbol& symbol = ruleNode->GetInfo();
				firstSymbolList.Append(symbol.m_nameCRC);
			}

			firstSymbolList.Append(item.m_lookAheadSymbolCRC);
			const dSymbol& sentenceSymbol = symbolNode->GetInfo();

			dTree<dList<void*>, dCRCTYPE>::dTreeNode* const ruleNodes = ruleMap.Find(sentenceSymbol.m_nameCRC);
			if (ruleNodes) {
				const dList<void*>& matchingRulesList = ruleNodes->GetInfo();
				_ASSERTE (matchingRulesList);
				for (dList<void*>::dListNode* node = matchingRulesList.GetFirst(); node; node = node->GetNext()) {
					dProductionRule::dListNode* const ruleNode = (dProductionRule::dListNode*) node->GetInfo();
					_ASSERTE (ruleNode->GetInfo().m_name == sentenceSymbol.m_name);
					dTree<int, dCRCTYPE> firstList;
					First (firstSymbolList, symbolList, ruleMap, firstList);
					dTree<int, dCRCTYPE>::Iterator firstIter (firstList);
					for (firstIter.Begin(); firstIter; firstIter ++) {
						dCRCTYPE symbol = firstIter.GetKey();
						dState::dListNode* const itemNode = state->FindItem(ruleNode, 0, symbol);
						if (!itemNode) {
							dItem newItem;
							newItem.m_indexMarker = 0;
							newItem.m_ruleNode = ruleNode;
							newItem.m_lookAheadSymbolCRC = symbol;
							newItem.m_lookAheadSymbolName = symbolList.Find (symbol)->GetInfo().m_name;
							state->AddItem(newItem);
						}
					}
				}
			}
		}
	}

	state->CalculateKey ();

	return state;
}


void dParserCompiler::First (
	const dList<dCRCTYPE>& symbolSet, 
	const dTree<dTokenInfo, dCRCTYPE>& symbolList, 
	const dTree<dList<void*>, dCRCTYPE>& ruleMap,
	dTree<int, dCRCTYPE>& firstSetOut) const
{
	if (symbolSet.GetCount() > 1) {

		dList<dCRCTYPE>::dListNode* node = symbolSet.GetFirst();
		bool deriveEmpty = true;
		while ((deriveEmpty) && node) {
			dCRCTYPE symbol = node->GetInfo();
			node = node->GetNext();

			dTree<int, dCRCTYPE> tmpFirst;
			dTree<int, dCRCTYPE> symbolListMark; 
			First (symbol, symbolListMark, symbolList, ruleMap, tmpFirst);
			dTree<int, dCRCTYPE>::Iterator iter (tmpFirst);
			deriveEmpty = false;  
			for (iter.Begin(); iter; iter ++) {
				dCRCTYPE symbol = iter.GetKey();
				if (symbol == 0) {
					deriveEmpty = true;  
				} else {
					firstSetOut.Insert(0, symbol);
				}
			}
		}
		if (deriveEmpty) {
			firstSetOut.Insert(0, 0);
		}

	} else  {
		dCRCTYPE symbol = symbolSet.GetFirst()->GetInfo();
		dTree<int, dCRCTYPE> symbolListMark; 
		First (symbol, symbolListMark, symbolList, ruleMap, firstSetOut);
	}
}


void dParserCompiler::First (
	dCRCTYPE symbol, 
	dTree<int, dCRCTYPE>& symbolListMark, 
	const dTree<dTokenInfo, dCRCTYPE>& symbolList, 
	const dTree<dList<void*>, dCRCTYPE>& ruleMap,
	dTree<int, dCRCTYPE>& firstSetOut) const
{
	if (symbolListMark.Find(symbol)) {
		return;
	}
	symbolListMark.Insert(0, symbol);

	dTree<dTokenInfo, dCRCTYPE>::dTreeNode* const node = symbolList.Find(symbol);
	_ASSERTE (node);
	if (node->GetInfo().m_type == TERMINAL) {
		firstSetOut.Insert(0, symbol);
	} else if (DoesSymbolDeriveEmpty (symbol, ruleMap)) {
		firstSetOut.Insert(0, 0);
	} else {

		dTree<dList<void*>, dCRCTYPE>::dTreeNode* const ruleNodes = ruleMap.Find(symbol);
		if (ruleNodes) {
			const dList<void*>& matchingRulesList = ruleNodes->GetInfo();
			for (dList<void*>::dListNode* node = matchingRulesList.GetFirst(); node; node = node->GetNext()) {
				dProductionRule::dListNode* const ruleInfoNode = (dProductionRule::dListNode*) node->GetInfo();
				const dRuleInfo& info = ruleInfoNode->GetInfo();

				bool allDeriveEmpty = true;
				for (dRuleInfo::dListNode* sentenceSymbolNode = info.GetFirst(); sentenceSymbolNode; sentenceSymbolNode = sentenceSymbolNode->GetNext()) {
					const dSymbol& sentenceSymnol = sentenceSymbolNode->GetInfo();
					if (!DoesSymbolDeriveEmpty (sentenceSymnol.m_nameCRC, ruleMap)) {
						allDeriveEmpty = false;
						dTree<int, dCRCTYPE> newFirstSetOut;
						First (sentenceSymnol.m_nameCRC, symbolListMark, symbolList, ruleMap, newFirstSetOut);
						dTree<int, dCRCTYPE>::Iterator iter (newFirstSetOut);
						for (iter.Begin(); iter; iter ++) {
							dCRCTYPE symbol = iter.GetKey();
							_ASSERTE (symbol != 0);
							_ASSERTE (symbolList.Find(symbol)->GetInfo().m_type == TERMINAL);
							firstSetOut.Insert(0, symbol);
						}
						break;
					}
				}
				if (allDeriveEmpty) {
					dTrace (("this could be a bug here, I am not sure if I should closure with the accepting state or not, need more debugging\n"))
//					firstSetOut.Insert(0, 0);
				}
			}
		}
	}
}


bool dParserCompiler::DoesSymbolDeriveEmpty (dCRCTYPE symbol, const dTree<dList<void*>, dCRCTYPE>& ruleMap) const 
{
	dTree<dList<void*>, dCRCTYPE>::dTreeNode* const ruleNodes = ruleMap.Find(symbol);
	if (ruleNodes) {
		//const dList<void*>& matchingRulesList = ruleMap.Find(symbol)->GetInfo();
		const dList<void*>& matchingRulesList = ruleNodes->GetInfo();
		for (dList<void*>::dListNode* node = matchingRulesList.GetFirst(); node; node = node->GetNext()) {
			dProductionRule::dListNode* const ruleInfoNode = (dProductionRule::dListNode*) node->GetInfo();

			const dRuleInfo& info = ruleInfoNode->GetInfo();
			if (symbol == info.m_nameCRC) {
				if (!info.GetCount()) {
					return true;
				}
			}
		}
	}
	return false;
}


// generates the got state for this symbol
dParserCompiler::dState* dParserCompiler::Goto (
	const dState* const state, 
	dCRCTYPE symbol, 
	const dTree<dTokenInfo, dCRCTYPE>& symbolList,
	const dTree<dList<void*>, dCRCTYPE>& ruleMap) const
{
	dList<dItem> itemSet;

	// iterate over each item contained on state
	for (dState::dListNode* node = state->GetFirst(); node; node = node->GetNext()) {
		const dItem& item = node->GetInfo();

		int index = 0;
		const dRuleInfo& ruleInfo = item.m_ruleNode->GetInfo();
		// for this item get it rule, and iterate over ever rule symbol.  		
		for (dRuleInfo::dListNode* infoSymbolNode = ruleInfo.GetFirst(); infoSymbolNode; infoSymbolNode = infoSymbolNode->GetNext()) {
			// see if this rule symbol match symbol
			const dSymbol& infoSymbol = infoSymbolNode->GetInfo();
			if (infoSymbol.m_nameCRC == symbol) {
				// a symbol was found, now see if it is at the right position in the rule
				if (index == item.m_indexMarker) {
					// the rule match symbol at the right position, add this rule to the new item set.
					dItem& newItem = itemSet.Append()->GetInfo();
					newItem.m_indexMarker = index + 1;
					newItem.m_ruleNode = item.m_ruleNode;
					newItem.m_lookAheadSymbolCRC = item.m_lookAheadSymbolCRC;
					newItem.m_lookAheadSymbolName = item.m_lookAheadSymbolName;
					break;
				}
			}
			index ++;
		}
	}

	//find the closure for this new item set.
	dState* const closureState = Closure (itemSet, symbolList, ruleMap);
	return closureState;
}





void dParserCompiler::ReplaceMacro (dString& data, const dString& newName, const dString& macro) const
{
	int size = int(macro.Size());
	int position = int (data.Find (macro));
	_ASSERTE (position != -1);
	data.Replace(position, size, newName);
}

void dParserCompiler::ReplaceAllMacros (dString& data, const dString& newName, const dString& macro) const
{
	int size = int (macro.Size());
	for (int i = data.Find (macro); i != -1; i = data.Find (macro)) {
		data.Replace(i, size, newName);
	}
}


void dParserCompiler::GenerateHeaderFile (
	const dString& className, 
	const dString& scannerClassName, 
	const char* const outputFileName, 
	const dTree<dTokenInfo, dCRCTYPE>& symbolList, 
	const dString& userVariableClass) 
{
	dString templateHeader;
	LoadTemplateFile("dParserTemplate_h.txt", templateHeader);

	ReplaceAllMacros (templateHeader, className, "$(className)");
	ReplaceAllMacros (templateHeader, scannerClassName, "$(scannerClass)");
	ReplaceMacro (templateHeader, userVariableClass, "$(userVariableClass)");


	dTree<dTree<dTokenInfo, dCRCTYPE>::dTreeNode*, int> sortToken;
	dTree<dTokenInfo, dCRCTYPE>::Iterator iter (symbolList);
	for (iter.Begin(); iter; iter ++) {
		const dTokenInfo& info = iter.GetNode()->GetInfo();
		if ((info.m_type == TERMINAL) && (info.m_tokenId >= 256)) {
			sortToken.Insert(iter.GetNode(), info.m_tokenId);
		}
	} 

	dTree<dTree<dTokenInfo, dCRCTYPE>::dTreeNode*, int>::Iterator iter1 (sortToken);
	bool first = true;
	char text[256];
	sprintf (text, " = %d, \n", DACCEPTING_TOKEN);
	dString enumdTokens ("\t\tACCEPTING_TOKEN");
	enumdTokens += text;

	sprintf (text, " = %d, \n", DERROR_TOKEN);
	enumdTokens += "\t\tERROR_TOKEN";
	enumdTokens += text;

	for (iter1.Begin(); iter1; iter1 ++) {
		dTree<dTokenInfo, dCRCTYPE>::dTreeNode* const node = iter1.GetNode()->GetInfo();
		//const dString& name = node->GetKey();
		const dString& name = node->GetInfo().m_name;
		enumdTokens += "\t\t";
		enumdTokens += name;
		if (first) {
			first = false;
			sprintf (text, " = %d, \n", iter1.GetKey());
			enumdTokens += text;
		} else {
			enumdTokens += ",\n";
		}
	}

	enumdTokens.Replace(enumdTokens.Size()-2, 2, "");
	ReplaceMacro (templateHeader, enumdTokens, "$(Tokens)");

	SaveFile(outputFileName, ".h", templateHeader);
}


void dParserCompiler::GenerateParserCode (
	const dString& className, 
	const dString& scannerClassName, 
	const char* const outputFileName, 
	const dTree<dTokenInfo, dCRCTYPE>& symbolList, 
	dTree<dState*, dCRCTYPE>& stateList, 
	const dString& userCode, 
	const dString& endUserCode,
	int lastTerminalTokenEnum)
{
	dString templateHeader ("");
	LoadTemplateFile("dParserTemplate _cpp.txt", templateHeader);

	int position = templateHeader.Find ("$(userCode)");
	templateHeader.Replace(position, 11, userCode);

	ReplaceAllMacros (templateHeader, className, "$(className)");
	ReplaceAllMacros (templateHeader, scannerClassName, "$(scannerClass)");


	char text[256];
	sprintf (text, "%d", lastTerminalTokenEnum);
	ReplaceMacro (templateHeader, text, "&(lastTerminalToken)");

	dTree<dState*, dCRCTYPE> sortedStates;
	dTree<dState*, dCRCTYPE>::Iterator stateIter (stateList);
	for (stateIter.Begin(); stateIter; stateIter ++) {
		dState* const state = stateIter.GetNode()->GetInfo();
		sortedStates.Insert(state, state->m_number);
	}

	dTree<int, dString> actionFilter;

	dString emptySematicAction ("");
	dString stateActionsStart ("");
	dString stateActionsCount ("");
	dString nextActionsStateList ("");
	dString sematicActions ("");
	int entriesCount = 0;

	int newLineCount = 0;
	int starAndCountIndex = 0;
	dTree<dState*, dCRCTYPE>::Iterator sortStateIter (sortedStates);


	const char* const caseTabs0 = "\t\t\t\t\t\t";
	//const char* const caseTabs1 = "\t\t\t\t\t\t\t";
	for (sortStateIter.Begin(); sortStateIter; sortStateIter ++) {
		dState* const state = sortStateIter.GetNode()->GetInfo();

		int count = 0;
		dTree<dActionEntry, int> actionSort;
		dTree<dAction, dCRCTYPE>::Iterator actionIter (state->m_actions);
		for (actionIter.Begin(); actionIter; actionIter++) {
			count ++;

			dAction& action = actionIter.GetNode()->GetInfo();
			if (action.m_type == dSHIFT) {
				dCRCTYPE actionSymbol = actionIter.GetKey();
				_ASSERTE (symbolList.Find(actionSymbol));

				dActionEntry entry;
				entry.m_stateType = char (action.m_type);
				entry.m_errorRule = state->m_hasErroItem ? 1 : 0;
				entry.m_ruleIndex = 0;
				entry.m_ruleSymbols = 0;
				entry.m_nextState = short (action.m_nextState);
				entry.m_token = short (symbolList.Find(actionSymbol)->GetInfo().m_tokenId);

				actionSort.Insert (entry, entry.m_token);

			} else if (action.m_type == dREDUCE) {

				dCRCTYPE actionSymbol = actionIter.GetKey();
				_ASSERTE (symbolList.Find(actionSymbol));

				dRuleInfo& reduceRule = action.m_reduceRuleNode->GetInfo();
				_ASSERTE (symbolList.Find(reduceRule.m_nameCRC));
				_ASSERTE (symbolList.Find(reduceRule.m_nameCRC)->GetInfo().m_tokenId >= 256);

				dActionEntry entry;
				entry.m_stateType = char (action.m_type);
				entry.m_errorRule = 0; //state->m_hasErroItem ? 1 : 0;
				entry.m_ruleIndex = short (reduceRule.m_ruleNumber);
				entry.m_ruleSymbols = short (reduceRule.GetCount());
				entry.m_nextState = short (symbolList.Find(reduceRule.m_nameCRC)->GetInfo().m_tokenId - lastTerminalTokenEnum);
				entry.m_token = short (symbolList.Find(actionSymbol)->GetInfo().m_tokenId);
				actionSort.Insert (entry, entry.m_token);

				if (!reduceRule.m_ruleReduced && (reduceRule.m_semanticActionCode != emptySematicAction)) {
					// issue a sematic action code;

					reduceRule.m_ruleReduced = true;
					char text[128];
					dString userSematicAction (reduceRule.m_semanticActionCode);
					int symbolsCount = int (entry.m_ruleSymbols);
					for (int i = 0; i < symbolsCount; i ++) {

						sprintf (text, "%d", symbolsCount - i);
						dString macro ("$");
						macro += text;

						sprintf (text, "%d", symbolsCount - i - 1);
						dString macroVariable ("parameter[");
						macroVariable += text;
						macroVariable += "].m_value";
						ReplaceAllMacros (userSematicAction, macroVariable, macro);
					}
					ReplaceAllMacros (userSematicAction, "entry.m_value", "$$");

					sprintf (text, "%d:", reduceRule.m_ruleNumber);
					sematicActions += caseTabs0;
					sematicActions += "case "; 
					sematicActions += text; 
					//sematicActions += "// rule ";
					sematicActions += "// ";
					sematicActions += reduceRule.m_name;
					sematicActions += " : ";
					for (dRuleInfo::dListNode* node = reduceRule.GetFirst(); node; node = node->GetNext()) {
						sematicActions+= node->GetInfo().m_name;
						sematicActions += " ";
					}
					sematicActions += "\n";
					sematicActions += userSematicAction;
					sematicActions += "\nbreak;\n\n";
				}

			} else {
				_ASSERTE (action.m_type == dACCEPT);

				dActionEntry entry;
				entry.m_stateType = char (action.m_type);
				entry.m_errorRule = 0; //state->m_hasErroItem ? 1 : 0;
				entry.m_ruleIndex = 0;
				entry.m_ruleSymbols = 0;
				entry.m_nextState = 0;
				entry.m_token = DACCEPTING_TOKEN;
				actionSort.Insert (entry, entry.m_token);
			}
		}

		int actionIndex = entriesCount;
		dString stateActions ("");
		dTree<dActionEntry, int>::Iterator iter (actionSort);
		for (iter.Begin(); iter; iter ++) {
			const dActionEntry& entry = iter.GetNode()->GetInfo();
			sprintf (text, "%d, %d, %d, %d, %d, %d, ", entry.m_token, entry.m_errorRule, entry.m_stateType, entry.m_nextState, entry.m_ruleSymbols, entry.m_ruleIndex);
			stateActions += text; 
			entriesCount ++;
		}

		dTree<int, dString>::dTreeNode* const stateActionNode = actionFilter.Find(stateActions);
		if (stateActionNode) {
			entriesCount = actionIndex;
			actionIndex =  stateActionNode->GetInfo();
		} else {
			actionFilter.Insert(actionIndex, stateActions);

			for (iter.Begin(); iter; iter ++) {
				if (newLineCount % 4 == 0) {
					nextActionsStateList += "\n\t\t\t";
				}
				newLineCount ++;
				const dActionEntry& entry = iter.GetNode()->GetInfo();
				sprintf (text, "dActionEntry (%d, %d, %d, %d, %d, %d), ", entry.m_token, entry.m_errorRule, entry.m_stateType, entry.m_nextState, entry.m_ruleSymbols, entry.m_ruleIndex);
				nextActionsStateList += text; 
			}
		}

		if ((starAndCountIndex % 24) == 0) {
			stateActionsStart += "\n\t\t\t";
			stateActionsCount += "\n\t\t\t";
		}
		starAndCountIndex ++;

		sprintf (text, "%d, ", actionIndex);
		stateActionsStart += text;

		sprintf (text, "%d, ", count);
		stateActionsCount += text;
	}
	nextActionsStateList.Replace(nextActionsStateList.Size()-2, 2, "");
	stateActionsCount.Replace(stateActionsCount.Size()-2, 2, "");
	stateActionsStart.Replace(stateActionsStart.Size()-2, 2, "");

	ReplaceMacro (templateHeader, stateActionsCount, "$(actionsCount)");
	ReplaceMacro (templateHeader, stateActionsStart, "$(actionsStart)");
	ReplaceMacro (templateHeader, nextActionsStateList, "$(actionTable)");

	ReplaceMacro (templateHeader, sematicActions, "$(semanticActionsCode)");


	dString stateGotoStart ("");
	dString stateGotoCount ("");
	dString nextGotoStateList ("");
	entriesCount = 0;
	int newLine = 0;
	int gotoStateCount = 0;
	for (sortStateIter.Begin(); sortStateIter; sortStateIter ++) {

		char text[256];
		dState* const state = sortStateIter.GetNode()->GetInfo();

		int currentEntryuCount = entriesCount;

		int count = 0;
		dTree<dState*, dCRCTYPE>::Iterator gotoIter (state->m_goto); 
		dTree<dTree<dState*, dCRCTYPE>::dTreeNode*, int> sortGotoActions;
		for (gotoIter.Begin(); gotoIter; gotoIter++) {
			int id = symbolList.Find(gotoIter.GetKey())->GetInfo().m_tokenId;
			sortGotoActions.Insert(gotoIter.GetNode(), id);
		}

		dTree<dTree<dState*, dCRCTYPE>::dTreeNode*, int>::Iterator iter1 (sortGotoActions);
		for (iter1.Begin(); iter1; iter1++) {
			count ++;
			if ((newLine % 5) == 0) {
				nextGotoStateList += "\n\t\t\t";
			}
			newLine ++;

			dTree<dState*, dCRCTYPE>::dTreeNode* const node = iter1.GetNode()->GetInfo();
			dState* const targetState = node->GetInfo();

			dGotoEntry entry;
			entry.m_nextState = short (targetState->m_number);
			entry.m_token = short(iter1.GetKey());

			sprintf (text, "dGotoEntry (%d, %d), ", entry.m_token, entry.m_nextState);
			nextGotoStateList += text;
			entriesCount ++;
		}

		if ((gotoStateCount % 24) == 0) {
			stateGotoStart += "\n\t\t\t";
			stateGotoCount += "\n\t\t\t";
		}
		gotoStateCount ++;

		sprintf (text, "%d, ", currentEntryuCount);
		stateGotoStart += text;

		sprintf (text, "%d, ", count);
		stateGotoCount += text;
	}

	nextGotoStateList.Replace(nextGotoStateList.Size()-2, 2, "");
	stateGotoCount.Replace(stateGotoCount.Size()-2, 2, "");
	stateGotoStart.Replace(stateGotoStart.Size()-2, 2, "");

	ReplaceMacro (templateHeader, stateGotoCount, "$(gotoCount)");
	ReplaceMacro (templateHeader, stateGotoStart, "$(gotoStart)");
	ReplaceMacro (templateHeader, nextGotoStateList, "$(gotoTable)");

	templateHeader += endUserCode;
	SaveFile(outputFileName, ".cpp", templateHeader);
}



void dParserCompiler::BuildParsingTable (
	const dTree<dState*, dCRCTYPE>& stateList, 
	dCRCTYPE startSymbol, 
	const dOperatorsPrecedence& operatorPrecedence) const
{
	dTree<dState*, dCRCTYPE>::Iterator stateIter (stateList);

	unsigned emptySymbol = 0;

	const dCRCTYPE acceptingSymbol = dCRC64 (DACCEPT_SYMBOL);
	// create Shift Reduce action table
	for (stateIter.Begin(); stateIter; stateIter ++) {
		dState* const state = stateIter.GetNode()->GetInfo();

		// add all shift actions first
		for (dList<dTransition>::dListNode* node = state->m_transitions.GetFirst(); node; node = node->GetNext()) {
			dTransition& transition = node->GetInfo();
			if (transition.m_type == TERMINAL) {

				// find item generating this shift action and mark it as used.
				const dState* const targetState = transition.m_targetState;
				_ASSERTE (!state->m_actions.Find (transition.m_symbol));
				dTree<dAction, dCRCTYPE>::dTreeNode* const actionNode = state->m_actions.Insert (transition.m_symbol); 
				dAction& action = actionNode->GetInfo();
				action.m_type = dSHIFT;
				action.m_myItem = NULL;
				action.m_nextState = targetState->m_number;
				action.m_reduceRuleNode = NULL;
			}
		}

		// add all reduce actions
		dList<dAction*> potencialConflictinActions;
		for (dState::dListNode* itemNode = state->GetFirst(); itemNode; itemNode = itemNode->GetNext()) {
			dItem& item = itemNode->GetInfo();
			const dRuleInfo& ruleInfo = item.m_ruleNode->GetInfo();
			if ((ruleInfo.m_ruleNumber == 0) && (item.m_indexMarker == 1)) {
				dTree<dAction, dCRCTYPE>::dTreeNode* const actionNode = state->m_actions.Insert (acceptingSymbol); 
				_ASSERTE (actionNode);
				dAction& action = actionNode->GetInfo();
				action.m_type = dACCEPT;
				action.m_myItem = &item;
				action.m_reduceRuleNode = NULL;
			} else if ((item.m_indexMarker == ruleInfo.GetCount()) && (ruleInfo.m_nameCRC != startSymbol)) {
				dTree<dAction, dCRCTYPE>::dTreeNode* actionNode = state->m_actions.Find (item.m_lookAheadSymbolCRC); 
				if (!actionNode) {
					actionNode = state->m_actions.Insert (item.m_lookAheadSymbolCRC); 
					dAction& action = actionNode->GetInfo();
					action.m_type = dREDUCE;
					action.m_myItem = &item;
					action.m_nextState = 0;
					action.m_reduceRuleNode = item.m_ruleNode;
				} else {
					dAction& action = actionNode->GetInfo();
					action.m_myItem = &item;
					action.m_reduceRuleNode = item.m_ruleNode;
					potencialConflictinActions.Append (&actionNode->GetInfo());
				}
			}
		}

		// now resolve all conflicting actions
		if (potencialConflictinActions.GetCount()) {

			// resolve conflicting actions
			dList<dAction*>::dListNode* nextActionNode = NULL;
			for (dList<dAction*>::dListNode* actionNode = potencialConflictinActions.GetFirst(); actionNode; actionNode = nextActionNode) {
				dAction* const action = actionNode->GetInfo();

				if (action->m_type == dREDUCE) {
					// this is a reduce reduce conflict
					_ASSERTE (0);
					dTrace (("This is a reduce Reduce conflict, resolve in favor of of first production rule\n")); 
				}
				nextActionNode = actionNode->GetNext();

				const dItem& item = *action->m_myItem;
				if (item.m_lastOperatorSymbolCRC != emptySymbol) {
					const dOperatorsAssociation* const operatorAssosiation = operatorPrecedence.FindAssociation (item.m_lastOperatorSymbolCRC);
					_ASSERTE (operatorAssosiation);
					if (operatorAssosiation->m_associativity == dOperatorsAssociation::m_left) {

						const dOperatorsAssociation* const lookAheadOperatorAssosiation = operatorPrecedence.FindAssociation (item.m_lookAheadSymbolCRC);
						if (!(lookAheadOperatorAssosiation && (lookAheadOperatorAssosiation->m_prioprity > operatorAssosiation->m_prioprity))) {
							action->m_type = dREDUCE;
						}
					}
					potencialConflictinActions.Remove(actionNode);
				}
			}

			// for any conflicting actions left, display warning
			for (dList<dAction*>::dListNode* actionNode = potencialConflictinActions.GetFirst(); actionNode; actionNode = actionNode->GetNext()) {
				dAction* const action = actionNode->GetInfo();

				dRuleInfo& rule = action->m_reduceRuleNode->GetInfo();
				dString sentence;
				sentence += rule.m_name;
				sentence += " : ";
				for (dRuleInfo::dListNode* node = rule.GetFirst(); node; node = node->GetNext()) {
					sentence += node->GetInfo().m_name;
					sentence += " ";
				}

				if (action->m_type == dSHIFT) {
					if (!rule.m_shiftReduceErrorMark) {
						rule.m_shiftReduceErrorMark = true;
						if (m_shiftReduceExpectedWarnings <= 0) {
							DisplayError ("\nstate %d: shift reduce warning resolved in favor of shift. on rule\n", state->m_number);
							DisplayError ("  %s\n", sentence.GetStr());
						}
						m_shiftReduceExpectedWarnings --;
					}
					
				} else {
					_ASSERTE (0);
					DisplayError ("\nstate %d: reduce reduce error resolved in favor of first sentence. on rule\n", state->m_number);
					DisplayError ("  %s\n", sentence.GetStr());
				}
			}
		}
	}


	// create Goto Table
	for (stateIter.Begin(); stateIter; stateIter ++) {
		dState* const state = stateIter.GetNode()->GetInfo();
		for (dList<dTransition>::dListNode* node = state->m_transitions.GetFirst(); node; node = node->GetNext()) {
			dTransition& transition = node->GetInfo();
			if (transition.m_type == NONTERMINAL) {
				state->m_goto.Insert (transition.m_targetState, transition.m_symbol); 
			}
		}
	}
}