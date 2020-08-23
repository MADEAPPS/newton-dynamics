%{
#include <stdafx.h>
#include "newtonGrammar.h"
%}

D				[0-9]
L				[a-zA-Z_]
H				[a-fA-F0-9]
E				[Ee][+-]?{D}+
FS				(f|F|l|L)
IS				(u|U|l|L)*
ANY				[0-9a-zA-Z_.]

AnyButAstr		[^\*]
AnyButSlash		[^\/]
Comment1        [\/][\/].*
Comment2        [\/][\*]({AnyButAstr}|[\*]{AnyButSlash})*[\*][\/]
Comment			({Comment1}|{Comment2})

WhiteSpace		[ \t\n\r]+


%%
// constant, labels, and strings
{L}({L}|{D})*		{ return(newtonGrammar::_LITERAL_IDENTIFIER); }
{D}({D})*			{ return(newtonGrammar::_NUMERIC_CONSTANT); }
["]({ANY})*["]		{ return(newtonGrammar::_QUOTED_CONSTANT); }

// keyworlds 
"int"				{ return newtonGrammar::_INT;}
"char"				{ return newtonGrammar::_CHAR;}
"void"				{ return newtonGrammar::_VOID;}
"short"				{ return newtonGrammar::_SHORT;}
"unsigned"          { return newtonGrammar::_UNSIGNED;}
"const"				{ return newtonGrammar::_CONST;}
"float"				{ return newtonGrammar::_FLOAT;}
"dFloat"			{ return newtonGrammar::_DFLOAT;}
"double"			{ return newtonGrammar::_DOUBLE;}
"dFloat64"			{ return newtonGrammar::_DFLOAT64;}
"struct"			{ return newtonGrammar::_STRUCT;}
"union"				{ return newtonGrammar::_UNION;}
"typedef"			{ return newtonGrammar::_TYPEDEF;}
"extern"		    { return newtonGrammar::_EXTERN;}
"#ifdef"		    { return newtonGrammar::_IFDEF;}
"#ifndef"		    { return newtonGrammar::_IFNDEF;}
"#else"			    { return newtonGrammar::_ELSE;}
"#endif"			{ return newtonGrammar::_ENDIF;}
"#define"			{ return newtonGrammar::_DEFINE;}
"#include"			{ return newtonGrammar::_INCLUDE;}

// special characters need a back slash prefix  
"\("		{ return('('); }
"\)"		{ return(')'); }
"\["		{ return('['); }
"\]"		{ return(']'); }
"\+"		{ return('+'); }
"\*"		{ return('*'); }
"\|"		{ return('|'); }
"\?"		{ return('?'); }
"\."		{ return('.'); }
"\{"		{ return('{'); }
"\}"		{ return('}'); }
"\~"		{ return('~'); }
"\;"		{ return(';'); }
"\,"		{ return(','); }
"\:"		{ return(':'); }
"\="		{ return('='); }
"\&"		{ return('&'); }
"\!"		{ return('!'); }
"\-"		{ return('-'); }
"\/"		{ return('/'); }
"\%"		{ return('%'); }
"\<"		{ return('<'); }
"\>"		{ return('>'); }
"\^"		{ return('^'); }

{WhiteSpace}	{/* skip is a white space*/}
{Comment}		{/* skip commnets */}

%%


