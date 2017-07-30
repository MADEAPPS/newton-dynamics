/***
 *** C# parser/scanner
 *** Copyright 2002 James Power, NUI Maynooth, Ireland <james.power@may.ie>
 *** This version: 19 Feb 2002
 ***
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the <organization> nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
***/

/* Based on Appendix C of the C# Language Specification,
 *  version 0.28 of 5/7/2001
 */
%{
#define YYERROR_VERBOSE
  extern int yylineno;
  #include "lex.yy.h"
  int yyerror(char *);
%}

/* Special tokens to help disambiguate rank_specifiers */
%token RANK_SPECIFIER

/* C.1.4 Tokens */
%token IDENTIFIER 
%token INTEGER_LITERAL REAL_LITERAL CHARACTER_LITERAL STRING_LITERAL


/* C.1.7 KEYWORDS */ 
%token  ABSTRACT AS BASE BOOL BREAK
%token  BYTE CASE CATCH CHAR CHECKED
%token  CLASS CONST CONTINUE DECIMAL DEFAULT
%token  DELEGATE DO DOUBLE ELSE ENUM
%token  EVENT EXPLICIT EXTERN FALSE FINALLY
%token  FIXED FLOAT FOR FOREACH GOTO
%token  IF IMPLICIT IN INT INTERFACE
%token  INTERNAL IS LOCK LONG NAMESPACE
%token  NEW NULL_LITERAL OBJECT OPERATOR OUT
%token  OVERRIDE PARAMS PRIVATE PROTECTED PUBLIC
%token  READONLY REF RETURN SBYTE SEALED
%token  SHORT SIZEOF STACKALLOC STATIC STRING
%token  STRUCT SWITCH THIS THROW TRUE
%token  TRY TYPEOF UINT ULONG UNCHECKED
%token  UNSAFE USHORT USING VIRTUAL VOID
%token  VOLATILE WHILE

/* The ones that seem to be context sensitive */
/* Attribute Targets */
%token ASSEMBLY FIELD METHOD MODULE PARAM PROPERTY TYPE
/* Accessor types */
%token GET SET 
/* Event accessor declarations */
%token ADD REMOVE

/*** PUNCTUATION AND SINGLE CHARACTER OPERATORS ***/
%token COMMA  ","
%token LEFT_BRACKET  "["
%token RIGHT_BRACKET "]"

/*** MULTI-CHARACTER OPERATORS ***/
%token PLUSEQ MINUSEQ STAREQ DIVEQ MODEQ
%token XOREQ  ANDEQ   OREQ LTLT GTGT GTGTEQ LTLTEQ EQEQ NOTEQ
%token LEQ GEQ ANDAND OROR PLUSPLUS MINUSMINUS ARROW

%start compilation_unit  /* I think */

%%

/***** C.1.8 Literals *****/
literal
  : boolean_literal
  | INTEGER_LITERAL
  | REAL_LITERAL
  | CHARACTER_LITERAL
  | STRING_LITERAL
  | NULL_LITERAL
  ;
boolean_literal
  : TRUE
  | FALSE
  ;
/********** C.2 Syntactic grammar **********/

/***** C.2.1 Basic concepts *****/
namespace_name
  : qualified_identifier
  ;
type_name
  : qualified_identifier
  ;
/***** C.2.2 Types *****/
type
  : non_array_type
  | array_type
  ;
non_array_type
  : simple_type
  | type_name
  ;
simple_type
  : primitive_type
  | class_type
  | pointer_type
  ;
primitive_type
  : numeric_type
  | BOOL
  ;
numeric_type
  : integral_type
  | floating_point_type
  | DECIMAL
  ;
integral_type
  : SBYTE | BYTE | SHORT | USHORT | INT | UINT | LONG | ULONG | CHAR
  ;
floating_point_type
  : FLOAT | DOUBLE
  ;
class_type
  : OBJECT | STRING
  ;
pointer_type
  : type '*'
  | VOID '*'
  ;
array_type
  : array_type rank_specifier
  | simple_type rank_specifier
  | qualified_identifier rank_specifier
  ;
rank_specifiers_opt
  : /* Nothing */
  | rank_specifier rank_specifiers_opt
  ;
rank_specifier
  : RANK_SPECIFIER
  ;
/***** C.2.3 Variables *****/
variable_reference
  : expression
  ;
/***** C.2.4 Expressions *****/
argument_list
  : argument
  | argument_list COMMA argument
  ;
argument
  : expression
  | REF variable_reference
  | OUT variable_reference
  ;
primary_expression
  : parenthesized_expression
  | primary_expression_no_parenthesis
  ;
primary_expression_no_parenthesis
  : literal
  | array_creation_expression
  | member_access
  | invocation_expression
  | element_access
  | this_access
  | base_access
  | new_expression
  | typeof_expression
  | sizeof_expression
  | checked_expression
  | unchecked_expression
  ;
parenthesized_expression
  : '(' expression ')'
  ;
member_access
  : primary_expression '.' IDENTIFIER
  | primitive_type '.' IDENTIFIER
  | class_type '.' IDENTIFIER
  ;
invocation_expression
  : primary_expression_no_parenthesis '(' argument_list_opt ')'
  | qualified_identifier '(' argument_list_opt ')'
  ;
argument_list_opt
  : /* Nothing */
  | argument_list
  ;
element_access
  : primary_expression LEFT_BRACKET expression_list RIGHT_BRACKET
  | qualified_identifier LEFT_BRACKET expression_list RIGHT_BRACKET
  ;
expression_list_opt
  : /* Nothing */
  | expression_list
  ;
expression_list
  : expression
  | expression_list COMMA expression
  ;
this_access
  : THIS
  ;
base_access
  : BASE '.' IDENTIFIER
  | BASE LEFT_BRACKET expression_list RIGHT_BRACKET
  ;
post_increment_expression
  : postfix_expression PLUSPLUS
  ;
post_decrement_expression
  : postfix_expression MINUSMINUS
  ;
new_expression
  : object_creation_expression
  ;
object_creation_expression
  : NEW type '(' argument_list_opt ')'
  ;
array_creation_expression
  : NEW non_array_type LEFT_BRACKET expression_list RIGHT_BRACKET rank_specifiers_opt array_initializer_opt
  | NEW array_type array_initializer
  ;
array_initializer_opt
  : /* Nothing */
  | array_initializer
  ;
typeof_expression
  : TYPEOF '(' type ')'
  | TYPEOF '(' VOID ')'
  ;
checked_expression
  : CHECKED '(' expression ')'
  ;
unchecked_expression
  : UNCHECKED '(' expression ')'
  ;
pointer_member_access
  : postfix_expression ARROW IDENTIFIER
  ;
addressof_expression
  : '&' unary_expression
  ;
sizeof_expression
  : SIZEOF '(' type ')'
  ;
postfix_expression
  : primary_expression
  | qualified_identifier
  | post_increment_expression
  | post_decrement_expression
  | pointer_member_access
  ;
unary_expression_not_plusminus
  : postfix_expression
  | '!' unary_expression
  | '~' unary_expression
  | cast_expression
  ;
pre_increment_expression
  : PLUSPLUS unary_expression
  ;
pre_decrement_expression
  : MINUSMINUS unary_expression
  ;
unary_expression
  : unary_expression_not_plusminus
  | '+' unary_expression
  | '-' unary_expression
  | '*' unary_expression
  | pre_increment_expression
  | pre_decrement_expression
  | addressof_expression
  ;
/* For cast_expression we really just want a (type) in the brackets,
 * but have to do some factoring to get rid of conflict with expressions.
 * The paremtnesised expression in the first three cases below should be 
 * semantically restricted to an identifier, optionally follwed by qualifiers
 */
cast_expression
  : '(' expression ')' unary_expression_not_plusminus
  | '(' multiplicative_expression '*' ')' unary_expression 
  | '(' qualified_identifier rank_specifier type_quals_opt ')' unary_expression  
  | '(' primitive_type type_quals_opt ')' unary_expression
  | '(' class_type type_quals_opt ')' unary_expression
  | '(' VOID type_quals_opt ')' unary_expression
  ;
type_quals_opt
  : /* Nothing */
  | type_quals
  ;
type_quals
  : type_qual
  | type_quals type_qual
  ;
type_qual 
  : rank_specifier 
  | '*'
  ;
multiplicative_expression
  : unary_expression
  | multiplicative_expression '*' unary_expression  
  | multiplicative_expression '/' unary_expression
  | multiplicative_expression '%' unary_expression
  ;
additive_expression
  : multiplicative_expression
  | additive_expression '+' multiplicative_expression
  | additive_expression '-' multiplicative_expression
  ;
shift_expression
  : additive_expression 
  | shift_expression LTLT additive_expression
  | shift_expression GTGT additive_expression
  ;
relational_expression
  : shift_expression
  | relational_expression '<' shift_expression
  | relational_expression '>' shift_expression
  | relational_expression LEQ shift_expression
  | relational_expression GEQ shift_expression
  | relational_expression IS type
  | relational_expression AS type
  ;
equality_expression
  : relational_expression
  | equality_expression EQEQ relational_expression
  | equality_expression NOTEQ relational_expression
  ;
and_expression
  : equality_expression
  | and_expression '&' equality_expression
  ;
exclusive_or_expression
  : and_expression
  | exclusive_or_expression '^' and_expression
  ;
inclusive_or_expression
  : exclusive_or_expression
  | inclusive_or_expression '|' exclusive_or_expression
  ;
conditional_and_expression
  : inclusive_or_expression
  | conditional_and_expression ANDAND inclusive_or_expression
  ;
conditional_or_expression
  : conditional_and_expression
  | conditional_or_expression OROR conditional_and_expression
  ;
conditional_expression
  : conditional_or_expression
  | conditional_or_expression '?' expression ':' expression
  ;
assignment
: unary_expression assignment_operator expression
  ;
assignment_operator
  : '=' | PLUSEQ | MINUSEQ | STAREQ | DIVEQ | MODEQ 
  | XOREQ | ANDEQ | OREQ | GTGTEQ | LTLTEQ 
  ;
expression
  : conditional_expression
  | assignment
  ;
constant_expression
  : expression
  ;
boolean_expression
  : expression
  ;
/***** C.2.5 Statements *****/
statement
  : labeled_statement
  | declaration_statement
  | embedded_statement
  ;
embedded_statement
  : block
  | empty_statement
  | expression_statement
  | selection_statement
  | iteration_statement
  | jump_statement
  | try_statement
  | checked_statement
  | unchecked_statement
  | lock_statement
  | using_statement
  | unsafe_statement
  | fixed_statement
  ;
block
  : '{' statement_list_opt '}'
  ;
statement_list_opt
  : /* Nothing */
  | statement_list
  ;

statement_list
  : statement
  | statement_list statement
  ;
empty_statement
  : ';'
  ;
labeled_statement
  : IDENTIFIER ':' statement
  ;
declaration_statement
  : local_variable_declaration ';'
  | local_constant_declaration ';'
  ;
local_variable_declaration
  : type variable_declarators
  ;
variable_declarators
  : variable_declarator
  | variable_declarators COMMA variable_declarator
  ;
variable_declarator
  : IDENTIFIER
  | IDENTIFIER '=' variable_initializer
  ;
variable_initializer
  : expression
  | array_initializer
  | stackalloc_initializer
  ;
stackalloc_initializer
: STACKALLOC type  LEFT_BRACKET expression RIGHT_BRACKET 
  ; 
local_constant_declaration
  : CONST type constant_declarators
  ;
constant_declarators
  : constant_declarator
  | constant_declarators COMMA constant_declarator
  ;
constant_declarator
  : IDENTIFIER '=' constant_expression
  ;
expression_statement
  : statement_expression ';'
  ;
statement_expression
  : invocation_expression
  | object_creation_expression
  | assignment
  | post_increment_expression
  | post_decrement_expression
  | pre_increment_expression
  | pre_decrement_expression
  ;
selection_statement
  : if_statement
  | switch_statement
  ;
if_statement
  : IF '(' boolean_expression ')' embedded_statement
  | IF '(' boolean_expression ')' embedded_statement ELSE embedded_statement
  ;
switch_statement
  : SWITCH '(' expression ')' switch_block
  ;
switch_block
  : '{' switch_sections_opt '}'
  ;
switch_sections_opt
  : /* Nothing */
  | switch_sections
  ;
switch_sections
  : switch_section
  | switch_sections switch_section
  ;
switch_section
  : switch_labels statement_list
  ;
switch_labels
  : switch_label
  | switch_labels switch_label
  ;
switch_label
  : CASE constant_expression ':'
  | DEFAULT ':'
  ;
iteration_statement
  : while_statement
  | do_statement
  | for_statement
  | foreach_statement
  ;
unsafe_statement
  : UNSAFE block
  ;
while_statement
  : WHILE '(' boolean_expression ')' embedded_statement
  ;
do_statement
  : DO embedded_statement WHILE '(' boolean_expression ')' ';'
  ;
for_statement
  : FOR '(' for_initializer_opt ';' for_condition_opt ';' for_iterator_opt ')' embedded_statement
  ;
for_initializer_opt
  : /* Nothing */
  | for_initializer
  ;
for_condition_opt
  : /* Nothing */
  | for_condition
  ;
for_iterator_opt
  : /* Nothing */
  | for_iterator
  ;
for_initializer
  : local_variable_declaration
  | statement_expression_list
  ;
for_condition
  : boolean_expression
  ;
for_iterator
  : statement_expression_list
  ;
statement_expression_list
  : statement_expression
  | statement_expression_list COMMA statement_expression
  ;
foreach_statement
  : FOREACH '(' type IDENTIFIER IN expression ')' embedded_statement
  ;
jump_statement
  : break_statement
  | continue_statement
  | goto_statement
  | return_statement
  | throw_statement
  ;
break_statement
  : BREAK ';'
  ;
continue_statement
  : CONTINUE ';'
  ;
goto_statement
  : GOTO IDENTIFIER ';'
  | GOTO CASE constant_expression ';'
  | GOTO DEFAULT ';'
  ;
return_statement
  : RETURN expression_opt ';'
  ;
expression_opt
  : /* Nothing */
  | expression
  ;
throw_statement
  : THROW expression_opt ';'
  ;
try_statement
  : TRY block catch_clauses
  | TRY block finally_clause
  | TRY block catch_clauses finally_clause
  ;
catch_clauses
  : catch_clause
  | catch_clauses catch_clause
  ;
catch_clause
  : CATCH '(' class_type identifier_opt ')' block
  | CATCH '(' type_name identifier_opt ')' block
  | CATCH block
  ;
identifier_opt
  : /* Nothing */
  | IDENTIFIER
  ;
finally_clause
  : FINALLY block
  ;
checked_statement
  : CHECKED block
  ;
unchecked_statement
  : UNCHECKED block
  ;
lock_statement
  : LOCK '(' expression ')' embedded_statement
  ;
using_statement
  : USING '(' resource_acquisition ')' embedded_statement
  ;
resource_acquisition
  : local_variable_declaration
  | expression
  ;
fixed_statement
/*! : FIXED '(' pointer_type fixed_pointer_declarators ')' embedded_statement */
  : FIXED '('  type fixed_pointer_declarators ')' embedded_statement
  ;
fixed_pointer_declarators
  : fixed_pointer_declarator
  | fixed_pointer_declarators COMMA fixed_pointer_declarator
  ;
fixed_pointer_declarator
  : IDENTIFIER '=' expression
  ;
compilation_unit
  : using_directives_opt attributes_opt 
  | using_directives_opt namespace_member_declarations
  ;
using_directives_opt
  : /* Nothing */
  | using_directives
  ;
attributes_opt
  : /* Nothing */
  | attributes
  ;
namespace_member_declarations_opt
  : /* Nothing */
  | namespace_member_declarations
  ;
namespace_declaration
  : attributes_opt NAMESPACE qualified_identifier namespace_body comma_opt
  ;
comma_opt
  : /* Nothing */
  | ';'
  ;
/*
qualified_identifier
  : IDENTIFIER
  | qualified_identifier '.' IDENTIFIER
  ;
*/
qualified_identifier
  : IDENTIFIER
  | qualifier IDENTIFIER
  ;
qualifier
  : IDENTIFIER '.' 
  | qualifier IDENTIFIER '.' 
  ;
namespace_body
  : '{' using_directives_opt namespace_member_declarations_opt '}'
  ;
using_directives
  : using_directive
  | using_directives using_directive
  ;
using_directive
  : using_alias_directive
  | using_namespace_directive
  ;
using_alias_directive
  : USING IDENTIFIER '=' qualified_identifier ';'
  ;
using_namespace_directive
  : USING namespace_name ';'
  ;
namespace_member_declarations
  : namespace_member_declaration
  | namespace_member_declarations namespace_member_declaration
  ;
namespace_member_declaration
  : namespace_declaration
  | type_declaration
  ;
type_declaration
  : class_declaration
  | struct_declaration
  | interface_declaration
  | enum_declaration
  | delegate_declaration
  ;

/***** Modifiers *****/
/* This now replaces:
 * class_modifier, constant_modifier, field_modifier, method_modifier, 
 * property_modifier, event_modifier, indexer_modifier, operator_modifier, 
 * constructor_modifier, struct_modifier, interface_modifier, 
 * enum_modifier, delegate_modifier
 */
modifiers_opt
  : /* Nothing */
  | modifiers
  ;
modifiers
  : modifier
  | modifiers modifier
  ;
modifier
  : ABSTRACT
  | EXTERN
  | INTERNAL
  | NEW
  | OVERRIDE
  | PRIVATE
  | PROTECTED
  | PUBLIC
  | READONLY
  | SEALED
  | STATIC
  | UNSAFE
  | VIRTUAL
  | VOLATILE
  ;
/***** C.2.6 Classes *****/
class_declaration
  : attributes_opt modifiers_opt CLASS IDENTIFIER class_base_opt class_body comma_opt
  ;
class_base_opt
  : /* Nothing */
  | class_base
  ;
class_base
  : ':' class_type
  | ':' interface_type_list
  | ':' class_type COMMA interface_type_list
  ;
interface_type_list
  : type_name
  | interface_type_list COMMA type_name
  ;
class_body
  : '{' class_member_declarations_opt '}'
  ;
class_member_declarations_opt
  : /* Nothing */
  | class_member_declarations
  ;
class_member_declarations
  : class_member_declaration
  | class_member_declarations class_member_declaration
  ;
class_member_declaration
  : constant_declaration
  | field_declaration
  | method_declaration
  | property_declaration
  | event_declaration
  | indexer_declaration
  | operator_declaration
  | constructor_declaration
  | destructor_declaration
/*  | static_constructor_declaration */
  | type_declaration
  ;
constant_declaration
  : attributes_opt modifiers_opt CONST type constant_declarators ';'
  ;
field_declaration
  : attributes_opt modifiers_opt type variable_declarators ';'
  ;
method_declaration
  : method_header method_body
  ;
/* Inline return_type to avoid conflict with field_declaration */
method_header
  : attributes_opt modifiers_opt type qualified_identifier '(' formal_parameter_list_opt ')'
  | attributes_opt modifiers_opt VOID qualified_identifier '(' formal_parameter_list_opt ')'
  ;
formal_parameter_list_opt
  : /* Nothing */
  | formal_parameter_list
  ;
return_type
  : type
  | VOID
  ;
method_body
  : block
  | ';'
  ;
formal_parameter_list
  : formal_parameter
  | formal_parameter_list COMMA formal_parameter
  ;
formal_parameter
  : fixed_parameter
  | parameter_array
  ;
fixed_parameter
  : attributes_opt parameter_modifier_opt type IDENTIFIER
  ;
parameter_modifier_opt
  : /* Nothing */
  | REF
  | OUT
  ;
parameter_array
/*!  : attributes_opt PARAMS array_type IDENTIFIER */
  : attributes_opt PARAMS type IDENTIFIER
  ;
property_declaration
  : attributes_opt modifiers_opt type qualified_identifier 
      ENTER_getset
    '{' accessor_declarations '}'
      EXIT_getset
  ;
accessor_declarations
  : get_accessor_declaration set_accessor_declaration_opt
  | set_accessor_declaration get_accessor_declaration_opt
  ;
set_accessor_declaration_opt
  : /* Nothing */
  | set_accessor_declaration
  ;
get_accessor_declaration_opt
  : /* Nothing */
  | get_accessor_declaration
  ;
get_accessor_declaration
  : attributes_opt GET 
      EXIT_getset
    accessor_body
      ENTER_getset
  ;
set_accessor_declaration
  : attributes_opt SET 
      EXIT_getset
    accessor_body
      ENTER_getset
  ;
accessor_body
  : block
  | ';'
  ;
event_declaration
  : attributes_opt modifiers_opt EVENT type variable_declarators ';'
  | attributes_opt modifiers_opt EVENT type qualified_identifier 
      ENTER_accessor_decl 
    '{' event_accessor_declarations '}'
      EXIT_accessor_decl
  ;
event_accessor_declarations
  : add_accessor_declaration remove_accessor_declaration
  | remove_accessor_declaration add_accessor_declaration
  ;
add_accessor_declaration
  : attributes_opt ADD 
      EXIT_accessor_decl 
    block 
      ENTER_accessor_decl
  ;
remove_accessor_declaration
  : attributes_opt REMOVE 
      EXIT_accessor_decl 
    block 
      ENTER_accessor_decl
  ;
indexer_declaration
  : attributes_opt modifiers_opt indexer_declarator 
      ENTER_getset
    '{' accessor_declarations '}'
      EXIT_getset
  ;
indexer_declarator
  : type THIS LEFT_BRACKET formal_parameter_list RIGHT_BRACKET
/* | type type_name '.' THIS LEFT_BRACKET formal_parameter_list RIGHT_BRACKET */
  | type qualified_this LEFT_BRACKET formal_parameter_list RIGHT_BRACKET
  ;
qualified_this
  : qualifier THIS
  ;
/* Widen operator_declaration to make modifiers optional */
operator_declaration
  : attributes_opt modifiers_opt operator_declarator operator_body
  ;
operator_declarator
  : overloadable_operator_declarator
  | conversion_operator_declarator
  ;
overloadable_operator_declarator
  : type OPERATOR overloadable_operator '(' type IDENTIFIER ')'
  | type OPERATOR overloadable_operator '(' type IDENTIFIER COMMA type IDENTIFIER ')'
  ;
overloadable_operator
  : '+' | '-' 
  | '!' | '~' | PLUSPLUS | MINUSMINUS | TRUE | FALSE
  | '*' | '/' | '%' | '&' | '|' | '^' 
  | LTLT | GTGT | EQEQ | NOTEQ | '>' | '<' | GEQ | LEQ
  ;
conversion_operator_declarator
  : IMPLICIT OPERATOR type '(' type IDENTIFIER ')'
  | EXPLICIT OPERATOR type '(' type IDENTIFIER ')'
  ;
constructor_declaration
  : attributes_opt modifiers_opt constructor_declarator constructor_body
  ;
constructor_declarator
  : IDENTIFIER '(' formal_parameter_list_opt ')' constructor_initializer_opt
  ;
constructor_initializer_opt
  : /* Nothing */
  | constructor_initializer
  ;
constructor_initializer
  : ':' BASE '(' argument_list_opt ')'
  | ':' THIS '(' argument_list_opt ')'
  ;
/* Widen from unsafe_opt STATIC to modifiers_opt */
/* This is now subsumed by constructor_declaration - delete
 * static_constructor_declaration
 *  : attributes_opt modifiers_opt IDENTIFIER '(' ')' block
 *  ;
 */
/* No longer needed after modification of static_constructor_declaration
 * unsafe_opt
 * : 
 * | UNSAFE
 * ;
 */
/* Widen from unsafe_opt to modifiers_opt */
destructor_declaration
  : attributes_opt modifiers_opt '~' IDENTIFIER '(' ')' block
  ;
operator_body
  : block
  | ';'
  ;
constructor_body /*** Added by JP - same as method_body ***/
  : block
  | ';'
  ;

/***** C.2.7 Structs *****/
struct_declaration
  : attributes_opt modifiers_opt STRUCT IDENTIFIER struct_interfaces_opt struct_body comma_opt
  ;
struct_interfaces_opt
  : /* Nothing */
  | struct_interfaces
  ;
struct_interfaces
  : ':' interface_type_list
  ;
struct_body
  : '{' struct_member_declarations_opt '}'
  ;
struct_member_declarations_opt
  : /* Nothing */
  | struct_member_declarations
  ;
struct_member_declarations
  : struct_member_declaration
  | struct_member_declarations struct_member_declaration
  ;
struct_member_declaration
  : constant_declaration
  | field_declaration
  | method_declaration
  | property_declaration
  | event_declaration
  | indexer_declaration
  | operator_declaration
  | constructor_declaration
/*  | static_constructor_declaration */
  | type_declaration
  ;

/***** C.2.8 Arrays *****/
array_initializer
  : '{' variable_initializer_list_opt '}'
  | '{' variable_initializer_list COMMA '}'
  ;
variable_initializer_list_opt
  : /* Nothing */
  | variable_initializer_list
  ;
variable_initializer_list
  : variable_initializer
  | variable_initializer_list COMMA variable_initializer
  ;

/***** C.2.9 Interfaces *****/
interface_declaration
  : attributes_opt modifiers_opt INTERFACE IDENTIFIER interface_base_opt interface_body comma_opt
  ;
interface_base_opt
  : /* Nothing */
  | interface_base
  ;
interface_base
  : ':' interface_type_list
  ;
interface_body
  : '{' interface_member_declarations_opt '}'
  ;
interface_member_declarations_opt
  : /* Nothing */
  | interface_member_declarations
  ;
interface_member_declarations
  : interface_member_declaration
  | interface_member_declarations interface_member_declaration
  ;
interface_member_declaration
  : interface_method_declaration
  | interface_property_declaration
  | interface_event_declaration
  | interface_indexer_declaration
  ;
/* inline return_type to avoid conflict with interface_property_declaration */
interface_method_declaration
  : attributes_opt new_opt type IDENTIFIER '(' formal_parameter_list_opt ')' interface_empty_body
  | attributes_opt new_opt VOID IDENTIFIER '(' formal_parameter_list_opt ')' interface_empty_body
  ;
new_opt
  : /* Nothing */
  | NEW
  ;
interface_property_declaration
  : attributes_opt new_opt type IDENTIFIER 
      ENTER_getset
    '{' interface_accessors '}'
      EXIT_getset
  ;
interface_indexer_declaration
  : attributes_opt new_opt type THIS 
    LEFT_BRACKET formal_parameter_list RIGHT_BRACKET 
      ENTER_getset
    '{' interface_accessors '}'
      EXIT_getset
  ;

interface_accessors
  : attributes_opt GET interface_empty_body
  | attributes_opt SET interface_empty_body
  | attributes_opt GET interface_empty_body attributes_opt SET interface_empty_body
  | attributes_opt SET interface_empty_body attributes_opt GET interface_empty_body
  ;
interface_event_declaration
  : attributes_opt new_opt EVENT type IDENTIFIER interface_empty_body
  ;

/* mono seems to allow this */
interface_empty_body
  : ';'
  | '{' '}'
  ;

/***** C.2.10 Enums *****/
enum_declaration
  : attributes_opt modifiers_opt ENUM IDENTIFIER enum_base_opt enum_body comma_opt
  ;
enum_base_opt
  : /* Nothing */
  | enum_base
  ;
enum_base
  : ':' integral_type
  ;
enum_body
  : '{' enum_member_declarations_opt '}'
  | '{' enum_member_declarations COMMA '}'
  ;
enum_member_declarations_opt
  : /* Nothing */
  | enum_member_declarations
  ;
enum_member_declarations
  : enum_member_declaration
  | enum_member_declarations COMMA enum_member_declaration
  ;
enum_member_declaration
  : attributes_opt IDENTIFIER
  | attributes_opt IDENTIFIER '=' constant_expression
  ;

/***** C.2.11 Delegates *****/
delegate_declaration
  : attributes_opt modifiers_opt DELEGATE return_type IDENTIFIER '(' formal_parameter_list_opt ')' ';'
  ;

/***** C.2.12 Attributes *****/
attributes
  : attribute_sections
  ;
attribute_sections
  : attribute_section
  | attribute_sections attribute_section
  ;
attribute_section
  : ENTER_attrib LEFT_BRACKET attribute_target_specifier_opt attribute_list RIGHT_BRACKET EXIT_attrib
  | ENTER_attrib LEFT_BRACKET attribute_target_specifier_opt attribute_list COMMA RIGHT_BRACKET EXIT_attrib
  ;
attribute_target_specifier_opt
  : /* Nothing */
  | attribute_target_specifier
  ;
attribute_target_specifier
  : attribute_target ':'
  ;
attribute_target
  : ASSEMBLY
  | FIELD
  | EVENT
  | METHOD
  | MODULE
  | PARAM
  | PROPERTY
  | RETURN
  | TYPE
  ;
attribute_list
  : attribute
  | attribute_list COMMA attribute
  ;
attribute
  : attribute_name attribute_arguments_opt
  ;
attribute_arguments_opt
  : /* Nothing */
  | attribute_arguments
  ;
attribute_name
  : type_name
  ;
attribute_arguments
  : '(' expression_list_opt ')'
  ;



/** Dummy rules for those context-sensitive "keywords" **/
ENTER_attrib 
  : { lex_enter_attrib(); }
  ;
EXIT_attrib 
  : { lex_exit_attrib(); }
  ;
ENTER_accessor_decl 
  : { lex_enter_accessor(); }
  ;
EXIT_accessor_decl
  : { lex_exit_accessor(); }
  ;
ENTER_getset
  : { lex_enter_getset(); }
  ;
EXIT_getset
  : { lex_exit_getset(); }
  ;


%%

int yyerror(char *s)
{
  fprintf(stderr,"%d:%s LA=[%s]\n",yylineno,s,yytname[YYTRANSLATE(yychar)]);
  return 1;
}

