// dTestAssembler.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <tchar.h>
#include <dVirtualMachine.h>
#include <dLittleScriptCompiler.h>


#ifdef _MSC_VER
#include <windows.h>
#include <crtdbg.h>
#endif


int _tmain(int argc, _TCHAR* argv[])
{
	#ifdef _MSC_VER
		_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	#endif


	if (argc < 2) {
		fprintf (stdout, "usage: dNewtonScriptCompiler filename\n");
		exit (0);
	}

	const char* const sourceFileName = argv[1];
	FILE* const sourceFile = fopen (sourceFileName, "r");
	if (!sourceFile) {
		fprintf (stdout, "script source file: \"%s\" not found\n", sourceFile);
		exit (0);
	}

	dString source;
	source.LoadFile(sourceFile);
	fclose (sourceFile);

	const char* const packacgeName = "demos";
	dScriptCompiler compiler(packacgeName);
	compiler.CompileSource (source.GetStr());
	return 0;
}

