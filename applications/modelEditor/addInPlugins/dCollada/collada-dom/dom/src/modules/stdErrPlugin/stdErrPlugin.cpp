/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <modules/stdErrPlugin.h>
#include <stdio.h>

quietErrorHandler quietErrorHandler::theInstance;

stdErrPlugin::stdErrPlugin() {
}

stdErrPlugin::~stdErrPlugin() {
}

void stdErrPlugin::handleError( daeString msg ) {
	//fprintf( stderr, "Error: %s\n", msg );
	//fflush( stderr );
	printf( "Error: %s\n", msg );
}

void stdErrPlugin::handleWarning( daeString msg ) {
	//fprintf( stderr, "Warning: %s\n", msg );
	//fflush( stderr );
	printf( "Warning: %s\n", msg );
}
