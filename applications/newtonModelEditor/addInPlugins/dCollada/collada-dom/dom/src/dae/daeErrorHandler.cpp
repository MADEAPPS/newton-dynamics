/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeErrorHandler.h>
#include <modules/stdErrPlugin.h>

daeErrorHandler *daeErrorHandler::_instance = NULL;
std::auto_ptr<daeErrorHandler> daeErrorHandler::_defaultInstance(new stdErrPlugin);

daeErrorHandler::daeErrorHandler() {
}

daeErrorHandler::~daeErrorHandler() {
}

void daeErrorHandler::setErrorHandler( daeErrorHandler *eh ) {
	_instance = eh;
}

daeErrorHandler *daeErrorHandler::get() {
	if ( _instance == NULL ) {
		return _defaultInstance.get();
	}
	return _instance;
}
