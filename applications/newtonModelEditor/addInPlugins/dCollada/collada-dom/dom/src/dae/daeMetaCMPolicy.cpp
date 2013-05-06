/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeMetaCMPolicy.h>

daeMetaCMPolicy::~daeMetaCMPolicy()
{
	for( size_t i = 0; i < _children.getCount(); i++ ) {
		delete _children[i];
	}
}

