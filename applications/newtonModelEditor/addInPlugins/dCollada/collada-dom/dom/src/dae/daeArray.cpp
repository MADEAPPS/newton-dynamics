/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeArrayTypes.h>
#include <dae/daeArray.h>

daeArray::daeArray():_count(0),_capacity(0),_data(NULL),_elementSize(4),_type(NULL)
{
}

daeArray::~daeArray()
{
}

void daeArray::setElementSize(size_t elementSize) {
	clear();
	_elementSize = elementSize;
}
