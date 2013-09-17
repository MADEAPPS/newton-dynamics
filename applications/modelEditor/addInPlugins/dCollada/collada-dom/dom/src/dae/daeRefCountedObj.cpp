/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeRefCountedObj.h>

daeRefCountedObj::daeRefCountedObj() : _refCount(0) { }

daeRefCountedObj::~daeRefCountedObj() { }

void daeRefCountedObj::release() const {
	if (--_refCount <= 0)
		delete this;
}

void daeRefCountedObj::ref() const {
	_refCount++;
}

void checkedRelease(const daeRefCountedObj* obj) {
	if (obj)
		obj->release();
}

void checkedRef(const daeRefCountedObj* obj) {
	if (obj)
		obj->ref();
}
