/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeStringRef.h>

//Contributed by Nus - Wed, 08 Nov 2006
// Nus: Use global pointer instead of local static.
static daeStringTable *pST = NULL;
//---------------------------

daeStringTable &daeStringRef::_stringTable()
{
//Contributed by Nus - Wed, 08 Nov 2006
  // static daeStringTable *st = new daeStringTable();
  // return *st;
  if(!pST)
    pST = new daeStringTable();
  return *pST;
}

void daeStringRef::releaseStringTable(void)
{
  if(pST) {
    delete pST;
    pST = NULL;
  }
}
//--------------------------------

daeStringRef::daeStringRef(daeString string)
{
	daeStringTable &st = _stringTable();
	_string = st.allocString(string);
}

const daeStringRef&
daeStringRef::set(daeString string)
{
	daeStringTable &st = _stringTable();
	_string = st.allocString(string);
	return *this;
}

const daeStringRef&
daeStringRef::operator= (daeString string)
{
	return set(string);
}
