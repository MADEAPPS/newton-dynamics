/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeStringTable.h>

daeStringTable::daeStringTable(int stringBufferSize):_stringBufferSize(stringBufferSize), _empty( "" )
{
	_stringBufferIndex = _stringBufferSize;
	//allocate initial buffer
	//allocateBuffer();
}

daeString daeStringTable::allocateBuffer()
{
	daeString buf = new daeChar[_stringBufferSize];
	_stringBuffersList.append(buf);
	_stringBufferIndex = 0;
	return buf;
}

daeString daeStringTable::allocString(daeString string)
{
	if ( string == NULL ) return _empty;
	size_t stringSize = strlen(string) + 1;
	size_t sizeLeft = _stringBufferSize - _stringBufferIndex;
	daeString buf;
	if (sizeLeft < stringSize)
	{
		if (stringSize > _stringBufferSize)
			_stringBufferSize = ((stringSize / _stringBufferSize) + 1) * _stringBufferSize ;
		buf = allocateBuffer();
	}
	else
	{
		buf = _stringBuffersList.get((daeInt)_stringBuffersList.getCount()-1);
	}
	daeChar *str = (char*)buf + _stringBufferIndex;
	memcpy(str,string,stringSize);
	_stringBufferIndex += stringSize;

	int align = sizeof(void*);
	_stringBufferIndex = (_stringBufferIndex+(align-1)) & (~(align-1));

	return str;
}

void daeStringTable::clear()
{
	unsigned int i;
	for (i=0;i<_stringBuffersList.getCount();i++)
#if _MSC_VER <= 1200
		delete [] (char *) _stringBuffersList[i];
#else
		delete [] _stringBuffersList[i];
#endif

	_stringBuffersList.clear();
	_stringBufferIndex = _stringBufferSize;
}
