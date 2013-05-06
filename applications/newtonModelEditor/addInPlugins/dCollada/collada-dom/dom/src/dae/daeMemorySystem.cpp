/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeMemorySystem.h>
//#include <malloc.h>

daeRawRef
daeMemorySystem::alloc(daeString pool, size_t n)
{
	(void)pool;
	void *mem = malloc(n);
//	memset(mem,0,n);
//	printf("alloc[%s] - %d = 0x%x\n",pool,n,mem);
	return (daeRawRef)mem;
}

void
daeMemorySystem::dealloc(daeString pool, daeRawRef mem)
{
	(void)pool;
//	printf("free[%s] - 0x%x\n",pool,mem);
	free(mem);
}

