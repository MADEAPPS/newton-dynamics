/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeError.h>

typedef struct 
{
	int errCode;
	const char *errString;
} DAEERROR;

static DAEERROR errorsArray[] =
{
	{ DAE_OK, "Success" },
	{ DAE_ERROR, "Generic error" },
	{ DAE_ERR_INVALID_CALL, "Invalid function call" },
	{ DAE_ERR_FATAL, "Fatal" },
	{ DAE_ERR_BACKEND_IO, "Backend IO" },
	{ DAE_ERR_BACKEND_VALIDATION, "Backend validation" },
	{ DAE_ERR_QUERY_SYNTAX, "Query syntax" },
	{ DAE_ERR_QUERY_NO_MATCH, "Query no match" },
	{ DAE_ERR_COLLECTION_ALREADY_EXISTS, "A document with the same name exists already" },
	{ DAE_ERR_COLLECTION_DOES_NOT_EXIST, "No document is loaded with that name or index" }, 
	{ DAE_ERR_NOT_IMPLEMENTED, "This function is not implemented in this reference implementation" },
};

const char *daeErrorString(int errorCode)
{
	int iErrorCount = (int)(sizeof(errorsArray)/sizeof(DAEERROR));
	for (int i=0;i<iErrorCount;i++)
	{
		if (errorsArray[i].errCode == errorCode)
			return errorsArray[i].errString;
	}
	return "Unknown Error code";
}
