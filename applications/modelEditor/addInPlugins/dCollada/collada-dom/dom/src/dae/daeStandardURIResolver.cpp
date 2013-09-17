/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <sstream>

#include <dae/daeStandardURIResolver.h>
#include <dae/daeDatabase.h>
#include <dae/daeURI.h>
#include <dae/daeIOPlugin.h>
#include <dae/daeErrorHandler.h>

using namespace std;

daeStandardURIResolver::daeStandardURIResolver(DAE& dae)
	: daeURIResolver(dae) { }

daeStandardURIResolver::~daeStandardURIResolver() { }

daeString
daeStandardURIResolver::getName()
{
	return "XMLResolver";
}

namespace {
	void printErrorMsg(const daeURI& uri) {
		ostringstream msg;
		msg << "daeStandardURIResolver::resolveElement() - Failed to resolve " << uri.str() << endl;
		daeErrorHandler::get()->handleError(msg.str().c_str());
	}
}

daeElement* daeStandardURIResolver::resolveElement(const daeURI& uri) {
	daeDocument* doc = uri.getReferencedDocument();
	if (!doc) {
		dae->open(uri.str());
		doc = uri.getReferencedDocument();
		if (!doc) {
			printErrorMsg(uri);
			return NULL;
		}
	}

	daeElement* elt = dae->getDatabase()->idLookup(uri.id(), doc);
	if (!elt)
		printErrorMsg(uri);

	return elt;
}
