/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include "dae/daeDatabase.h"
using namespace std;

daeDatabase::daeDatabase(DAE& dae) : dae(dae) { }

DAE* daeDatabase::getDAE() {
	return &dae;
}

daeDocument* daeDatabase::getDoc(daeUInt index) {
	return getDocument(index);
}

daeElement* daeDatabase::idLookup(const string& id, daeDocument* doc) {
	vector<daeElement*> elts = idLookup(id);
	for (size_t i = 0; i < elts.size(); i++)
		if (elts[i]->getDocument() == doc)
			return elts[i];
	return NULL;
}

vector<daeElement*> daeDatabase::typeLookup(daeInt typeID, daeDocument* doc) {
	vector<daeElement*> result;
	typeLookup(typeID, result);
	return result;
}

vector<daeElement*> daeDatabase::sidLookup(const string& sid, daeDocument* doc) {
	vector<daeElement*> result;
	sidLookup(sid, result, doc);
	return result;
}
