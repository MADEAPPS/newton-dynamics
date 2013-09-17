/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDocument.h>
#include <dae/daeDatabase.h>


daeDocument::daeDocument(DAE& dae, bool zaeRootDocument, const std::string& extractedFileURI)
: dae(&dae), uri(dae), mZAERootDocument(zaeRootDocument), mExtractedFileURI(dae, extractedFileURI)
{ }

daeDocument::~daeDocument() {
}

void daeDocument::insertElement( daeElementRef element ) {
	dae->getDatabase()->insertElement( this, element.cast() );
}

void daeDocument::removeElement( daeElementRef element ) {
	dae->getDatabase()->removeElement( this, element.cast() );
}

void daeDocument::changeElementID( daeElementRef element, daeString newID ) {
	dae->getDatabase()->changeElementID( element.cast(), newID );
}

void daeDocument::changeElementSID( daeElementRef element, daeString newSID ) {
	dae->getDatabase()->changeElementSID( element.cast(), newSID );
}

DAE* daeDocument::getDAE() {
	return dae;
}

daeDatabase* daeDocument::getDatabase() {
	return dae->getDatabase();
}
