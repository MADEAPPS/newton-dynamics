/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <sstream>

#include <dom.h>
#include <dae/daeDatabase.h>
#include <dae/daeIOPluginCommon.h>
#include <dae/daeMetaElement.h>
#include <dae/daeErrorHandler.h>
#include <dae/daeMetaElementAttribute.h>
#ifndef NO_ZAE
#include <dae/daeZAEUncompressHandler.h>
#endif 

using namespace std;

daeIOPluginCommon::daeIOPluginCommon()
	: database(NULL),
		topMeta(NULL)
{
}

daeIOPluginCommon::~daeIOPluginCommon()
{
}

daeInt daeIOPluginCommon::setMeta(daeMetaElement *_topMeta)
{
	topMeta = _topMeta;
	return DAE_OK;
}

void daeIOPluginCommon::setDatabase(daeDatabase* _database)
{
	database = _database;
}

// This function needs to be re-entrant, it can be called recursively from inside of resolveAll
// to load files that the first file depends on.
daeInt daeIOPluginCommon::read(const daeURI& uri, daeString docBuffer)
{
	// Make sure topMeta has been set before proceeding
	if (topMeta == NULL) 
	{
		return DAE_ERR_BACKEND_IO;
	}

	// Generate a version of the URI with the fragment removed
	daeURI fileURI(*uri.getDAE(), uri.str(), true);

	//check if document already exists
	if ( database->isDocumentLoaded( fileURI.getURI() ) )
	{
		return DAE_ERR_COLLECTION_ALREADY_EXISTS;
	}

     daeElementRef domObject = docBuffer ?
            readFromMemory(docBuffer, fileURI) :
            readFromFile(fileURI); // Load from URI

#ifdef NO_ZAE

	if (!domObject) {
		string msg = docBuffer ?
			"Failed to load XML document from memory\n" :
			string("Failed to load ") + fileURI.str() + "\n";
		daeErrorHandler::get()->handleError(msg.c_str());
		return DAE_ERR_BACKEND_IO;
	}

	// Insert the document into the database, the Database will keep a ref on the main dom, so it won't get deleted
	// until we clear the database

	daeDocument *document = NULL;

	int res = database->insertDocument(fileURI.getURI(),domObject,&document);
	if (res!= DAE_OK)
		return res;

#else

    bool zaeRoot = false;
    string extractedURI = "";
    if (!domObject) {
        daeZAEUncompressHandler zaeHandler(fileURI);
        if (zaeHandler.isZipFile())
        {
            string rootFilePath = zaeHandler.obtainRootFilePath();
            daeURI rootFileURI(*fileURI.getDAE(), rootFilePath);
            domObject = readFromFile(rootFileURI);
            if (!domObject)
            {
                string msg = string("Failed to load ") + fileURI.str() + "\n";
                daeErrorHandler::get()->handleError(msg.c_str());
                return DAE_ERR_BACKEND_IO;
            }
            zaeRoot = true;
            extractedURI = rootFileURI.str();
        }
        else
        {
            string msg = docBuffer ?
                "Failed to load XML document from memory\n" :
            string("Failed to load ") + fileURI.str() + "\n";
            daeErrorHandler::get()->handleError(msg.c_str());
            return DAE_ERR_BACKEND_IO;
        }
    }

    // Insert the document into the database, the Database will keep a ref on the main dom, so it won't get deleted
    // until we clear the database

    daeDocument *document = NULL;

    int res = database->insertDocument(fileURI.getURI(),domObject,&document, zaeRoot, extractedURI);
    if (res!= DAE_OK)
        return res;

#endif

	return DAE_OK;
}










daeElementRef daeIOPluginCommon::beginReadElement(daeElement* parentElement,
																									daeString elementName,
																									const vector<attrPair>& attributes,
																									daeInt lineNumber) {
	daeMetaElement* parentMeta = parentElement ? parentElement->getMeta() : topMeta;
	daeElementRef element = parentMeta->create(elementName);
	
	if(!element)
	{
		ostringstream msg;
		msg << "The DOM was unable to create an element named " << elementName << " at line "
			     << lineNumber << ". Probably a schema violation.\n";
		daeErrorHandler::get()->handleWarning( msg.str().c_str() );
		return NULL;
	}

	// Process the attributes
	for (size_t i = 0; i < attributes.size(); i++) {
		daeString name  = attributes[i].first,
			        value = attributes[i].second;
		if (!element->setAttribute(name, value)) {
			ostringstream msg;
			msg << "The DOM was unable to create an attribute " << name << " = " << value
				  << " at line " << lineNumber << ".\nProbably a schema violation.\n";
			daeErrorHandler::get()->handleWarning(msg.str().c_str());
		}
	}
		
	if (parentElement == NULL) {
		// This is the root element. Check the COLLADA version.
		daeURI *xmlns = (daeURI*)(element->getMeta()->getMetaAttribute( "xmlns" )->getWritableMemory( element ));
		if ( strcmp( xmlns->getURI(), COLLADA_NAMESPACE ) != 0 ) {
			// Invalid COLLADA version
			daeErrorHandler::get()->handleError("Trying to load an invalid COLLADA version for this DOM build!");
			return NULL;
		}
	}
	
	return element;
}

bool daeIOPluginCommon::readElementText(daeElement* element, daeString text, daeInt elementLineNumber) {
	if (element->setCharData(text))
		return true;
	
	ostringstream msg;
	msg << "The DOM was unable to set a value for element of type " << element->getTypeName()
			<< " at line " << elementLineNumber << ".\nProbably a schema violation.\n";
	daeErrorHandler::get()->handleWarning(msg.str().c_str());
	return false;
}
