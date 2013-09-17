/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>

// The user can choose whether or not to include TinyXML support in the DOM. Supporting TinyXML will
// require linking against it. By default TinyXML support isn't included.
#if defined(DOM_INCLUDE_TINYXML)

#if defined(DOM_DYNAMIC) && defined(_MSC_VER)
#pragma comment(lib, "tinyxml.lib")
#endif

#if defined(_MSC_VER)
#pragma warning(disable: 4100) // warning C4100: 'element' : unreferenced formal parameter
#endif


#include <string>
#include <tinyxml.h>
#include <dom.h>
#include <dae/daeMetaElement.h>
#include <dae/daeErrorHandler.h>
#include <dae/daeMetaElementAttribute.h>
#include <dae/daeTinyXMLPlugin.h>
#include <dae/daeDocument.h>

using namespace std;

namespace {
	daeInt getCurrentLineNumber(TiXmlElement* element) {
		return -1;
	}
}

daeTinyXMLPlugin::daeTinyXMLPlugin()
{
  m_doc = NULL;
	supportedProtocols.push_back("*");
}

daeTinyXMLPlugin::~daeTinyXMLPlugin()
{
}

daeInt daeTinyXMLPlugin::setOption( daeString option, daeString value )
{
	return DAE_ERR_INVALID_CALL;
}

daeString daeTinyXMLPlugin::getOption( daeString option )
{
	return NULL;
}

daeElementRef daeTinyXMLPlugin::readFromFile(const daeURI& uri) {
	string file = cdom::uriToNativePath(uri.str());
	if (file.empty())
		return NULL;
	TiXmlDocument doc;
	doc.LoadFile(file.c_str());
	if (!doc.RootElement()) {
		daeErrorHandler::get()->handleError((std::string("Failed to open ") + uri.str() +
		                                     " in daeTinyXMLPlugin::readFromFile\n").c_str());
		return NULL;
	}
	return readElement(doc.RootElement(), NULL);
}

daeElementRef daeTinyXMLPlugin::readFromMemory(daeString buffer, const daeURI& baseUri) {
	TiXmlDocument doc;
	doc.Parse(buffer);
	if (!doc.RootElement()) {
		daeErrorHandler::get()->handleError("Failed to open XML document from memory buffer in "
		                                    "daeTinyXMLPlugin::readFromMemory\n");
		return NULL;
	}
	return readElement(doc.RootElement(), NULL);
}

daeElementRef daeTinyXMLPlugin::readElement(TiXmlElement* tinyXmlElement, daeElement* parentElement) {
	std::vector<attrPair> attributes;
	for (TiXmlAttribute* attrib = tinyXmlElement->FirstAttribute(); attrib != NULL; attrib = attrib->Next())
		attributes.push_back(attrPair(attrib->Name(), attrib->Value()));
		
	daeElementRef element = beginReadElement(parentElement, tinyXmlElement->Value(),
	                                         attributes, getCurrentLineNumber(tinyXmlElement));
	if (!element) {
		// We couldn't create the element. beginReadElement already printed an error message.
		return NULL;
	}

  if (tinyXmlElement->GetText() != NULL)
		readElementText(element, tinyXmlElement->GetText(), getCurrentLineNumber(tinyXmlElement));
  
  // Recurse children
  for (TiXmlElement* child = tinyXmlElement->FirstChildElement(); child != NULL; child = child->NextSiblingElement())
    element->placeElement(readElement(child, element));

	return element;
}

daeInt daeTinyXMLPlugin::write(const daeURI& name, daeDocument *document, daeBool replace)
{
	// Make sure database and document are both set
	if (!database)
		return DAE_ERR_INVALID_CALL;
	if(!document)
		return DAE_ERR_COLLECTION_DOES_NOT_EXIST;

	string fileName = cdom::uriToNativePath(name.str());
	if (fileName.empty())
	{
		daeErrorHandler::get()->handleError( "can't get path in write\n" );
		return DAE_ERR_BACKEND_IO;
	}
	// If replace=false, don't replace existing files
	if(!replace)
	{
		// Using "stat" would be better, but it's not available on all platforms
		FILE *tempfd = fopen(fileName.c_str(), "r");
		if(tempfd != NULL)
		{
			// File exists, return error
			fclose(tempfd);
			return DAE_ERR_BACKEND_FILE_EXISTS;
		}
		fclose(tempfd);
	}
	
  m_doc = new TiXmlDocument(name.getURI());
  if (m_doc)
  {
    m_doc->SetTabSize(4);

   	TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );  
	  m_doc->LinkEndChild( decl ); 

    writeElement(document->getDomRoot());

    m_doc->SaveFile(fileName.c_str());
    delete m_doc;
    m_doc = NULL;
  }
	return DAE_OK;
}

void daeTinyXMLPlugin::writeElement( daeElement* element )
{
	daeMetaElement* _meta = element->getMeta();
  if (!_meta->getIsTransparent() ) 
  {
		TiXmlElement* tiElm = new TiXmlElement( element->getElementName() );  
        
		if (m_elements.empty() == true) {
				m_doc->LinkEndChild(tiElm);
			} else {
			TiXmlElement* first = m_elements.front();
			first->LinkEndChild(tiElm);
		}
		m_elements.push_front(tiElm);

		daeMetaAttributeRefArray& attrs = _meta->getMetaAttributes();
		
		int acnt = (int)attrs.getCount();
		
		for(int i=0;i<acnt;i++) 
    {
			writeAttribute( attrs[i], element );
		}
	}
	writeValue(element);
	
	daeElementRefArray children;
	element->getChildren( children );
	for ( size_t x = 0; x < children.getCount(); x++ ) 
  {
		writeElement( children.get(x) );
	}

	if (!_meta->getIsTransparent() ) 
  {
    m_elements.pop_front();
	}
}


void daeTinyXMLPlugin::writeValue( daeElement* element )
{
	if (daeMetaAttribute* attr = element->getMeta()->getValueAttribute()) {
		std::ostringstream buffer;
		attr->memoryToString(element, buffer);
		std::string s = buffer.str();
		if (!s.empty())
			m_elements.front()->LinkEndChild( new TiXmlText(buffer.str().c_str()) );
	}
}

void daeTinyXMLPlugin::writeAttribute( daeMetaAttribute* attr, daeElement* element )
{
	ostringstream buffer;
	attr->memoryToString(element, buffer);
	string str = buffer.str();

	// Don't write the attribute if
	//  - The attribute isn't required AND
	//     - The attribute has no default value and the current value is ""
	//     - The attribute has a default value and the current value matches the default
	if (!attr->getIsRequired()) {
		if(!attr->getDefaultValue()  &&  str.empty())
			return;
		if(attr->getDefaultValue()  &&  attr->compareToDefault(element) == 0)
			return;
	}

	m_elements.front()->SetAttribute(attr->getName(), str.c_str());
}

#endif // DOM_INCLUDE_TINYXML
