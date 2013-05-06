/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dae/domAny.h>
#include <dae/daeMetaAttribute.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>
#include <dae/daeErrorHandler.h>

daeElementRef
domAny::create(DAE& dae)
{
	domAnyRef ref = new domAny;
	return ref;
}


daeMetaElement *
domAny::registerElement(DAE& dae)
{
	daeMetaElement *_Meta = new daeMetaElement(dae);
	_Meta->setName( "any" );
	_Meta->registerClass(domAny::create);
	_Meta->setIsInnerClass( true );

	daeMetaCMPolicy *cm = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaAny( _Meta, cm, 0, 0, -1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );
	_Meta->setAllowsAny( true );
	
	_Meta->addContents(daeOffsetOf(domAny,_contents));
	_Meta->addContentsOrder(daeOffsetOf(domAny,_contentsOrder));
	
	//VALUE
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAny , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	_Meta->setElementSize(sizeof(domAny));
	_Meta->validate();

	return _Meta;
}

domAny::~domAny() {
	// domAny objects own their corresponding daeMetaElement
	delete _meta;
}

// Implementation of daeMetaAttribute that understands how domAny works
class domAnyAttribute : public daeMetaAttribute {
public:
	virtual daeChar* getWritableMemory(daeElement* e) {
		return (daeChar*)&((domAny*)e)->attrs[_offset];
	}
};

daeBool domAny::setAttribute(daeString attrName, daeString attrValue) {
	if (_meta == NULL)
		return false;
	
	//if the attribute already exists set it.
	if (daeElement::setAttribute(attrName, attrValue))
		return true;

	//else register it and then set it.
	attrs.append("");
	daeMetaAttribute *ma = new domAnyAttribute;
	ma->setName( attrName );
	ma->setType( getDAE()->getAtomicTypes().get("xsString"));
	ma->setOffset((daeInt)attrs.getCount()-1);
	ma->setContainer( _meta );
	if (ma->getType()) {
		_meta->appendAttribute(ma);
		_validAttributeArray.append( true );
		ma->stringToMemory(this, attrValue);
		return true;
	}

	delete ma;
	return false;
}


