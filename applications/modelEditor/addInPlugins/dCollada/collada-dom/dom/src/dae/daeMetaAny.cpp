/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeMetaAny.h>
#include <dae/domAny.h>
#include <dae/daeMetaElementAttribute.h>


daeMetaAny::daeMetaAny( daeMetaElement *container, daeMetaCMPolicy *parent, daeUInt ordinal,
												 daeInt minO, daeInt maxO) : daeMetaCMPolicy( container, parent, ordinal, minO, maxO )
{}

daeMetaAny::~daeMetaAny()
{}

daeElement *daeMetaAny::placeElement( daeElement *parent, daeElement *child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after ) {
	//remove element from praent
	(void)offset;
	(void)before;
	(void)after;
	daeElement::removeFromParent( child );
	child->setParentElement( parent );
	//*************************************************************************
	ordinal = 0;
	return child;
}

daeBool daeMetaAny::removeElement( daeElement *parent, daeElement *child ) {
	(void)parent;
	(void)child;
	return true;
}

daeMetaElement * daeMetaAny::findChild( daeString elementName ) {
	if ( elementName != NULL ) {
		const daeMetaElementRefArray &metas = _container->getDAE()->getAllMetas();
		size_t cnt = metas.getCount();
		for ( size_t x = 0; x < cnt; x++ ) {
			if ( metas[x] && !metas[x]->getIsInnerClass() && strcmp( elementName, metas[x]->getName() ) == 0 ) {
				return metas[x];
			}
		}
	}
	return domAny::registerElement(*_container->getDAE());
}

void daeMetaAny::getChildren( daeElement *parent, daeElementRefArray &array ) {
	(void)parent;
	(void)array;
	//this is taken care of by the _contents in metaElement
}

