/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeMetaSequence.h>

daeMetaSequence::daeMetaSequence( daeMetaElement *container, daeMetaCMPolicy *parent, daeUInt ordinal,
									daeInt minO, daeInt maxO) : 
									daeMetaCMPolicy( container, parent, ordinal, minO, maxO )
{}

daeMetaSequence::~daeMetaSequence()
{}

daeElement *daeMetaSequence::placeElement( daeElement *parent, daeElement *child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after ) {
	(void)offset;
	if ( _maxOccurs == -1 ) {
		//Needed to prevent infinate loops. If unbounded check to see if you have the child before just trying to place
		if ( findChild( child->getElementName() ) == NULL ) {
			return NULL;
		}
	}

	size_t cnt = _children.getCount();
	for ( daeInt i = 0; ( i < _maxOccurs || _maxOccurs == -1 ); i++ ) {
		for ( size_t x = 0; x < cnt; x++ ) {
			if ( _children[x]->placeElement( parent, child, ordinal, i, before, after ) != NULL ) {
				ordinal = ordinal + (i * ( _maxOrdinal + 1 )) + _ordinalOffset;
				return child;
			}
		}
	}
	return NULL;
}

daeBool daeMetaSequence::removeElement( daeElement *parent, daeElement *child ) {
	size_t cnt = _children.getCount();
	for ( size_t x = 0; x < cnt; x++ ) {
		if ( _children[x]->removeElement( parent, child ) ) {
			return true;
		}
	}
	return false;
}

daeMetaElement * daeMetaSequence::findChild( daeString elementName ) {
	daeMetaElement *me = NULL;
	size_t cnt = _children.getCount();
	for ( size_t x = 0; x < cnt; x++ ) {
		me = _children[x]->findChild( elementName );
		if ( me != NULL ) {
			return me;
		}
	}
	return NULL;
}

void daeMetaSequence::getChildren( daeElement *parent, daeElementRefArray &array ) {
	size_t cnt = _children.getCount();
	for ( size_t x = 0; x < cnt; x++ ) {
		_children[x]->getChildren( parent, array );
	}
}
