/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaElementAttribute.h>
#include <dae/daeMetaElement.h>

daeMetaGroup::daeMetaGroup( daeMetaElementAttribute *econ, daeMetaElement *container, 
							daeMetaCMPolicy *parent, daeUInt ordinal, daeInt minO, daeInt maxO) : 
							daeMetaCMPolicy( container, parent, ordinal, minO, maxO ), _elementContainer( econ )
{}

daeMetaGroup::~daeMetaGroup()
{
	if ( _elementContainer != NULL ) {
		delete _elementContainer;
	}
}

daeElement *daeMetaGroup::placeElement( daeElement *parent, daeElement *child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after ) {
	(void)offset;
	daeString nm = child->getElementName();
	if ( findChild( nm ) == NULL ) {
		return false;
	}
	daeElementRef el;

	//check if the element trying to be placed is a group element. If so Just add it don't create a new one.
	if ( strcmp( nm, _elementContainer->getName() ) == 0 ) {
		if ( _elementContainer->placeElement(parent, child, ordinal, offset ) != NULL ) {
			return child;
		}
	}

#if 1
	daeInt elCnt = _elementContainer->getCount(parent);
	//check existing groups
      //This doesn't work properly. Because the choice can't check if you make two decisions you cannot fail
	  //here when you are supposed to. Luckily the current schema just has groups with single choices so
	  //every element needs a new group container. Wasteful but thats how the schema is and its how it works.
	for ( daeInt x = 0; x < elCnt; x++ ) {
		daeMemoryRef mem = _elementContainer->get(parent, x );
		if ( mem != NULL ) {
			el = *(daeElementRef*)mem;
		}
		if ( el == NULL ) {
			continue;
		}
		if ( before != NULL ) {
			if ( _elementContainer->_elementType->placeBefore( before, el, child, &ordinal ) ) {
				ordinal = ordinal + _ordinalOffset;
				return el;
			}
		}
		else if ( after != NULL ) {
			if ( _elementContainer->_elementType->placeAfter( after, el, child, &ordinal ) ) {
				ordinal = ordinal + _ordinalOffset;
				return el;
			}
		}
		else {
			if ( _elementContainer->_elementType->place( el, child, &ordinal ) ) {
				ordinal = ordinal + _ordinalOffset;
				return el;
			}
		}
	}
#endif
	//if you couldn't place in existing groups make a new one if you can
	el = _elementContainer->placeElement(parent, _elementContainer->_elementType->create(), ordinal, offset );
	if ( el != NULL ) {
		//el = *(daeElementRef*)_elementContainer->get(parent, elCnt );
		if ( before != NULL ) {
			if ( _elementContainer->_elementType->placeBefore( before, el, child, &ordinal ) ) {
				ordinal = ordinal + _ordinalOffset;
				return el;
			}
		}
		else if ( after != NULL ) {
			if ( _elementContainer->_elementType->placeAfter( after, el, child, &ordinal ) ) {
				ordinal = ordinal + _ordinalOffset;
				return el;
			}
		}
		else {
			if ( _elementContainer->_elementType->place( el, child, &ordinal ) ) {
				ordinal = ordinal + _ordinalOffset;
				return el;
			}
		}
	}
	return NULL;
}

daeBool daeMetaGroup::removeElement( daeElement *parent, daeElement *child ) {
	daeElementRef el;
	daeInt elCnt = _elementContainer->getCount(parent);
	for ( daeInt x = 0; x < elCnt; x++ ) {
		daeMemoryRef mem = _elementContainer->get(parent, x );
		if ( mem != NULL ) {
			el = *(daeElementRef*)mem;
		}
		if ( el == NULL ) {
			continue;
		}
		if ( el->removeChildElement( child ) ) {
			//check if there are any more children in this group. If not remove the group container element too.
			daeElementRefArray array;
			getChildren( parent, array );
			if ( array.getCount() == 0 )
			{
				_elementContainer->removeElement( parent, el );
			}
			return true;
		}
	}
	return false;
}

daeMetaElement * daeMetaGroup::findChild( daeString elementName ) {
	if ( strcmp( _elementContainer->getName(), elementName ) == 0 ) {
		return _elementContainer->getElementType();
	}
	return _elementContainer->_elementType->getCMRoot()->findChild( elementName );
}

void daeMetaGroup::getChildren( daeElement *parent, daeElementRefArray &array ) {
	size_t cnt = _elementContainer->getCount( parent );
	for ( size_t x = 0; x < cnt; x++ ) {
		(*((daeElementRef*)_elementContainer->get(parent, (daeInt)x )))->getChildren( array );
		/*daeElementRef el = (*((daeElementRef*)_elementContainer->get(parent, (daeInt)x )));
		size_t cnt2 = _children.getCount();
		for ( size_t i = 0; i < cnt2; i++ ) {
			_children[i]->getChildren( el, array );
		}*/
	}
	//_elementContainer->_elementType->getChildren( parent, array );
}

