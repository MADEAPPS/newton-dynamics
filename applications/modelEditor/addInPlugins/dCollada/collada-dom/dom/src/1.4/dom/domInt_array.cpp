/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 

#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domInt_array.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInt_array::create(DAE& dae)
{
	domInt_arrayRef ref = new domInt_array(dae);
	return ref;
}


daeMetaElement *
domInt_array::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int_array" );
	meta->registerClass(domInt_array::create);

	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("ListOfInts"));
		ma->setOffset( daeOffsetOf( domInt_array , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domInt_array , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInt_array , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: count
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domInt_array , attrCount ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: minInclusive
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "minInclusive" );
		ma->setType( dae.getAtomicTypes().get("xsInteger"));
		ma->setOffset( daeOffsetOf( domInt_array , attrMinInclusive ));
		ma->setContainer( meta );
		ma->setDefaultString( "-2147483648");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: maxInclusive
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "maxInclusive" );
		ma->setType( dae.getAtomicTypes().get("xsInteger"));
		ma->setOffset( daeOffsetOf( domInt_array , attrMaxInclusive ));
		ma->setContainer( meta );
		ma->setDefaultString( "2147483647");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInt_array));
	meta->validate();

	return meta;
}

