/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <sstream>

#include <dae/daeIDRef.h>
#include <dae/daeDatabase.h>
#include <dae/daeErrorHandler.h>

using namespace std;

void
daeIDRef::initialize()
{
	id = "";
	container = NULL;
}

daeIDRef::daeIDRef()
{
	initialize();
}

daeIDRef::daeIDRef(daeString IDRefString)
{
	initialize();
	setID(IDRefString);
}

daeIDRef::daeIDRef(const daeIDRef& copyFrom_)
{
	initialize();
	copyFrom(copyFrom_);
}

daeIDRef::daeIDRef(daeElement& container) {
	initialize();
	setContainer(&container);
}


void
daeIDRef::reset()
{
	setID("");
}

bool daeIDRef::operator==(const daeIDRef& other) const {
	return (!strcmp(other.getID(), getID()));
}

daeIDRef &daeIDRef::operator=( const daeIDRef& other) {
	if (!container)
		container = other.container;
	id = other.getID();
	return *this;
}

daeString
daeIDRef::getID() const
{
	return id.c_str();
}

void
daeIDRef::setID(daeString _IDString)
{
	id = _IDString ? _IDString : "";
}

daeElement* daeIDRef::getElement() const {
	if (container)
		return container->getDAE()->getIDRefResolvers().resolveElement(id, container->getDocument());
	return NULL;
}

daeElement* daeIDRef::getContainer() const {
	return(container);
}

void daeIDRef::setContainer(daeElement* cont) {
	container = cont;
}

void
daeIDRef::print()
{
	fprintf(stderr,"id = %s\n",id.c_str());
	fflush(stderr);
}

// These methods are provided for backward compatibility only.
void daeIDRef::validate() { }

void daeIDRef::resolveElement( daeString ) { }

void daeIDRef::resolveID() { }

daeIDRef &daeIDRef::get( daeUInt idx ) {
	(void)idx;
	return *this;
}

size_t daeIDRef::getCount() const {
	return 1;
}

daeIDRef& daeIDRef::operator[](size_t index) {
	(void)index;
	return *this;
}

void
daeIDRef::copyFrom(const daeIDRef& copyFrom) {
	*this = copyFrom;
}

daeIDRef::ResolveState daeIDRef::getState() const {
	if (id.empty())
		return id_empty;
	if (getElement())
		return id_success;
	return id_failed_id_not_found;
}


daeIDRefResolver::daeIDRefResolver(DAE& dae) : dae(&dae) { }

daeIDRefResolver::~daeIDRefResolver() { }


daeDefaultIDRefResolver::daeDefaultIDRefResolver(DAE& dae) : daeIDRefResolver(dae) { }

daeDefaultIDRefResolver::~daeDefaultIDRefResolver() { }

daeString
daeDefaultIDRefResolver::getName()
{
	return "DefaultIDRefResolver";
}

daeElement* daeDefaultIDRefResolver::resolveElement(const string& id, daeDocument* doc) {
	return doc ? dae->getDatabase()->idLookup(id, doc) : NULL;
}


daeIDRefResolverList::daeIDRefResolverList() { }

daeIDRefResolverList::~daeIDRefResolverList() {
	for (size_t i = 0; i < resolvers.getCount(); i++)
		delete resolvers[i];
}

void daeIDRefResolverList::addResolver(daeIDRefResolver* resolver) {
	resolvers.append(resolver);
}

void daeIDRefResolverList::removeResolver(daeIDRefResolver* resolver) {
	resolvers.remove(resolver);
}

daeElement* daeIDRefResolverList::resolveElement(const string& id, daeDocument* doc) {
	for(size_t i = 0; i < resolvers.getCount(); i++)
		if (daeElement* el = resolvers[i]->resolveElement(id, doc))
			return el;
	return NULL;
}
