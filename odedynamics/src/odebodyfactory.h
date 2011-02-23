#ifndef __ODEBODYFACTORY__
#define __ODEBODYFACTORY__

#include "odebody.h"
#include "odedynamics.h"
#include <map>
#include <vector>

#include <xercesc/dom/DOMErrorHandler.hpp>
#include <iostream>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLChar.hpp>

#include "xerces_bodies.h"
class ODEBody;
class ODEDynamics;

class XmlJoint;
class XmlBody;
class XmlWorldParser;

class ODEBodyFactory{

private:

	double bodySize;
	double defaultSize ;
	double x ;
	double y ;
	double z;
	double space;
	int count;
	const char* body_file;
	XmlWorldParser* parser;
	map<string, ODEBody*> bodyMap;


public:
	static ODEBodyFactory* instance;
	dWorldID  world;

	ODEBodyFactory(const char* body_file);
//	static ODEBodyFactory* instanceof();
	/**
	 * Returns a new ODEBody which is not necessarily in the
	 * specified position. If no new
	 *
	 * @param parent the ODE Dynamics layer this body will reside in
	 * @param d The device to be attached to a body
	 * @return a new ODEBody if possible, null if factory is out of bodies to make
	 */
	ODEBody* next_body(ODEDynamics* parent, Device* d);
	void create_joints();
	bool empty();
	int numBodies();
	const char* getBodyFile();
	void setBodyFile(const char* file);

};

#endif //__ODEBODYFACTORY__
