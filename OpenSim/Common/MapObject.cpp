/* -------------------------------------------------------------------------- *
 *                          OpenSim:  MapObject.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <iostream>
#include <string>
#include <cassert>
#include "osimCommonDLL.h"
#include "MapObject.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MapItem::
MapItem(const string &key,const string &val)
{
    constructProperties();
    upd_key() = key;
    upd_value() = val;
}

MapItem::MapItem()
{
    constructProperties();
}

void MapItem::constructProperties()
{
    constructProperty_key("key_not_set");
    constructProperty_value("value_not_set");
}

MapObject::MapObject() {
    constructProperty_list_MapItems();
};

MapObject::MapObject(const std::string &aFileName) : Object(aFileName, false) {
    constructProperty_list_MapItems();
    updateFromXMLDocument();

}
