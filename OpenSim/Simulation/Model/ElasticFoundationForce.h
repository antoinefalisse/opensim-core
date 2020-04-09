#ifndef OPENSIM_ELASTIC_FOUNDATION_FORCE_H_
#define OPENSIM_ELASTIC_FOUNDATION_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ElasticFoundationForce.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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
// INCLUDE
#include "Force.h"
#include "OpenSim/Common/Set.h"

namespace OpenSim {

//==============================================================================
//                       ELASTIC FOUNDATION FORCE
//==============================================================================
/** This Force subclass implements an elastic foundation contact model. It 
places a spring at the center of each face of each ContactMesh it acts on.  
Those springs interact with all objects (both meshes and other objects) the 
mesh comes in contact with.

@author Peter Eastman **/
class OSIMSIMULATION_API ElasticFoundationForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(ElasticFoundationForce, Force);
public:
    class ContactParameters;
    class ContactParametersSet;
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(contact_parameters, 
        ElasticFoundationForce::ContactParametersSet,
        "Material properties.");
    OpenSim_DECLARE_PROPERTY(transition_velocity, osim_double_adouble,
        "Slip velocity (creep) at which peak static friction occurs.");


//==============================================================================
// PUBLIC METHODS
//==============================================================================
    ElasticFoundationForce();
    explicit ElasticFoundationForce(ContactParameters* params);

    /**
     * Create a SimTK::Force which implements this Force.
     */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    ContactParametersSet& updContactParametersSet();
    const ContactParametersSet& getContactParametersSet();

    /** Takes over ownership of the passed-in object. **/
    void addContactParameters(ContactParameters* params);
    /**
     * Get the transition velocity for switching between static and dynamic friction.
     */
	osim_double_adouble getTransitionVelocity() const;
    /**
     * %Set the transition velocity for switching between static and dynamic friction.
     */
    void setTransitionVelocity(osim_double_adouble velocity);

    /**
     * Access to ContactParameters. Methods assume size 1 of ContactParametersSet and add one ContactParameter if needed
     */
	osim_double_adouble getStiffness() ;
    void setStiffness(osim_double_adouble stiffness) ;
	osim_double_adouble getDissipation() ;
    void setDissipation(osim_double_adouble dissipation);
	osim_double_adouble getStaticFriction() ;
    void setStaticFriction(osim_double_adouble friction);
	osim_double_adouble getDynamicFriction() ;
    void setDynamicFriction(osim_double_adouble friction);
	osim_double_adouble getViscousFriction() ;
    void setViscousFriction(osim_double_adouble friction);
    void addGeometry(const std::string& name);

    //-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    OpenSim::Array<std::string> getRecordLabels() const override ;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<osim_double_adouble> getRecordValues(const SimTK::State& state) const override ;
private:
    // INITIALIZATION
    void constructProperties();

//==============================================================================
};  // END of class ElasticFoundationForce
//==============================================================================
//==============================================================================

#ifndef SWIG
//==============================================================================
//              ELASTIC FOUNDATION FORCE :: CONTACT PARAMETERS
//==============================================================================

class OSIMSIMULATION_API ElasticFoundationForce::ContactParameters 
:   public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(ElasticFoundationForce::ContactParameters, 
                                Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(geometry, std::string,
        "Names of geometry objects affected by these parameters.");

    OpenSim_DECLARE_PROPERTY(stiffness, osim_double_adouble,
        "");
    OpenSim_DECLARE_PROPERTY(dissipation, osim_double_adouble,
        "");
    OpenSim_DECLARE_PROPERTY(static_friction, osim_double_adouble,
        "");
    OpenSim_DECLARE_PROPERTY(dynamic_friction, osim_double_adouble,
        "");
    OpenSim_DECLARE_PROPERTY(viscous_friction, osim_double_adouble,
        "");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    ContactParameters();
    ContactParameters(osim_double_adouble stiffness,
		osim_double_adouble dissipation,
		osim_double_adouble staticFriction, osim_double_adouble dynamicFriction,
		osim_double_adouble viscousFriction);

    const Property<std::string>& getGeometry() const;
    Property<std::string>& updGeometry();
    void addGeometry(const std::string& name);
	osim_double_adouble getStiffness() const;
    void setStiffness(osim_double_adouble stiffness);
	osim_double_adouble getDissipation() const;
    void setDissipation(osim_double_adouble dissipation);
	osim_double_adouble getStaticFriction() const;
    void setStaticFriction(osim_double_adouble friction);
	osim_double_adouble getDynamicFriction() const;
    void setDynamicFriction(osim_double_adouble friction);
	osim_double_adouble getViscousFriction() const;
    void setViscousFriction(osim_double_adouble friction);

private:
    void constructProperties();
};


//==============================================================================
//             ELASTIC FOUNDATION FORCE :: CONTACT PARAMETERS SET
//==============================================================================

class OSIMSIMULATION_API ElasticFoundationForce::ContactParametersSet 
:   public Set<ElasticFoundationForce::ContactParameters> {
OpenSim_DECLARE_CONCRETE_OBJECT(ElasticFoundationForce::ContactParametersSet, 
                                Set<ElasticFoundationForce::ContactParameters>);

public:
    ContactParametersSet();

private:
    void setNull();
};

#endif

} // end of namespace OpenSim

#endif // OPENSIM_ELASTIC_FOUNDATION_FORCE_H_
