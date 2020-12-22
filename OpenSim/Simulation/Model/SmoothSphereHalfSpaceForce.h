#ifndef OPENSIM_SMOOTH_SPHERE_HALF_SPACE_FORCE_H_
#define OPENSIM_SMOOTH_SPHERE_HALF_SPACE_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim: SmoothSphereHalfSpaceForce.h                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse, Gil Serrancoli                                 *
 * Contributors: Peter Eastman                                                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "Force.h"
#include <OpenSim/Common/Set.h>

namespace OpenSim {

/** This compliant contact force model is similar to HuntCrossleyForce, except
that this model applies force even when not in contact. Unlike
HuntCrossleyForce, the normal force is differentiable as a function of
penetration depth. This component is designed for use in gradient-based
optimizations, in which the model is required to be differentiable. This
component models contact between a single sphere and a single half space.
This force does NOT use ContactGeometry objects; the description of the
contact geometries is done through properties of this component.

### Constant contact force

This force applies a constant contact force even when the sphere and half-space
are not contacting. This constant force is set with the constant_contact_force
property. Its default value is appropriate for walking; the value may need to
be adjusted for different contact scenarios or models with a very light mass.

@see SimTK::SmoothSphereHalfSpaceForce

The graph below compares the smooth approximation of the Hertz force to that
from HuntCrossleyForce.

\htmlonly <style>div.image
img[src="SmoothSphereHalfSpaceForce_HertzForce.png"]{width:750px;}</style>
\endhtmlonly
@image html SmoothSphereHalfSpaceForce_HertzForce.png "Curves produced using E=1e6, R=0.8, cf=1e-5, and bd=300"

The graph below compares the smooth approximation of the dissipative force to
 that from HuntCrossleyForce.

\htmlonly <style>div.image
img[src="SmoothSphereHalfSpaceForce_HuntCrossleyForce.png"]{width:750px;}</style>
\endhtmlonly
@image html SmoothSphereHalfSpaceForce_HuntCrossleyForce.png "Curves produced using x=0.1, E=1e6, R=0.8, c=2, cf=1e-5, bd=300, and bv=50"

Serrancoli, G., Falisse, A., Dembia, C., Vantilt, J., Tanghe, K., Lefeber, D.,
Jonkers, I., De Schutter, J., De Groote, F. (2019). Subject-exoskeleton contact
model calibration leads to accurate interaction force predictions. IEEE
Transactions on Neural Systems and Rehabilitation Engineering, 1–1.
doi:10.1109/tnsre.2019.2924536 */
class OSIMSIMULATION_API SmoothSphereHalfSpaceForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(SmoothSphereHalfSpaceForce, Force);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================
    OpenSim_DECLARE_PROPERTY(stiffness, osim_double_adouble,
            "The stiffness constant (i.e., plain strain modulus), "
            "default is 1 (N/m^2)");
    OpenSim_DECLARE_PROPERTY(dissipation, osim_double_adouble,
            "The dissipation coefficient, default is 0 (s/m).");
    OpenSim_DECLARE_PROPERTY(static_friction, osim_double_adouble,
            "The coefficient of static friction, default is 0.");
    OpenSim_DECLARE_PROPERTY(dynamic_friction, osim_double_adouble,
            "The coefficient of dynamic friction, default is 0.");
    OpenSim_DECLARE_PROPERTY(viscous_friction, osim_double_adouble,
            "The coefficient of viscous friction, default is 0.");
    OpenSim_DECLARE_PROPERTY(transition_velocity, osim_double_adouble,
            "The transition velocity, default is 0.01 (m/s).");
    OpenSim_DECLARE_PROPERTY(constant_contact_force, osim_double_adouble,
            "The constant that enforces non-null derivatives, "
            "default is 1e-5 (N).");
    OpenSim_DECLARE_PROPERTY(hertz_smoothing, osim_double_adouble,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the Hertz force. The larger the "
            "steeper the transition but the worse for optimization, "
            "default is 300.");
    OpenSim_DECLARE_PROPERTY(hunt_crossley_smoothing, osim_double_adouble,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the Hunt-Crossley force. The larger "
            "the steeper the transition but the worse for optimization, "
            "default is 50.");
    OpenSim_DECLARE_PROPERTY(contact_sphere_radius, osim_double_adouble,
            "The radius of the contact sphere.");
    OpenSim_DECLARE_PROPERTY(contact_sphere_location, SimTK::Vec3,
            "The location of the contact sphere in the sphere frame.");
    OpenSim_DECLARE_PROPERTY(contact_half_space_location, SimTK::Vec3,
            "The location of the contact half space in the half space frame, "
            "default is Vec3(0).");
    OpenSim_DECLARE_PROPERTY(contact_half_space_orientation, SimTK::Vec3,
            "The orientation of the contact half space in the half space "
            "frame (body-fixed XYZ Euler angles), default represents ground "
            "(0,0,-0.5*SimTK::Pi).");
    //OpenSim_DECLARE_PROPERTY(force_visualization_radius, osim_double_adouble,
    //        "The radius of the cylinder that visualizes contact "
    //        "forces generated by this force component. Default: 0.01 m");
    //OpenSim_DECLARE_OPTIONAL_PROPERTY(force_visualization_scale_factor, osim_double_adouble,
    //        "(Optional) The scale factor that determines the length of the "
    //        "cylinder that visualizes contact forces generated by this force "
    //        "component. The cylinder will be one meter long when the contact "
    //        "force magnitude is equal to this value. If this property is not "
    //        "specified, the total weight of the model is used "
    //        "as the scale factor.")

    //=========================================================================
    // SOCKETS
    //=========================================================================
    OpenSim_DECLARE_SOCKET(sphere_frame, PhysicalFrame,
            "The body to which the contact sphere is attached.");
    OpenSim_DECLARE_SOCKET(half_space_frame, PhysicalFrame,
            "The body to which the contact half space is attached.");

    //=========================================================================
    // PUBLIC METHODS
    //=========================================================================
    SmoothSphereHalfSpaceForce();

    SmoothSphereHalfSpaceForce(const std::string& name,
            const PhysicalFrame& contactSphereBodyFrame,
            const PhysicalFrame& contactHalfSpaceBodyFrame);

	// TODO: unclear why it does not work
    //SmoothSphereHalfSpaceForce(const std::string& name,
    //        const PhysicalFrame& contactSphereBodyFrame,
    //        SimTK::Vec3 contactSphereLocation,
    //        osim_double_adouble contactSphereRadius,
    //        const PhysicalFrame& contactHalfSpaceBodyFrame,
    //        SimTK::Vec3 contactHalfSpaceLocation,
    //        SimTK::Vec3 contactHalfSpaceOrientation);

    //=========================================================================
    // REPORTING
    //=========================================================================
    /// Obtain names of the quantities (column labels) of the force values to
    /// be reported. The order is the three forces (XYZ) and three torques (XYZ)
    /// applied on the sphere followed by the three forces (XYZ) and three
    /// torques (XYZ) applied on the half space. Forces and torques are
    /// expressed in the ground frame.
    OpenSim::Array<std::string> getRecordLabels() const override;
    /// Obtain the values to be reported that correspond to the labels. The
    /// values are expressed in the ground frame.
    OpenSim::Array<osim_double_adouble> getRecordValues(
            const SimTK::State& state) const override;

protected:
    /// Create a SimTK::Force which implements this Force.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    //void extendRealizeInstance(const SimTK::State& state) const override;
    //void generateDecorations(bool fixed, const ModelDisplayHints& hints,
    //        const SimTK::State& state,
    //        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;

private:
    // INITIALIZATION
    void constructProperties();
    //mutable double m_forceVizScaleFactor;

//=============================================================================
}; // END of class SmoothSphereHalfSpaceForce
//=============================================================================
//=============================================================================

} // namespace OpenSim

#endif // OPENSIM_SMOOTH_SPHERE_HALF_SPACE_FORCE_H_
