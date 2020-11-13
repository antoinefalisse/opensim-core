#ifndef __InverseKinematicsKSTool_h__
#define __InverseKinematicsKSTool_h__
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  InverseKinematicsKSTool.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Wouter Aerts, Friedl De Groote, Antoine Falisse      *
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

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include "Tool.h"
#include <SimTKcommon/internal/ReferencePtr.h>

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

class Model;
class MarkersReference;
class CoordinateReference;

//=============================================================================
//=============================================================================
/**
 * A Tool that performs an Inverse Kinematics analysis with a given model.
 * Inverse kinematics (Kalman smoothing) is the solution of internal
 * coordinates that poses the model in the most likely pose given the the
 * observation of markers in spatial coordinates, the landmark locations
 * (markers) affixed to the model, and the process model expressing that
 *  human motion is smooth.
 *
 * @author Ajay Seth, Friedl De Groote, Wouter Aerts, Antoine Falisse
 * @version 3.0
 */
class OSIMTOOLS_API InverseKinematicsKSTool: public Tool {
OpenSim_DECLARE_CONCRETE_OBJECT(InverseKinematicsKSTool, Tool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:

    /** Pointer to the model being investigated. */
    SimTK::ReferencePtr<Model> _model;

public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
            "Name/path to the xml .osim file used to load a model to be analyzed.");

    OpenSim_DECLARE_PROPERTY(constraint_weight, double,
            "The relative weighting of kinematic constraint errors. By default this "
            "is Infinity, which means constraints are strictly enforced as part of "
            "the optimization and are not appended to the objective (cost) function. "
            "Any other non-zero positive scalar is the penalty factor for "
            "constraint violations.");

    OpenSim_DECLARE_PROPERTY(accuracy, double,
            "The accuracy of the solution in absolute terms, i.e. the number of "
            "significant digits to which the solution can be trusted. Default 1e-5.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(
            IKTaskSet,
            "Markers and coordinates to be considered (tasks) and their weightings. "
            "The sum of weighted-squared task errors composes the cost function.");

    OpenSim_DECLARE_PROPERTY(marker_file, std::string,
            "TRC file (.trc) containing the time history of observations of marker "
            "positions obtained during a motion capture experiment. Markers in this "
            "file that have a corresponding task and model marker are included.");


    OpenSim_DECLARE_PROPERTY(coordinate_file, std::string,
            "The name of the storage (.sto or .mot) file containing the time "
            "history of coordinate observations. Coordinate values from this file are "
            "included if there is a corresponding model coordinate and task. ");

    OpenSim_DECLARE_LIST_PROPERTY_SIZE(
            time_range, double, 2, "The time range for the study.");

    OpenSim_DECLARE_PROPERTY(report_errors, bool,
            "Flag (true or false) indicating whether or not to report marker "
            "errors from the inverse kinematics solution.");

    OpenSim_DECLARE_PROPERTY(output_motion_file, std::string,
            "Name of the resulting inverse kinematics motion (.mot) file.");


    OpenSim_DECLARE_PROPERTY(report_marker_locations, bool,
            "Flag indicating whether or not to report model marker locations. "
            "Note, model marker locations are expressed in Ground.");

    OpenSim_DECLARE_PROPERTY(order_smoother, int,
            "Order of the smoother. Derivatives up to order_smoother+1 "
            "will be estimated (default is 4).")

    OpenSim_DECLARE_PROPERTY(time_scale, double,
            "Scale factor for the time. For numerical reasons, choose "
            "time_scale such that the generalized coordinates and their "
            "derivatives have the same order of magnitude (default is 20).");

    OpenSim_DECLARE_PROPERTY(sd_process, double,
            "Estimate of the standard deviation of the (order_smoother+1)th "
            "derivative of the generalized coordinates with order_smoother "
            "the order of the filter (default is 15).");

    OpenSim_DECLARE_PROPERTY(sd_meas, double,
            "Estimate of the standard deviation of the marker error "
            "(default is 0.03).");

    OpenSim_DECLARE_PROPERTY(use_visualizer, bool,
            "Whether or not to use the visualizer when running the KS tool "
            "(default is false).");



//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~InverseKinematicsKSTool();
    InverseKinematicsKSTool();
    InverseKinematicsKSTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;

    /* Handle reading older formats/Versioning */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;

    //---- Setters and getters for various attributes
    void setModel(Model& aModel) { _model = &aModel; };
    void setStartTime(double d) { upd_time_range(0) = d; };
    double getStartTime() const { return get_time_range(0); };

    void setEndTime(double d) { upd_time_range(1) = d; };
    double getEndTime() const { return get_time_range(1); };

    void setMarkerDataFileName(const std::string& markerDataFileName) {
        upd_marker_file() = markerDataFileName;
    };
    const std::string& getMarkerDataFileName() const {
        return get_marker_file();
    };

    void setCoordinateFileName(const std::string& coordDataFileName) {
        upd_coordinate_file() = coordDataFileName;
    };
    const std::string& getCoordinateFileName() const {
        return get_coordinate_file();
    };

private:
	void constructProperties();

public:
    //-------------------------------------------------------------------------
    // OPERATORS
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // GET AND SET
    //-------------------------------------------------------------------------
    void setOutputMotionFileName(const std::string aOutputMotionFileName) {
        upd_output_motion_file() = aOutputMotionFileName;
    }
    std::string getOutputMotionFileName() { return get_output_motion_file(); }
    IKTaskSet& getIKTaskSet() { return upd_IKTaskSet(); }

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	bool run() override SWIG_DECLARE_EXCEPTION;

    /** @cond **/ // hide from Doxygen
    // For testing/debugging it is necessary to know exactly what are the
    // MarkersReference (set of marker trajectories and their weights) and
    // CoordinateReferences that are being used by the InverseKinematicsSolver.
    void populateReferences(MarkersReference& markersReference,
        SimTK::Array_<CoordinateReference>&coordinateReferences,
        Array<std::string>& mNotUsed, Array<std::string>& mInTaskSet,
        Array<std::string>& mWeightTooHigh) const;
    /** @endcond **/

    #ifndef SWIG
    // The map provides the index of each state variable in
    // SimTK::State::getY() from its state variable path string.
    // Copied from MocoUtilities.
    std::unordered_map<std::string, int> createSystemYIndexMap(const Model& model);
    #endif


//=============================================================================
};	// END of class InverseKinematicsKSTool
//=============================================================================
} // namespace

#endif // __InverseKinematicsKSTool_h__
