/* -------------------------------------------------------------------------- *
 *                    OpenSim:  InverseKinematicsKSTool.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "InverseKinematicsKSTool.h"

#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"
#include "IKTaskSet.h"

#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Stopwatch.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InverseKinematicsKSTool::~InverseKinematicsKSTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InverseKinematicsKSTool::InverseKinematicsKSTool() : Tool()
{
    constructProperties();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
InverseKinematicsKSTool::InverseKinematicsKSTool(const string &aFileName, bool aLoadModel) :
    Tool(aFileName, true)
{
    constructProperties();
    updateFromXMLDocument();
}

//_____________________________________________________________________________
/**
 * Construct properties
 */
void InverseKinematicsKSTool::constructProperties()
{
    constructProperty_model_file("");
    constructProperty_constraint_weight(Infinity);
    constructProperty_accuracy(1e-5);
    constructProperty_IKTaskSet(IKTaskSet());
    constructProperty_marker_file("");
    constructProperty_coordinate_file("");
    Array<double> range{Infinity, 2};
    range[0] = -Infinity; // Make range -Infinity to Infinity unless limited by
                          // data
    constructProperty_time_range(range);

    constructProperty_report_errors(true);
    constructProperty_output_motion_file("");
    constructProperty_report_marker_locations(false);

    constructProperty_order_smoother(4);
    constructProperty_time_scale(20.0);
    constructProperty_sd_process(15.0);
    constructProperty_sd_meas(0.03);
    constructProperty_use_visualizer(false);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the inverse kinematics (Kalman smoothing) tool.
 */
bool InverseKinematicsKSTool::run()
{
    bool success = false;
    bool modelFromFile=true;
    Kinematics* kinematicsReporter = nullptr;
    try{
        //Load and create the indicated model
        if (_model.empty()) {
            OPENSIM_THROW_IF_FRMOBJ(get_model_file().empty(), Exception,
                    "No model filename was provided.");
            _model.reset(new Model(get_model_file()));
        }
        else
            modelFromFile = false;

        // although newly loaded model will be finalized
        // there is no guarantee that the _model has not been edited/modified
        _model->finalizeFromProperties();
        _model->printBasicInfo();

        // Do the maneuver to change then restore working directory so that the
        // parsing code behaves properly if called from a different directory.
        string saveWorkingDirectory = IO::getCwd();
        string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
        IO::chDir(directoryOfSetupFile);

        // Define reporter for output
        kinematicsReporter = new Kinematics();
        kinematicsReporter->setRecordAccelerations(false);
        kinematicsReporter->setInDegrees(true);
        _model->addAnalysis(kinematicsReporter);

        cout<<"Running tool "<<getName()<<".\n";

        // Get the trial name to label data written to files
        string trialName = getName();

        // Set the visualizer if desired
        if (get_use_visualizer()) {
            _model->setUseVisualizer(true);
            _model->updForceSet().clearAndDestroy();
        }

        // Initialize the model's underlying system and get its default state.
        SimTK::State& s = _model->initSystem();

        SimTK::Vector qDefaults = s.getQ();

        //Convert old Tasks to references for assembly and tracking
        MarkersReference markersReference;
        SimTK::Array_<CoordinateReference> coordinateReferences;
        Array<std::string> mNotUsed;
        Array<std::string> mInTaskSet;
        Array<std::string> mWeightTooHigh;
        // populate the references according to the setting of this Tool
        populateReferences(markersReference, coordinateReferences, mNotUsed,
                mInTaskSet, mWeightTooHigh);
        // Control if all the markers in the model are also in the TaskSet
        const MarkerSet& modelMarkerSet = _model->getMarkerSet();
        for(int index_model=0; index_model < modelMarkerSet.getSize(); index_model++){
            string modelMarkerName = modelMarkerSet.get(index_model).getName();
            bool notInTaskSet = true;
            for (int mInTaskSetIndex = 0; mInTaskSetIndex < mInTaskSet.size(); mInTaskSetIndex++) {
                if (modelMarkerName.compare(mInTaskSet[mInTaskSetIndex]) == 0){
                    notInTaskSet = false;
                    break;
                }
            }
            if (notInTaskSet) {
                mNotUsed.append(modelMarkerName);
            }
        }

        // Determine the start time, if the provided time range is not
        // specified then use time from marker reference.
        // Adjust the time range for the tool if the provided range exceeds
        // that of the marker data.
        SimTK::Vec2 markersValidTimeRange = markersReference.getValidTimeRange();
        double start_time = (markersValidTimeRange[0] > get_time_range(0)) ?
            markersValidTimeRange[0] : get_time_range(0);
        double final_time = (markersValidTimeRange[1] < get_time_range(1)) ?
            markersValidTimeRange[1] : get_time_range(1);

        SimTK_ASSERT2_ALWAYS(final_time >= start_time,
            "InverseKinematicsTool final time (%f) is before start time (%f).",
            final_time, start_time);

        const auto& markersTable = markersReference.getMarkerTable();
        const int start_ix = int(
            markersTable.getNearestRowIndexForTime(start_time) );
        const int final_ix = int(
            markersTable.getNearestRowIndexForTime(final_time) );
        const int Nframes = final_ix - start_ix + 1;
        const auto& times = markersTable.getIndependentColumn();

        // create the solver given the input data
        InverseKinematicsSolver ikSolver(*_model, markersReference,
            coordinateReferences, get_constraint_weight());
        ikSolver.setAccuracy(get_accuracy());
        s.updTime() = times[start_ix];
        ikSolver.assemble(s);
        kinematicsReporter->begin(s);

        AnalysisSet& analysisSet = _model->updAnalysisSet();
        analysisSet.begin(s);
        // Get the actual number of markers the Solver is using, which
        // can be fewer than the number of references if there isn't a
        // corresponding model marker for each reference.
        int nMarkers = ikSolver.getNumMarkersInUse();
        SimTK::Array_<double> squaredMarkerErrors(nMarkers, 0.0);
        SimTK::Array_<Vec3> markerLocations(nMarkers, Vec3(0));
        SimTK::Array_<Vec3> measMarkerLocations(nMarkers, Vec3(0));
        std::cout << "Number of markers: " << nMarkers << std::endl;
        std::cout << "Marker(s) not used: " << mNotUsed <<std::endl;

        Stopwatch watch;
        double dt = 1.0/markersReference.getSamplingFrequency();

        // Set up process model
        Matrix F(get_order_smoother(), get_order_smoother()); F=0.0;
        Matrix Q(get_order_smoother(), get_order_smoother()); Q=0.0;
        Vector G(get_order_smoother()); G=0.0;
        for (int j=0; j<get_order_smoother(); ++j) F(j,j) = 1;
        double Ts = dt*get_time_scale();
        int fac = 1;
        double diag;
        for (int n=1; n<get_order_smoother(); ++n){
            fac = fac*n;
            diag = pow(Ts,n)/fac;
            for (int j=0; j<get_order_smoother()-n; ++j) F(j,j+n) = diag;
            G(get_order_smoother()-n) = diag;
        }
        fac = fac*get_order_smoother();
        G(0) = pow(Ts,get_order_smoother())/fac;
        Q = pow(get_sd_process(),2)*G*~G;

        // Find locked and clamped joints
        int nq = s.getNQ();
        int nu = s.getNU();
        int ns = nq*get_order_smoother();
        Array<int> iLocked;
        Array<int> iClamped_t;
        Array<double> lowerClampbounds;
        Array<double> upperClampbounds;
        // OpenSim and Simbody have difference conventions for indexing the
        // states; systemYIndexMap relates OpenSim coordinates to Simbody
        // indices.
        auto systemYIndexMap = createSystemYIndexMap(*_model);
        // Loop over coordinates
        for (const auto& coord : _model->getComponentList<Coordinate>()) {
            cout << coord.getName() << endl;
            int iqx = systemYIndexMap[coord.getStateVariableNames()[0]];
            if (coord.getLocked(s)) {
                iLocked.append(iqx);
                cout << coord.getName() << " (" << iqx <<") is locked" << endl;
            }
            else {
                if (coord.getClamped(s)) {
                    iClamped_t.append(iqx);
                    lowerClampbounds.append(coord.getRangeMin());
                    upperClampbounds.append(coord.getRangeMax());
                    cout << coord.getName() << " (" << iqx <<") is clamped" << endl;
                }
            }
        }
        // Note from Antoine when adjusting the code for OpenSim >= 4.0.
        // In the matrix x below, values for unlocked coordinates are gathered
        // in the order of the Simbody states => s.getQ().
        // Later, values for clamped coordinates are processed based on the
        // matrix x. We therefore need to obtain the indices of the clamped
        // coordinates in the matrix x. This piece of code does that. Assume
        // coordinates with indices 5 and 7 are locked, and coordinates with
        // indices 6 and 9 are clamped. In matrix x, values of the coordinates
        // with indices 5 and 7 will not be included. Therefore, in the matrix
        // x, the indices of the clamped coordinates are 5 (6-1) and 7 (9-2).
        // This should be better coded, not very robust but seems to work.
        Array<int> iClamped;
        int nLocked_temp = 0;
        for(int k=0; k<nq; ++k){
            if (iLocked.findIndex(k) != -1) nLocked_temp += 1;
            if (iClamped_t.findIndex(k) != -1) iClamped.append(k-nLocked_temp);
        }
        int nLocked = iLocked.size();
        int nClamped = iClamped.size();
        std::cout << "Number locked: " << iLocked.size() << std::endl;
        std::cout << "Number clamped: " << nClamped << std::endl;
        std::cout << "Lower clamp bounds: " << lowerClampbounds << std::endl;
        std::cout << "Upper clamp bounds: " << upperClampbounds << std::endl;

        // Initial state estimate based on IK (global optimization) solution
        int Nframesinit = 3;
        Matrix qIK(nq,Nframesinit); qIK = 0;
        for (int i = 0; i < Nframesinit; i++) {
            s.updTime() = start_time + i*dt;
            ikSolver.track(s);
            qIK(i) = s.getQ();
        }

        int nqf = nq-nLocked; // number of free coordinates
        int nsf = nqf*get_order_smoother(); // number of free states
        Vector x(nsf); x=0.0;
        int j = 0;
        for(int k=0; k<nq; ++k){
            if (iLocked.findIndex(k) == -1){
                x(j*get_order_smoother()) = qIK(k,0);
                x(j*get_order_smoother()+1) = (qIK(k,2) - qIK(k,0))/(2*Ts);
                x(j*get_order_smoother()+2) = (qIK(k,2) - 2*qIK(k,1) + qIK(k,0))/pow(Ts,2);
                j = j+1;
            }
        }

        s.updTime() = start_time;
        s.updQ() = qIK(0);
        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
        kinematicsReporter->step(s, 0);

        // Initial covariance estimate
        Matrix P(nsf,nsf); P=0.0;
        for(int n=0; n<nsf; ++n) P(n,n) = 1;

        // Initialization
        Vector x_pred(nsf); x_pred=0.0;
        Matrix P_pred(nsf,nsf); P_pred=0.0;

        SimTK::Array_<Vector> x_pred_array(Nframes,Vector(nsf));
        SimTK::Array_<Matrix> P_pred_array(Nframes,Matrix(nsf,nsf));
        SimTK::Array_<Vector> x_kf_array(Nframes,Vector(nsf));
        SimTK::Array_<Matrix> P_kf_array(Nframes,Matrix(nsf,nsf));

        x_kf_array[0] = x;
        P_kf_array[0] = P;

        ////////////
        // FILTER //
        ////////////
        double tol_m = 0.000001;
        // Loop over time frames
        for (int n = 1; n < Nframes; n++) {
            // PREDICT
            for(int i=0; i<nqf; ++i){
                for(int j=0; j<nqf; ++j) {
                    P_pred(i*get_order_smoother(), j*get_order_smoother(),
                            get_order_smoother(), get_order_smoother()) =
                        F*P(i*get_order_smoother(), j*get_order_smoother(),
                                get_order_smoother(), get_order_smoother())*~F;
                }
            }

            for(int i=0; i<nqf; ++i){
                x_pred(i*get_order_smoother(),get_order_smoother()) =
                    F*x(i*get_order_smoother(), get_order_smoother());
                P_pred(i*get_order_smoother(), i*get_order_smoother(),
                        get_order_smoother(), get_order_smoother()) =
                    P_pred(i*get_order_smoother(), i*get_order_smoother(),
                                get_order_smoother(), get_order_smoother()) + Q;
            }

            // Set new state
            s.updTime() = start_time + n*dt;
            Vector qpred(nq); qpred = 0;
            int j=0;
            for(int k=0; k<nq; ++k) {
                if (iLocked.findIndex(k) == -1){
                    qpred(k) = x_pred(j*get_order_smoother());
                    j = j+1;
                }
                else { // Update
                    qpred(k) = qDefaults(k);
                }
            }
            s.updQ() = qpred;

            ikSolver.setState(s); // Function added to AssemblySolver
            _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

            x_pred_array[n] = x_pred;
            P_pred_array[n] = P_pred;

            // UPDATE
            // innovation
            ikSolver.computeCurrentMarkerLocations(markerLocations);
            double nextTime = s.getTime();
            markersReference.getValuesAtTime(nextTime, measMarkerLocations);
            // get markers defined by the model
            const MarkerSet& modelMarkerSet = _model->getMarkerSet();
            Array<Vec3> innov;
            Vec3 diff(0);
            int il = 0;
            int index = 0;
            int index_model = 0;
            bool markerNotMeasured = false;
            // Array<double> measlocations(0.0, 3*nMarkers);
            Array<std::string> mNotUsedInStep = mNotUsed;
            for(int i=0; i < nMarkers; i++){ // Loop over all the markers in the marker-file
            // 1. Check if we have this marker in the model, else ignore it
                index_model = modelMarkerSet.getIndex(ikSolver.getMarkerNameForIndex(i));
                // cout << measMarkerLocations[i] << endl;
                if(index_model >= 0){
                    // 2. Check if marker used for estimation
                    index = mNotUsed.findIndex(ikSolver.getMarkerNameForIndex(i));
                    if (index == -1) {
                        // 3. Check if marker location is measured in current frame
                        // Don't use marker for update if its position is not measured [0 0 0] or if it contains NaN.
                        markerNotMeasured = (SimTK::isNaN(measMarkerLocations[i][0]) || SimTK::isNaN(measMarkerLocations[i][1]) || SimTK::isNaN(measMarkerLocations[i][2]))
                            || (measMarkerLocations[i][0] < tol_m && measMarkerLocations[i][0] > -tol_m && measMarkerLocations[i][1] < tol_m && measMarkerLocations[i][1] > -tol_m && measMarkerLocations[i][2] < tol_m && measMarkerLocations[i][2] > -tol_m);
                        if (markerNotMeasured) {
                            // cout << ikSolver.getMarkerNameForIndex(i) << " not visible" << endl;
                            mNotUsedInStep.append(ikSolver.getMarkerNameForIndex(i));
                        }
                        else {
                            diff = measMarkerLocations[i] - markerLocations[il];
                            innov.append(diff);
                            //std::cout << ikSolver.getMarkerNameForIndex(i) << "  " << measMarkerLocations[i] << "  " << markerLocations[il] << "  " << diff <<std::endl;
                        }
                        il = il + 1;
                    }
                }
            }
            int nMarkersUsed = innov.getSize();
            // cout << "Time = " << s.getTime() << ": number of used markers = " << nMarkersUsed << endl;
            Vector innovation(3*nMarkersUsed); innovation=0.0;
            for(int j=0; j<nMarkersUsed; ++j) {
                for(int k=0; k<3; ++k)
                    innovation[j*3+k] = innov[j][k];
            }
            // Construct Jacobian
            Matrix measModelJ(3*nMarkersUsed, nu); measModelJ=0.0;
            int indexU = 0;
            index = 0;
            il = 0;
            //Loop through all markers in the reference
            for(int i=0; i < nMarkers; i++){
                // Check if we have this marker in the model, else ignore it
                index = modelMarkerSet.getIndex(ikSolver.getMarkerNameForIndex(i));
                if(index >= 0){
                    // Check if the marker is used in the current frame
                    indexU = mNotUsedInStep.findIndex(ikSolver.getMarkerNameForIndex(i));
                    if (indexU == -1) {
                        Marker &marker = modelMarkerSet[index];
                        std::string bName = marker.getParentFrameName();
                        const OpenSim::PhysicalFrame & pFrame = _model->getComponent<PhysicalFrame>(bName);
                        const SimTK::MobilizedBodyIndex moBodyIndex = pFrame.getMobilizedBodyIndex();
                        // cout << "mobilized body index of " << bName << " is " << moBodyIndex << endl;
                        Matrix Jm(3, nu); Jm = 0.0;
                        SimTK::Vec3 mLoc = marker.get_location();
                        _model->getMatterSubsystem().calcStationJacobian(s, moBodyIndex, mLoc, Jm);
                        measModelJ(il*3, 0, 3, nu) = Jm;
                        il = il + 1;
                    }
                }
            }
            //std::cout << "Measured modelJ" << measModelJ << std::endl;
            Matrix P_predr(nsf, nq); P_predr=0.0;
            j=0;
            for(int k=0; k<nq; ++k) {
                if (iLocked.findIndex(k) == -1){
                    P_predr(0, k, nsf, 1) = P_pred(0, j*get_order_smoother(), nsf, 1);
                    j = j+1;
                }
            }
            Matrix NinvPT(nu, nsf); NinvPT=0.0;
            bool transpose = 0;
            for (int j=0; j<nsf; ++j) {
                _model->getMatterSubsystem().multiplyByNInv(s, transpose, ~P_predr[j], NinvPT(j));
            }
            Matrix PHT(nsf, 3*nMarkersUsed); PHT=0.0;
            PHT = ~NinvPT * ~measModelJ;

            Matrix S(3*nMarkersUsed, 3*nMarkersUsed); S=0.0;
            Matrix PHTr(nq, 3*nMarkersUsed); PHTr=0.0;
            j=0;
            for(int k=0; k<nq; ++k) {
                if (iLocked.findIndex(k) == -1){
                    PHTr(k, 0, 1, 3*nMarkersUsed) = PHT(j*get_order_smoother(), 0, 1, 3*nMarkersUsed);
                    j = j+1;
                }
            }
            Matrix NinvPHTr(nu, 3*nMarkersUsed); NinvPHTr=0.0;
            for (int j=0; j<3*nMarkersUsed; ++j) {
                _model->getMatterSubsystem().multiplyByNInv(s, transpose, PHTr(j), NinvPHTr(j));
            }
            S = measModelJ * NinvPHTr; // +R!

            SimTK::Array_<double> weights;
            markersReference.getWeights(s, weights);
            index = 0;
            il = 0;
            // Loop through all markers in the reference
            // cout << "Marker names size " << nMarkers << endl;
            for(int j=0; j < nMarkers; j++){
                // Check if we have this marker in the model, else ignore it
                index = modelMarkerSet.getIndex(ikSolver.getMarkerNameForIndex(j));
                if(index >= 0){
                    // Check if the marker is used in the current frame
                    index = mNotUsedInStep.findIndex(ikSolver.getMarkerNameForIndex(j));
                    if (index == -1) {
                        S(3*il,3*il) = S(3*il,3*il) +  pow(get_sd_meas() * 1/weights[j],2); // pow(get_sd_meas() ,2);
                        S(3*il+1,3*il+1) = S(3*il+1,3*il+1) + pow(get_sd_meas() * 1/weights[j],2); // pow(get_sd_meas() ,2);
                        S(3*il+2,3*il+2) = S(3*il+2,3*il+2) + pow(get_sd_meas() * 1/weights[j],2); // pow(get_sd_meas() ,2);
                        il = il+1;
                    }
                }
            }

            // Solve the pseudoinverse problem of ~K = pinv(~S)*~PHT;
            // ( if S has full rank : Matrix K = PHT * S.invert();
            Matrix transposeS = ~S;
            SimTK::FactorQTZ pinvS(transposeS);
            double invCond = pinvS.getRCondEstimate();

            Matrix transposeK(3*nMarkersUsed, nsf); transposeK = 0.0;
            Matrix transposeKm(3*nMarkersUsed, nsf); transposeKm = 0.0;
            Matrix transposePHT = ~PHT;
            pinvS.solve<Real>(transposePHT, transposeKm);
            Matrix K = ~transposeKm;

            x = x_pred + K*innovation;

            Matrix P_predr2(nq, nsf); P_predr2=0.0;
            j=0;
            for(int k=0; k<nq; ++k) {
                if (iLocked.findIndex(k) == -1){
                    P_predr2(k, 0, 1, nsf) = P_pred(j*get_order_smoother(), 0, 1, nsf);
                    j = j+1;
                }
            }

            Matrix NinvP(nu, nsf); NinvP=0.0;
            for (int j=0; j<nsf; ++j) {
                _model->getMatterSubsystem().multiplyByNInv(s, transpose, P_predr2(j), NinvP(j));
            }

            P = P_pred - K*measModelJ*NinvP;

            // Check whether inequality constraints are active
            lowerClampbounds;
            Array<int> iIneqAct;
            Array<double> coordBound;
            for (int clampedNr = 0; clampedNr < nClamped; clampedNr++) {
                int DOFnr = iClamped.get(clampedNr);
                // std::cout << x(DOFnr*get_order_smoother()) << "  " << lowerClampbounds.get(clampedNr) << "  " << upperClampbounds.get(clampedNr) << std::endl;
                if (x(DOFnr*get_order_smoother()) < lowerClampbounds.get(clampedNr) || x(DOFnr*get_order_smoother()) > upperClampbounds.get(clampedNr)) {
                    iIneqAct.append(DOFnr);
                    if (x(DOFnr*get_order_smoother()) < lowerClampbounds.get(clampedNr)) coordBound.append(lowerClampbounds.get(clampedNr));
                    else coordBound.append(upperClampbounds.get(clampedNr));
                }
            }

            int noIneqAct = iIneqAct.getSize(); // number of constraints
            if (noIneqAct > 0) {
                Matrix D(noIneqAct,nsf); D=0.0;
                Vector d(nsf); d=0.0;
                for (int i=0; i<noIneqAct; i++) {
                    D(i, iIneqAct.get(i)*get_order_smoother()) = 1.0;
                    d(iIneqAct.get(i)*get_order_smoother()) = coordBound[i];
                }

                Matrix DPDt = D * P * ~D;

                SimTK::FactorSVD pinvDPDt(DPDt);
                Matrix invDPDt(noIneqAct,noIneqAct); invDPDt = 0.0;
                pinvDPDt.inverse<Real>(invDPDt);
                x = x - P*~D*invDPDt*D*(x-d);
            }

            x_kf_array[n] = x;
            P_kf_array[n] = P;

            Vector qhat(nq); qhat = 0;
            j=0;
            for(int k=0; k<nq; ++k) {
                if (iLocked.findIndex(k) == -1){
                    qhat(k) = x(j*get_order_smoother());
                    j = j+1;
                }
                else { // Update
                    qhat(k) = qDefaults(k);
                }
            }


            if (get_use_visualizer()) {
                s.updQ() = qhat;
                _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
                _model->getVisualizer().show(s);
            }
        }

        //////////////
        // SMOOTHER //
        //////////////
        // RAUCH-TUNG-STRIEBEL
        Matrix invF = F.invert();
        Matrix invFQ = invF*Q;
        Vector x_rts(nsf); x_rts = x_kf_array[Nframes-1];
        SimTK::Array_<Vector> x_rts_array(Nframes,Vector(nsf));
        x_rts_array[Nframes-1] = x_rts;
        // Loop over time frames
        for (int n = 1; n < Nframes; n++) {
            int index = Nframes-1-n;

            // Probably not most robust solution!
            Matrix invPpred = P_pred_array[index+1].invert();

            Matrix Ktilde(nsf,nsf); Ktilde = 0;
            for(int i=0; i<nqf; ++i){
                for(int j=0; j<nqf; ++j)    Ktilde(i*get_order_smoother(), j*get_order_smoother(), get_order_smoother(), get_order_smoother()) = invFQ*invPpred(i*get_order_smoother(), j*get_order_smoother(), get_order_smoother(), get_order_smoother());
            }

            Matrix Ftilde = -Ktilde;
            for(int i=0; i<nqf; ++i){
                Ftilde(i*get_order_smoother(), i*get_order_smoother(), get_order_smoother(), get_order_smoother()) = Ftilde(i*get_order_smoother(), i*get_order_smoother(), get_order_smoother(), get_order_smoother()) + invF;
            }

            x_rts = Ftilde * x_rts + Ktilde * x_pred_array[index+1];
            x_rts_array[index] = x_rts;
        }

        // Report results
        Storage *modelMarkerLocations = get_report_marker_locations() ?
            new Storage(Nframes, "ModelMarkerLocations") : nullptr;
        Storage *modelMarkerErrors = get_report_errors() ?
            new Storage(Nframes, "ModelMarkerErrors") : nullptr;
        // Loop over time frames
        for (int n = 0; n < Nframes; n++) {
            s.updTime() = start_time + n*dt;
            Vector q_ks(nq); q_ks = 0;
            Vector x_ks = x_rts_array[n];
            j=0;
            for(int k=0; k<nq; ++k) {
                if (iLocked.findIndex(k) == -1){
                    q_ks(k) = x_ks(j*get_order_smoother());
                    j = j+1;
                }
                else { // Update
                    q_ks(k) = qDefaults(k);
                }
            }
            s.updQ() = q_ks;

            if (get_use_visualizer()) {
                _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
                _model->getVisualizer().show(s);
            }

            if(get_report_errors()){
                Array<double> markerErrors(0.0, 3);
                double totalSquaredMarkerError = 0.0;
                double maxSquaredMarkerError = 0.0;
                int worst = -1;

                ikSolver.setState(s);
                ikSolver.computeCurrentMarkerLocations(markerLocations);
                double nextTime = s.getTime();
                markersReference.getValuesAtTime(nextTime, measMarkerLocations);
                // get markers defined by the model
                const MarkerSet& modelMarkerSet = _model->getMarkerSet();

                int il = 0;
                int index = 0;
                Vec3 diff(0);
                double SquaredError = 0.0;
                int nm = 0;
                bool markerNotMeasured = false;
                Array<std::string> mNotUsedInStep = mNotUsed;
                for(int i=0; i < ikSolver.getNumMarkersInUse(); i++){
                    // 1. Check if we have this marker in the model, else ignore it
                    index = modelMarkerSet.getIndex(ikSolver.getMarkerNameForIndex(i));
                    if(index >= 0){
                        // 2. Check if marker used for estimation
                        index = mNotUsed.findIndex(ikSolver.getMarkerNameForIndex(i));
                        if (index == -1) {
                            // 3. Check if marker location is measured in current frame
                            // Don't use marker for update if its position is not measured [0 0 0] of if it contains NaN.
                            markerNotMeasured = (SimTK::isNaN(measMarkerLocations[i][0]) || SimTK::isNaN(measMarkerLocations[i][1]) || SimTK::isNaN(measMarkerLocations[i][2]))
                            || (measMarkerLocations[i][0] < tol_m && measMarkerLocations[i][0] > -tol_m && measMarkerLocations[i][1] < tol_m && measMarkerLocations[i][1] > -tol_m && measMarkerLocations[i][2] < tol_m && measMarkerLocations[i][2] > -tol_m);
                            if (markerNotMeasured) {
                                cout << "Marker " << ikSolver.getMarkerNameForIndex(i) << " not visible" << endl;
                            }
                            else {
                                diff = measMarkerLocations[i] - markerLocations[il];
                                SquaredError = diff.normSqr();
                                totalSquaredMarkerError += SquaredError;
                                if(SquaredError > maxSquaredMarkerError){
                                    maxSquaredMarkerError = SquaredError;
                                    worst = i;
                                }
                                nm += 1;
                            }
                            il = il + 1; // version 2.1
                        }
                    }
                }

                double rms = nm > 0 ? sqrt(totalSquaredMarkerError / nm) : 0;
                markerErrors.set(0, totalSquaredMarkerError);
                markerErrors.set(1, rms);
                markerErrors.set(2, sqrt(maxSquaredMarkerError));
                modelMarkerErrors->append(s.getTime(), 3, &markerErrors[0]);

                cout << "Frame " << n << " (t=" << s.getTime() << "):\t"
                    << "total squared error = " << totalSquaredMarkerError
                    << ", marker error: RMS=" << rms << ", max="
                    << sqrt(maxSquaredMarkerError) << " ("
                    << ikSolver.getMarkerNameForIndex(worst) << ")" << endl;
            }

            if(get_report_marker_locations()){
                ikSolver.computeCurrentMarkerLocations(markerLocations);
                Array<double> locations(0.0, 3*nMarkers);
                for(int j=0; j<nMarkers; ++j){
                    for(int k=0; k<3; ++k)
                        locations.set(3*j+k, markerLocations[j][k]);
                }

                modelMarkerLocations->append(s.getTime(), 3*nMarkers, &locations[0]);

            }

            kinematicsReporter->step(s, n);
            analysisSet.step(s, n);

        }

        if (mWeightTooHigh.size()!=0) {
            for (int marker_nr = 0; marker_nr<mWeightTooHigh.size(); marker_nr++) {
                cout << "\nWarning!! Weight for marker " << mWeightTooHigh[marker_nr]
                    << " in TaskSet is too high == unrealistically small "
                    << "measurement error assumed. Choose weights between 0.25 and 4.\n" << endl;
            }
        }

        // Do the maneuver to change then restore working directory
        // so that output files are saved to same folder as setup file.
        if (get_output_motion_file() != "" &&
                get_output_motion_file() != "Unassigned") {
            kinematicsReporter->getPositionStorage()->print(
                    get_output_motion_file());
        }
        // Remove the analysis we added to the model, this also deletes it
        _model->removeAnalysis(kinematicsReporter);

        if (modelMarkerErrors) {
            Array<string> labels("", 4);
            labels[0] = "time";
            labels[1] = "total_squared_error";
            labels[2] = "marker_error_RMS";
            labels[3] = "marker_error_max";

            modelMarkerErrors->setColumnLabels(labels);
            modelMarkerErrors->setName("Model Marker Errors from KS");

            IO::makeDir(getResultsDir());
            string errorFileName = trialName + "_ks_marker_errors";
            Storage::printResult(modelMarkerErrors, errorFileName,
                                 getResultsDir(), -1, ".sto");

            delete modelMarkerErrors;
        }

        if(modelMarkerLocations){
            Array<string> labels("", 3*nMarkers+1);
            labels[0] = "time";
            Array<string> XYZ("", 3*nMarkers);
            XYZ[0] = "_tx"; XYZ[1] = "_ty"; XYZ[2] = "_tz";

            for(int j=0; j<nMarkers; ++j){
                for(int k=0; k<3; ++k)
                    labels.set(3*j+k+1, ikSolver.getMarkerNameForIndex(j)+XYZ[k]);
            }
            modelMarkerLocations->setColumnLabels(labels);
            modelMarkerLocations->setName("Model Marker Locations from KS");

            IO::makeDir(getResultsDir());
            string markerFileName = trialName + "_ks_model_marker_locations";
            Storage::printResult(modelMarkerLocations, markerFileName,
                                 getResultsDir(), -1, ".sto");

            delete modelMarkerLocations;
        }

        IO::chDir(saveWorkingDirectory);

        success = true;

        cout << "InverseKinematicsTool completed " << Nframes << " frames in "
            << watch.getElapsedTimeFormatted() << "\n" <<endl;

    }
    catch (const std::exception& ex) {
        std::cout << "InverseKinematicsKSTool Failed: " << ex.what() << std::endl;
        // If failure happened after kinematicsReporter was added, make sure to cleanup
        if (kinematicsReporter!= nullptr)
            _model->removeAnalysis(kinematicsReporter);
        throw (Exception("InverseKinematicsKSTool Failed, "
            "please see messages window for details..."));
    }

    if (modelFromFile)
        _model.reset();

    return success;
}

// Handle conversion from older format
void InverseKinematicsKSTool::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if ( versionNumber < XMLDocument::getLatestVersion()){
        std::string newFileName = getDocumentFileName();
        if (versionNumber < 20300){
            std::string origFilename = getDocumentFileName();
            newFileName=IO::replaceSubstring(newFileName, ".xml", "_v23.xml");
            cout << "Old version setup file encountered. Converting to new file "<< newFileName << endl;
            SimTK::Xml::Document doc = SimTK::Xml::Document(origFilename);
            doc.writeToFile(newFileName);
        }
        if (versionNumber <= 20201){
            // get filename and use SimTK::Xml to parse it
            SimTK::Xml::Document doc = SimTK::Xml::Document(newFileName);
            Xml::Element root = doc.getRootElement();
            if (root.getElementTag()=="OpenSimDocument"){
                int curVersion = root.getRequiredAttributeValueAs<int>("Version");
                if (curVersion <= 20201) root.setAttributeValue("Version", "20300");
                Xml::element_iterator iter(root.element_begin("IKTool"));
                iter->setElementTag("InverseKinematicsTool");
                Xml::element_iterator toolIter(iter->element_begin("IKTrialSet"));
                // No optimizer_algorithm specification anymore
                Xml::element_iterator optIter(iter->element_begin("optimizer_algorithm"));
                if (optIter!= iter->element_end())
                    iter->eraseNode(optIter);

                Xml::element_iterator objIter(toolIter->element_begin("objects"));
                Xml::element_iterator trialIter(objIter->element_begin("IKTrial"));
                // Move children of (*trialIter) to root
                Xml::node_iterator p = trialIter->node_begin();
                for (; p!= trialIter->node_end(); ++p) {
                    iter->insertNodeAfter( iter->node_end(), p->clone());
                }
                iter->insertNodeAfter( iter->node_end(), Xml::Element("constraint_weight", "20.0"));
                iter->insertNodeAfter( iter->node_end(), Xml::Element("accuracy", "1e-4"));
                // erase node for IKTrialSet
                iter->eraseNode(toolIter);
                Xml::Document newDocument;
                Xml::Element docElement= newDocument.getRootElement();
                docElement.setAttributeValue("Version", "20300");
                docElement.setElementTag("OpenSimDocument");
                // Copy all children of root to newRoot
                docElement.insertNodeAfter(docElement.node_end(), iter->clone());
                newDocument.writeToFile(newFileName);
                setDocument(new XMLDocument(newFileName));
                aNode = updDocument()->getRootDataElement();
            }
            else {
                if (root.getElementTag()=="IKTool"){
                    root.setElementTag("InverseKinematicsTool");
                    Xml::element_iterator toolIter(root.element_begin("IKTrialSet"));
                    if (toolIter== root.element_end())
                        throw (Exception("Old IKTool setup file doesn't have required IKTrialSet element.. Aborting"));
                    // No optimizer_algorithm specification anymore
                    Xml::element_iterator optIter(root.element_begin("optimizer_algorithm"));
                    if (optIter!= root.element_end())
                    root.eraseNode(optIter);

                    Xml::element_iterator objIter(toolIter->element_begin("objects"));
                    Xml::element_iterator trialIter(objIter->element_begin("IKTrial"));
                    // Move children of (*trialIter) to root
                    Xml::node_iterator p = trialIter->node_begin();
                    for (; p!= trialIter->node_end(); ++p) {
                        root.insertNodeAfter( root.node_end(), p->clone());
                    }
                    root.insertNodeAfter( root.node_end(), Xml::Element("constraint_weight", "20.0"));
                    root.insertNodeAfter( root.node_end(), Xml::Element("accuracy", "1e-5"));
                    // erase node for IKTrialSet
                    root.eraseNode(toolIter);

                    // Create an OpenSimDocument node and move root inside it
                    Xml::Document newDocument;
                    Xml::Element docElement= newDocument.getRootElement();
                    docElement.setAttributeValue("Version", "20300");
                    docElement.setElementTag("OpenSimDocument");
                    // Copy all children of root to newRoot
                    docElement.insertNodeAfter(docElement.node_end(), doc.getRootElement().clone());
                    newDocument.writeToFile(newFileName);
                    setDocument(new XMLDocument(newFileName));
                    aNode = updDocument()->getRootDataElement();
                }
                else
                    ;   // Something wrong! bail out
            }
        }
    }
    Object::updateFromXMLNode(aNode, versionNumber);
}

void InverseKinematicsKSTool::populateReferences(MarkersReference& markersReference,
    SimTK::Array_<CoordinateReference>&coordinateReferences,
    Array<std::string>& mNotUsed, Array<std::string>& mInTaskSet,
    Array<std::string>& mWeightTooHigh) const
{
    FunctionSet *coordFunctions = NULL;
    // Load the coordinate data
    // bool haveCoordinateFile = false;
    if (get_coordinate_file() != "" && get_coordinate_file() != "Unassigned") {
        Storage coordinateValues(get_coordinate_file());
        // Convert degrees to radian (TODO: this needs to have a check that the storage is, in fact, in degrees!)
        _model->getSimbodyEngine().convertDegreesToRadians(coordinateValues);
        // haveCoordinateFile = true;
        coordFunctions = new GCVSplineSet(5, &coordinateValues);
    }

    Set<MarkerWeight> markerWeights;
    // Loop through old "IKTaskSet" and assign weights to the coordinate and marker references
    // For coordinates, create the functions for coordinate reference values
    int index = 0;
    for (int i = 0; i < get_IKTaskSet().getSize(); i++) {
        if (!get_IKTaskSet()[i].getApply()) {
            if(IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&get_IKTaskSet()[i])) {
                mNotUsed.append(markerTask->getName());
                mInTaskSet.append(markerTask->getName());
            }
        }
        if (IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask *>(&get_IKTaskSet()[i])) {
            CoordinateReference *coordRef = NULL;
            if (coordTask->getValueType() == IKCoordinateTask::FromFile) {
                if (!coordFunctions)
                    throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");

                index = coordFunctions->getIndex(coordTask->getName(), index);
                if (index >= 0) {
                    coordRef = new CoordinateReference(coordTask->getName(), coordFunctions->get(index));
                }
            }
            else if ((coordTask->getValueType() == IKCoordinateTask::ManualValue)) {
                Constant reference(Constant(coordTask->getValue()));
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }
            else { // assume it should be held at its default value
                double value = _model->getCoordinateSet().get(coordTask->getName()).getDefaultValue();
                Constant reference = Constant(value);
                coordRef = new CoordinateReference(coordTask->getName(), reference);
            }

            if (coordRef == NULL)
                throw Exception("InverseKinematicsTool: value for coordinate " + coordTask->getName() + " not found.");
            else
                coordRef->setWeight(coordTask->getWeight());

            coordinateReferences.push_back(*coordRef);
        }
        else if (IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask *>(&get_IKTaskSet()[i])) {
            if (markerTask->getApply()) {
                double w = markerTask->getWeight();
                if (w < 0.01) w = 0.01;
                // Only track markers that have a task and it is "applied"
                markerWeights.adoptAndAppend(
                    new MarkerWeight(markerTask->getName(), w));
                if (w > 4) mWeightTooHigh.append(markerTask->getName());
                mInTaskSet.append(markerTask->getName());
            }
        }
    }

    //Read in the marker data file and set the weights for associated markers.
    //Markers in the model and the marker file but not in the markerWeights are
    //ignored
    markersReference.initializeFromMarkersFile(get_marker_file(), markerWeights);
}

std::unordered_map<std::string, int> InverseKinematicsKSTool::createSystemYIndexMap(
        const Model& model) {
    std::unordered_map<std::string, int> sysYIndices;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                sysYIndices[svNames[isv]] = iy;
                s.updY()[iy] = 0;
                break;
            }
        }
        if (SimTK::isNaN(s.updY()[iy])) {
            // If we reach here, this is an unused slot for a quaternion.
            s.updY()[iy] = 0;
        }
    }
    SimTK_ASSERT2_ALWAYS(svNames.size() == (int)sysYIndices.size(),
            "Expected to find %i state indices but found %i.", svNames.size(),
            sysYIndices.size());
    return sysYIndices;
}
