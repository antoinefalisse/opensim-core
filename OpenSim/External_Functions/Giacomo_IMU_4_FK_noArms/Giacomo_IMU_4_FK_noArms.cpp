/*  This code describes the OpenSim model and the skeleton dynamics
    Author: Antoine Falisse
    Contributor: Joris Gillis, Gil Serrancoli, Chris Dembia
*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Common/LinearFunction.h>
//#include <OpenSim/Common/Constant.h>
//#include <OpenSim/Common/SimmSpline.h>
//#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>
#include "SimTKcommon/internal/recorder.h"
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

/*  The function F describes the OpenSim model and, implicitly, the skeleton
    dynamics. F takes as inputs joint positions and velocities (states x),
    joint accelerations (controls u), and returns the joint torques as well as
    several variables for use in the optimal control problems. F is templatized
    using type T. F(x,u)->(r).
*/

// Inputs/outputs of function F
/// number of vectors in inputs/outputs of function F
constexpr int n_in = 2;
constexpr int n_out = 1;
/// number of elements in input/output vectors of function F
constexpr int ndof = 23;        // # degrees of freedom (excluding locked)
constexpr int NX = ndof*2;      // # states
constexpr int NU = ndof;        // # controls
constexpr int NR = ndof + 2*8*3 + 8*9;    // # residual torques + # joint origins
//constexpr int NR = 4;    // # residual torques + # joint origins

// Helper function value
template<typename T>
T value(const Recorder& e) { return e; }
template<>
double value(const Recorder& e) { return e.getValue(); }

// OpenSim and Simbody use different indices for the states/controls when the
// kinematic chain has joints up and down the origin (e.g., lumbar joint/arms
// and legs with pelvis as origin).
// The two following functions allow getting the indices from one reference
// system to the other. These functions are inspired from
// createSystemYIndexMap() in Moco.
// getIndicesOSInSimbody() returns the indices of the OpenSim Qs in the Simbody
// reference system. Note that we only care about the order here so we divide
// by 2 because the states include both Qs and Qdots.
SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) {
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    SimTK::Array_<int> idxOSInSimbody(s.getNQ());
    s.updQ() = 0;
    for (int iy = 0; iy < s.getNQ(); ++iy) {
        s.updQ()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                s.updQ()[iy] = 0;
                idxOSInSimbody[iy] = isv/2;
                break;
            }
        }
    }
    return idxOSInSimbody;
}
// getIndicesSimbodyInOS() returns the indices of the Simbody Qs in the OpenSim
// reference system.
SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) {
    auto idxOSInSimbody = getIndicesOSInSimbody(model);
    auto s = model.getWorkingState();
    SimTK::Array_<int> idxSimbodyInOS(s.getNQ());
	for (int iy = 0; iy < s.getNQ(); ++iy) {
		for (int iyy = 0; iyy < s.getNQ(); ++iyy) {
			if (idxOSInSimbody[iyy] == iy) {
				idxSimbodyInOS[iy] = iyy;
				break;
			}
		}
	}
    return idxSimbodyInOS;
}

// This function returns the linear acceleration of the imu frame wrt the
// ground frame expressed in the ground frame.
// A_GB contains the angular and linear accelerations of the bodies. It is a
// Vector_ (whose first entry is the ground) of SpatialVecs (whose first and
// second entries contain the angular and linear accelerations, respectively).
// frameName is the name of the body segment to which the imu is attached.
// imuTranslation_B is the translation offset of the imu frame'origin from the
// parent (body segment) frame's origin, expressed in the parent frame.
//const SimTK::Vec3 getLinearAccelerationIMUInGround(Model& model,
//        const State& s, const Vector_<SpatialVec>& A_GB ,
//        const std::string& frameName, const Vec3& gravity_G,
//        const Vec3& imuTranslation_B)
//{
//    /// Rotation
//    const SimTK::Rotation R_GB = model.getBodySet().get(frameName).getMobilizedBody().getBodyTransform(s).R();
//    /// Body linear acceleration in ground
//    const SimTK::Vec3 linAcc_G = A_GB[model.getBodySet().get(frameName).getMobilizedBodyIndex()][1];
//    /// Body angular acceleration in ground
//    const SimTK::Vec3 angAcc_G = A_GB[model.getBodySet().get(frameName).getMobilizedBodyIndex()][0];
//    /// Body angular velocity in ground
//    const SimTK::Vec3 angVel_G = model.getBodySet().get(frameName).getAngularVelocityInGround(s);
//    /// Body angular velocity in body
//    const SimTK::Vec3 angVel_B = ~R_GB*angVel_G;
//    /// Body angular acceleration in body
//    const SimTK::Vec3 angAcc_B = ~R_GB*angAcc_G;
//    /// Sensor linear acceleration
//    /// See van den Bogert et al. (1995), equation (1), p949.
//    const SimTK::Vec3 linAcc_imu_B = ~R_GB * (linAcc_G - gravity_G) + SimTK::cross(angAcc_B, imuTranslation_B) + SimTK::cross(angVel_B, SimTK::cross(angVel_B, imuTranslation_B));
//    const SimTK::Vec3 linAcc_imu_G = R_GB * linAcc_imu_B;
//
//    return linAcc_imu_G;
//}

const SimTK::Vec3 getLinearAccelerationIMUInGround2(Model& model,
        const State& s, const Vector_<SpatialVec>& A_GB ,
        const std::string& frameName, const Vec3& gravity_G,
        const Vec3& imuTranslation_B)
{
    /// Rotation
    const SimTK::Rotation R_GB = model.getBodySet().get(frameName).getMobilizedBody().getBodyTransform(s).R();
    /// Body linear acceleration in ground
    const SimTK::Vec3 linAcc_G = A_GB[model.getBodySet().get(frameName).getMobilizedBodyIndex()][1];
    /// Body angular acceleration in ground
    const SimTK::Vec3 angAcc_G = A_GB[model.getBodySet().get(frameName).getMobilizedBodyIndex()][0];
    /// Body angular velocity in ground
    const SimTK::Vec3 angVel_G = model.getBodySet().get(frameName).getAngularVelocityInGround(s);
    /// Sensor linear acceleration
    /// See van den Bogert et al. (1995), equation (1), p949.
    const SimTK::Vec3 linAcc_imu_G = (linAcc_G - gravity_G) + SimTK::cross(angAcc_G, R_GB*imuTranslation_B) + SimTK::cross(angVel_G, SimTK::cross(angVel_G, R_GB*imuTranslation_B));

    return linAcc_imu_G;
}

// Function F
template<typename T>
int F_generic(const T** arg, T** res) {

    // OpenSim model: create components
    /// Model
    OpenSim::Model* model;
    /// Bodies
    OpenSim::Body* pelvis;
    OpenSim::Body* femur_r;
    OpenSim::Body* femur_l;
    OpenSim::Body* tibia_r;
    OpenSim::Body* tibia_l;
    OpenSim::Body* talus_r;
    OpenSim::Body* talus_l;
    OpenSim::Body* calcn_r;
    OpenSim::Body* calcn_l;
    OpenSim::Body* toes_r;
    OpenSim::Body* toes_l;
    OpenSim::Body* torso;
    //OpenSim::Body* humerus_r;
    //OpenSim::Body* humerus_l;
    //OpenSim::Body* ulna_r;
    //OpenSim::Body* ulna_l;
    //OpenSim::Body* radius_r;
    //OpenSim::Body* radius_l;
    //OpenSim::Body* hand_r;
    //OpenSim::Body* hand_l;
    /// Joints
    OpenSim::CustomJoint* ground_pelvis;
    OpenSim::CustomJoint* hip_r;
    OpenSim::CustomJoint* hip_l;
    OpenSim::CustomJoint* knee_r;
    OpenSim::CustomJoint* knee_l;
    OpenSim::CustomJoint* ankle_r;
    OpenSim::CustomJoint* ankle_l;
    OpenSim::CustomJoint* subtalar_r;
    OpenSim::CustomJoint* subtalar_l;
    OpenSim::PinJoint* mtp_r;
    OpenSim::PinJoint* mtp_l;
    OpenSim::CustomJoint* back;
    //OpenSim::CustomJoint* shoulder_r;
    //OpenSim::CustomJoint* shoulder_l;
    //OpenSim::CustomJoint* elbow_r;
    //OpenSim::CustomJoint* elbow_l;
    //OpenSim::CustomJoint* radioulnar_r;
    //OpenSim::CustomJoint* radioulnar_l;
    //OpenSim::WeldJoint* radius_hand_r;
    //OpenSim::WeldJoint* radius_hand_l;
    /// Contact elements
    OpenSim::HuntCrossleyForce_smooth* HC_1_r;
    OpenSim::HuntCrossleyForce_smooth* HC_2_r;
    OpenSim::HuntCrossleyForce_smooth* HC_3_r;
    OpenSim::HuntCrossleyForce_smooth* HC_4_r;
    OpenSim::HuntCrossleyForce_smooth* HC_5_r;
    OpenSim::HuntCrossleyForce_smooth* HC_6_r;
    OpenSim::HuntCrossleyForce_smooth* HC_1_l;
    OpenSim::HuntCrossleyForce_smooth* HC_2_l;
    OpenSim::HuntCrossleyForce_smooth* HC_3_l;
    OpenSim::HuntCrossleyForce_smooth* HC_4_l;
    OpenSim::HuntCrossleyForce_smooth* HC_5_l;
    OpenSim::HuntCrossleyForce_smooth* HC_6_l;

    // OpenSim model: initialize components
    /// Model
    model = new OpenSim::Model();
    /// Body specifications
    pelvis = new OpenSim::Body("pelvis", 11.751210011095651, Vec3(-0.069729482228687481, 0, 0), Inertia(0.099778065737821386, 0.08453958682650041, 0.056197957258948036, 0, 0, 0));
    femur_l = new OpenSim::Body("femur_l", 9.2810312301269491, Vec3(0, -0.17139710302915984, 0), Inertia(0.13581183005094824, 0.035601159333743713, 0.14321605976993196, 0, 0, 0));
    femur_r = new OpenSim::Body("femur_r", 9.2810312301269491, Vec3(0, -0.17139710302915984, 0), Inertia(0.13581183005094824, 0.035601159333743713, 0.14321605976993196, 0, 0, 0));
    tibia_l = new OpenSim::Body("tibia_l", 3.6993810916309018, Vec3(0, -0.20693800230674214, 0), Inertia(0.061783188539925718, 0.0062518702689210552, 0.062641288380758026, 0, 0, 0));
    tibia_r = new OpenSim::Body("tibia_r", 3.6993810916309018, Vec3(0, -0.20693800230674214, 0), Inertia(0.061783188539925718, 0.0062518702689210552, 0.062641288380758026, 0, 0, 0));
    talus_l = new OpenSim::Body("talus_l", 0.099781013934751236, Vec3(0, 0, 0), Inertia(0.0011448780870697791, 0.0011448780870697791, 0.0011448780870697791, 0, 0, 0));
    talus_r = new OpenSim::Body("talus_r", 0.099781013934751236, Vec3(0, 0, 0), Inertia(0.0011448780870697791, 0.0011448780870697791, 0.0011448780870697791, 0, 0, 0));
    calcn_l = new OpenSim::Body("calcn_l", 1.2472626741843904, Vec3(0.10711632521912184, 0.03213489756573655, 0), Inertia(0.0016028293218976909, 0.0044650245395721384, 0.0046940001569860954, 0, 0, 0));
    calcn_r = new OpenSim::Body("calcn_r", 1.2472626741843904, Vec3(0.10711632521912184, 0.03213489756573655, 0), Inertia(0.0016028293218976909, 0.0044650245395721384, 0.0046940001569860954, 0, 0, 0));
    toes_l = new OpenSim::Body("toes_l", 0.21612567618267114, Vec3(0.037062248525816154, 0.0064269795131473104, 0.018745356913346323), Inertia(0.00011448780870697792, 0.00022897561741395584, 0.0011448780870697791, 0, 0, 0));
    toes_r = new OpenSim::Body("toes_r", 0.21612567618267114, Vec3(0.037062248525816154, 0.0064269795131473104, -0.018745356913346323), Inertia(0.00011448780870697792, 0.00022897561741395584, 0.0011448780870697791, 0, 0, 0));
    torso = new OpenSim::Body("torso", 26.767853484219973, Vec3(-0.029119398106347269, 0.31060691313437089, 0), Inertia(1.3861651249269105, 0.7102392349150769, 1.345647175191848, 0, 0, 0));
    //humerus_l = new OpenSim::Body("humerus_l", 2.028049108223819, Vec3(0, -0.18049429839653774, 0), Inertia(0.014350103502831833, 0.0049503412468751024, 0.016107528701613263, 0, 0, 0));
    //humerus_r = new OpenSim::Body("humerus_r", 2.028049108223819, Vec3(0, -0.18049429839653774, 0), Inertia(0.014350103502831833, 0.0049503412468751024, 0.016107528701613263, 0, 0, 0));
    //ulna_l = new OpenSim::Body("ulna_l", 0.6061696596536138, Vec3(0, -0.1408058196310753, 0), Inertia(0.0040338514094605477, 0.00084163408880709608, 0.0043756801413223296, 0, 0, 0));
    //ulna_r = new OpenSim::Body("ulna_r", 0.6061696596536138, Vec3(0, -0.1408058196310753, 0), Inertia(0.0040338514094605477, 0.00084163408880709608, 0.0043756801413223296, 0, 0, 0));
    //radius_l = new OpenSim::Body("radius_l", 0.6061696596536138, Vec3(0, -0.1408058196310753, 0), Inertia(0.0040338514094605477, 0.00084163408880709608, 0.0043756801413223296, 0, 0, 0));
    //radius_r = new OpenSim::Body("radius_r", 0.6061696596536138, Vec3(0, -0.1408058196310753, 0), Inertia(0.0040338514094605477, 0.00084163408880709608, 0.0043756801413223296, 0, 0, 0));
    //hand_l = new OpenSim::Body("hand_l", 0.45649813875148687, Vec3(0, -0.079553389651757511, 0), Inertia(0.001214785772194061, 0.00074494149931631317, 0.0018249023932063251, 0, 0, 0));
    //hand_r = new OpenSim::Body("hand_r", 0.45649813875148687, Vec3(0, -0.079553389651757511, 0), Inertia(0.001214785772194061, 0.00074494149931631317, 0.0018249023932063251, 0, 0, 0));
    /// Joints
    /// Ground-Pelvis transform
    SpatialTransform st_ground_pelvis;
    st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
    st_ground_pelvis[0].setFunction(new LinearFunction());
    st_ground_pelvis[0].setAxis(Vec3(0, 0, 1));
    st_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("pelvis_list", 1, 1));
    st_ground_pelvis[1].setFunction(new LinearFunction());
    st_ground_pelvis[1].setAxis(Vec3(1, 0, 0));
    st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_rotation", 1, 1));
    st_ground_pelvis[2].setFunction(new LinearFunction());
    st_ground_pelvis[2].setAxis(Vec3(0, 1, 0));
    st_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tx", 1, 1));
    st_ground_pelvis[3].setFunction(new LinearFunction());
    st_ground_pelvis[3].setAxis(Vec3(1, 0, 0));
    st_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("pelvis_ty", 1, 1));
    st_ground_pelvis[4].setFunction(new LinearFunction());
    st_ground_pelvis[4].setAxis(Vec3(0, 1, 0));
    st_ground_pelvis[5].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tz", 1, 1));
    st_ground_pelvis[5].setFunction(new LinearFunction());
    st_ground_pelvis[5].setAxis(Vec3(0, 0, 1));
    /// Hip_l transform
    SpatialTransform st_hip_l;
    st_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
    st_hip_l[0].setFunction(new LinearFunction());
    st_hip_l[0].setAxis(Vec3(0, 0, 1));
    st_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
    st_hip_l[1].setFunction(new LinearFunction());
    st_hip_l[1].setAxis(Vec3(-1, 0, 0));
    st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
    st_hip_l[2].setFunction(new LinearFunction());
    st_hip_l[2].setAxis(Vec3(0, -1, 0));
    /// Hip_r transform
    SpatialTransform st_hip_r;
    st_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
    st_hip_r[0].setFunction(new LinearFunction());
    st_hip_r[0].setAxis(Vec3(0, 0, 1));
    st_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
    st_hip_r[1].setFunction(new LinearFunction());
    st_hip_r[1].setAxis(Vec3(1, 0, 0));
    st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
    st_hip_r[2].setFunction(new LinearFunction());
    st_hip_r[2].setAxis(Vec3(0, 1, 0));
    /// Knee_l transform
    SpatialTransform st_knee_l;
    st_knee_l[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
    st_knee_l[2].setFunction(new LinearFunction());
    st_knee_l[2].setAxis(Vec3(0, 0, 1));
    /// Knee_r transform
    SpatialTransform st_knee_r;
    st_knee_r[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
    st_knee_r[2].setFunction(new LinearFunction());
    st_knee_r[2].setAxis(Vec3(0, 0, 1));
    /// Ankle_l transform
    SpatialTransform st_ankle_l;
    st_ankle_l[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_l", 1, 1));
    st_ankle_l[0].setFunction(new LinearFunction());
    st_ankle_l[0].setAxis(Vec3(0.10501355, 0.17402245, 0.97912632));
    /// Ankle_r transform
    SpatialTransform st_ankle_r;
    st_ankle_r[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_r", 1, 1));
    st_ankle_r[0].setFunction(new LinearFunction());
    st_ankle_r[0].setAxis(Vec3(-0.10501355, -0.17402245, 0.97912632));
    /// Subtalar_l transform
    SpatialTransform st_subtalar_l;
    st_subtalar_l[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_l", 1, 1));
    st_subtalar_l[0].setFunction(new LinearFunction());
    st_subtalar_l[0].setAxis(Vec3(-0.78717961, -0.60474746, -0.12094949));
    /// Subtalar_r transform
    SpatialTransform st_subtalar_r;
    st_subtalar_r[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_r", 1, 1));
    st_subtalar_r[0].setFunction(new LinearFunction());
    st_subtalar_r[0].setAxis(Vec3(0.78717961, 0.60474746, -0.12094949));
    /// Back transform
    SpatialTransform st_back;
    st_back[0].setCoordinateNames(OpenSim::Array<std::string>("lumbar_extension", 1, 1));
    st_back[0].setFunction(new LinearFunction());
    st_back[0].setAxis(Vec3(0, 0, 1));
    st_back[1].setCoordinateNames(OpenSim::Array<std::string>("lumbar_bending", 1, 1));
    st_back[1].setFunction(new LinearFunction());
    st_back[1].setAxis(Vec3(1, 0, 0));
    st_back[2].setCoordinateNames(OpenSim::Array<std::string>("lumbar_rotation", 1, 1));
    st_back[2].setFunction(new LinearFunction());
    st_back[2].setAxis(Vec3(0, 1, 0));
    ///// Shoulder_l transform
    //SpatialTransform st_sho_l;
    //st_sho_l[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_l", 1, 1));
    //st_sho_l[0].setFunction(new LinearFunction());
    //st_sho_l[0].setAxis(Vec3(0, 0, 1));
    //st_sho_l[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_l", 1, 1));
    //st_sho_l[1].setFunction(new LinearFunction());
    //st_sho_l[1].setAxis(Vec3(-1, 0, 0));
    //st_sho_l[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_l", 1, 1));
    //st_sho_l[2].setFunction(new LinearFunction());
    //st_sho_l[2].setAxis(Vec3(0, -1, 0));
    ///// Shoulder_r transform
    //SpatialTransform st_sho_r;
    //st_sho_r[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_r", 1, 1));
    //st_sho_r[0].setFunction(new LinearFunction());
    //st_sho_r[0].setAxis(Vec3(0, 0, 1));
    //st_sho_r[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_r", 1, 1));
    //st_sho_r[1].setFunction(new LinearFunction());
    //st_sho_r[1].setAxis(Vec3(1, 0, 0));
    //st_sho_r[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_r", 1, 1));
    //st_sho_r[2].setFunction(new LinearFunction());
    //st_sho_r[2].setAxis(Vec3(0, 1, 0));
    ///// Elbow_l transform
    //SpatialTransform st_elb_l;
    //st_elb_l[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_l", 1, 1));
    //st_elb_l[0].setFunction(new LinearFunction());
    //st_elb_l[0].setAxis(Vec3(-0.22604696, -0.022269, 0.97386183));
    ///// Elbow_r transform
    //SpatialTransform st_elb_r;
    //st_elb_r[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_r", 1, 1));
    //st_elb_r[0].setFunction(new LinearFunction());
    //st_elb_r[0].setAxis(Vec3(0.22604696, 0.022269, 0.97386183));
    ///// Radioulnar_l transform
    //SpatialTransform st_radioulnar_l;
    //st_radioulnar_l[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_l", 1, 1));
    //st_radioulnar_l[0].setFunction(new LinearFunction());
    //st_radioulnar_l[0].setAxis(Vec3(-0.05639803, -0.99840646, 0.001952));
    ///// Radioulnar_r transform
    //SpatialTransform st_radioulnar_r;
    //st_radioulnar_r[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_r", 1, 1));
    //st_radioulnar_r[0].setFunction(new LinearFunction());
    //st_radioulnar_r[0].setAxis(Vec3(0.05639803, 0.99840646, 0.001952));
    /// Joint specifications
    ground_pelvis = new CustomJoint("ground_pelvis", model->getGround(), Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0), st_ground_pelvis);
    hip_l = new CustomJoint("hip_l", *pelvis, Vec3(-0.069729482228687481, -0.065192627656523949, -0.08235377321209908), Vec3(0), *femur_l, Vec3(0), Vec3(0), st_hip_l);
    hip_r = new CustomJoint("hip_r", *pelvis, Vec3(-0.069729482228687481, -0.065192627656523949, 0.08235377321209908), Vec3(0), *femur_r, Vec3(0), Vec3(0), st_hip_r);
    knee_l = new CustomJoint("knee_l", *femur_l, Vec3(-0.0045368400000000003, -0.39907365, 0), Vec3(0), *tibia_l, Vec3(0), Vec3(0), st_knee_l);
    knee_r = new CustomJoint("knee_r", *femur_r, Vec3(-0.0045368400000000003, -0.39907365, 0), Vec3(0), *tibia_r, Vec3(0), Vec3(0), st_knee_r);
    ankle_l = new CustomJoint("ankle_l", *tibia_l, Vec3(0, -0.47661136042795454, 0), Vec3(0), *talus_l, Vec3(0), Vec3(0), st_ankle_l);
    ankle_r = new CustomJoint("ankle_r", *tibia_r, Vec3(0, -0.47661136042795454, 0), Vec3(0), *talus_r, Vec3(0), Vec3(0), st_ankle_r);
    subtalar_l = new CustomJoint("subtalar_l", *talus_l, Vec3(-0.052240631809365723, -0.044935298429421615, -0.0084836129573544507), Vec3(0), *calcn_l, Vec3(0), Vec3(0), st_subtalar_l);
    subtalar_r = new CustomJoint("subtalar_r", *talus_r, Vec3(-0.052240631809365723, -0.044935298429421615, 0.0084836129573544507), Vec3(0), *calcn_r, Vec3(0), Vec3(0), st_subtalar_r);
    mtp_l = new PinJoint("mtp_l", *calcn_l, Vec3(0.19152398949178984, -0.0021423265043824368, -0.0011568563123665158), Vec3(0), *toes_l, Vec3(0), Vec3(0));
    mtp_r = new PinJoint("mtp_r", *calcn_r, Vec3(0.19152398949178984, -0.0021423265043824368, 0.0011568563123665158), Vec3(0), *toes_r, Vec3(0), Vec3(0));
    back = new CustomJoint("back", *pelvis, Vec3(-0.099317664221058408, 0.080381227745941017, 0), Vec3(0), *torso, Vec3(0), Vec3(0), st_back);
    //shoulder_l = new CustomJoint("shoulder_l", *torso, Vec3(0.0030623900341841875, 0.36059521321693366, -0.16500992260263453), Vec3(0), *humerus_l, Vec3(0), Vec3(0), st_sho_l);
    //shoulder_r = new CustomJoint("shoulder_r", *torso, Vec3(0.0030623900341841875, 0.36059521321693366, 0.16500992260263453), Vec3(0), *humerus_r, Vec3(0), Vec3(0), st_sho_r);
    //elbow_l = new CustomJoint("elbow_l", *humerus_l, Vec3(0.014421812854093517, -0.31410344120358441, 0.010527791717515771), Vec3(0), *ulna_l, Vec3(0), Vec3(0), st_elb_l);
    //elbow_r = new CustomJoint("elbow_r", *humerus_r, Vec3(0.014421812854093517, -0.31410344120358441, -0.010527791717515771), Vec3(0), *ulna_r, Vec3(0), Vec3(0), st_elb_r);
    //radioulnar_l = new CustomJoint("radioulnar_l", *ulna_l, Vec3(-0.0078589566368657427, -0.015195696294888168, -0.030472003264362887), Vec3(0), *radius_l, Vec3(0), Vec3(0),st_radioulnar_l);
    //radioulnar_r = new CustomJoint("radioulnar_r", *ulna_r, Vec3(-0.0078589566368657427, -0.015195696294888168, 0.030472003264362887), Vec3(0), *radius_r, Vec3(0), Vec3(0),st_radioulnar_r);
    //radius_hand_l = new WeldJoint("radius_hand_l", *radius_l, Vec3(-0.010277276874462305, -0.27552611746618899, -0.015900163494535866), Vec3(0), *hand_l, Vec3(0), Vec3(0));
    //radius_hand_r = new WeldJoint("radius_hand_r", *radius_r, Vec3(-0.010277276874462305, -0.27552611746618899, 0.015900163494535866), Vec3(0), *hand_r, Vec3(0), Vec3(0));
    /// Add bodies and joints to model
    model->addBody(pelvis);		    model->addJoint(ground_pelvis);
    model->addBody(femur_l);		model->addJoint(hip_l);
    model->addBody(femur_r);		model->addJoint(hip_r);
    model->addBody(tibia_l);		model->addJoint(knee_l);
    model->addBody(tibia_r);		model->addJoint(knee_r);
    model->addBody(talus_l);		model->addJoint(ankle_l);
    model->addBody(talus_r);		model->addJoint(ankle_r);
    model->addBody(calcn_l);		model->addJoint(subtalar_l);
    model->addBody(calcn_r);		model->addJoint(subtalar_r);
    model->addBody(toes_l);		    model->addJoint(mtp_l);
    model->addBody(toes_r);		    model->addJoint(mtp_r);
    model->addBody(torso);          model->addJoint(back);
    //model->addBody(humerus_l);      model->addJoint(shoulder_l);
    //model->addBody(humerus_r);      model->addJoint(shoulder_r);
    //model->addBody(ulna_l);         model->addJoint(elbow_l);
    //model->addBody(ulna_r);         model->addJoint(elbow_r);
    //model->addBody(radius_l);       model->addJoint(radioulnar_l);
    //model->addBody(radius_r);       model->addJoint(radioulnar_r);
    //model->addBody(hand_l);         model->addJoint(radius_hand_l);
    //model->addBody(hand_r);         model->addJoint(radius_hand_r);
    /// Contact elements
    /// Parameters
    osim_double_adouble radiusSphere_s1 = 0.032320000000000002;
    osim_double_adouble radiusSphere_s2 = 0.032320000000000002;
    osim_double_adouble radiusSphere_s3 = 0.023373999999999999;
    osim_double_adouble radiusSphere_s4 = 0.020507999999999998;
    osim_double_adouble radiusSphere_s5 = 0.016244000000000001;
    osim_double_adouble radiusSphere_s6 = 0.018414;
    osim_double_adouble stiffness = 1000000;
    osim_double_adouble dissipation = 2.0;
    osim_double_adouble staticFriction = 0.8;
    osim_double_adouble dynamicFriction = 0.8;
    osim_double_adouble viscousFriction = 0.5;
    osim_double_adouble transitionVelocity = 0.2;
    Vec3 normal = Vec3(0, 1, 0);
    osim_double_adouble offset = 0;
    Vec3 locSphere_1_r(-0.00049404209447999998, -0.01, -0.0058569632627999994);
    Vec3 locSphere_2_r(0.070322939999999987, -0.01, 0.023442152048999999);
    Vec3 locSphere_3_r(0.19338808499999999, -0.01, 0.024827513967);
    Vec3 locSphere_4_r(0.19338808499999999, -0.01, -0.01172049);
    Vec3 locSphere_5_r(0.062299092545999996, -0.01, -0.0040052430476999997);
    Vec3 locSphere_6_r(2.0371383668999996e-06, -0.01, 0.026129660406);
    Vec3 locSphere_1_l(locSphere_1_r[0],locSphere_1_r[1],-locSphere_1_r[2]);
    Vec3 locSphere_2_l(locSphere_2_r[0],locSphere_2_r[1],-locSphere_2_r[2]);
    Vec3 locSphere_3_l(locSphere_3_r[0],locSphere_3_r[1],-locSphere_3_r[2]);
    Vec3 locSphere_4_l(locSphere_4_r[0],locSphere_4_r[1],-locSphere_4_r[2]);
    Vec3 locSphere_5_l(locSphere_5_r[0],locSphere_5_r[1],-locSphere_5_r[2]);
    Vec3 locSphere_6_l(locSphere_6_r[0],locSphere_6_r[1],-locSphere_6_r[2]);
    /// Left foot contact shere specifications
    HC_1_l = new HuntCrossleyForce_smooth("sphere_1_l", "calcn_l", locSphere_1_l, radiusSphere_s1,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_2_l = new HuntCrossleyForce_smooth("sphere_2_l", "calcn_l", locSphere_2_l, radiusSphere_s2,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_3_l = new HuntCrossleyForce_smooth("sphere_3_l", "calcn_l", locSphere_3_l, radiusSphere_s3,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_4_l = new HuntCrossleyForce_smooth("sphere_4_l", "calcn_l", locSphere_4_l, radiusSphere_s4,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_5_l = new HuntCrossleyForce_smooth("sphere_5_l", "toes_l", locSphere_5_l, radiusSphere_s5,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_6_l = new HuntCrossleyForce_smooth("sphere_6_l", "toes_l", locSphere_6_l, radiusSphere_s6,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    /// Add left foot contact spheres to model
    model->addComponent(HC_1_l);
    HC_1_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_2_l);
    HC_2_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_3_l);
    HC_3_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_4_l);
    HC_4_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_5_l);
    HC_5_l->connectSocket_body_sphere(*toes_l);
    model->addComponent(HC_6_l);
    HC_6_l->connectSocket_body_sphere(*toes_l);
    /// Right foot contact shere specifications
    HC_1_r = new HuntCrossleyForce_smooth("sphere_1_r", "calcn_r", locSphere_1_r, radiusSphere_s1,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_2_r = new HuntCrossleyForce_smooth("sphere_2_r", "calcn_r", locSphere_2_r, radiusSphere_s2,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_3_r = new HuntCrossleyForce_smooth("sphere_3_r", "calcn_r", locSphere_3_r, radiusSphere_s3,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_4_r = new HuntCrossleyForce_smooth("sphere_4_r", "calcn_r", locSphere_4_r, radiusSphere_s4,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_5_r = new HuntCrossleyForce_smooth("sphere_5_r", "toes_r", locSphere_5_r, radiusSphere_s5,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_6_r = new HuntCrossleyForce_smooth("sphere_6_r", "toes_r", locSphere_6_r, radiusSphere_s6,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    /// Add right foot contact spheres to model
    model->addComponent(HC_1_r);
    HC_1_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_2_r);
    HC_2_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_3_r);
    HC_3_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_4_r);
    HC_4_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_5_r);
    HC_5_r->connectSocket_body_sphere(*toes_r);
    model->addComponent(HC_6_r);
    HC_6_r->connectSocket_body_sphere(*toes_r);

    // Initialize system and state
    SimTK::State* state;
    state = new State(model->initSystem());

    // Read inputs
    std::vector<T> x(arg[0], arg[0] + NX);
    std::vector<T> u(arg[1], arg[1] + NU);

    // States and controls
    T ua[NU]; /// joint accelerations (Qdotdots) - controls
    Vector QsUs(NX); /// joint positions (Qs) and velocities (Us) - states

    // Assign inputs to model variables
    /// States
    for (int i = 0; i < NX; ++i) QsUs[i] = x[i];
    /// Controls
    T ut[NU];
    for (int i = 0; i < NU; ++i) ut[i] = u[i];
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesOSInSimbody = getIndicesOSInSimbody(*model);
    for (int i = 0; i < ndof; ++i) ua[i] = ut[indicesOSInSimbody[i]];

    // Set state variables and realize
    model->setStateVariableValues(*state, QsUs);
    model->realizeVelocity(*state);

    // Compute residual forces
    /// appliedMobilityForces (# mobilities)
    Vector appliedMobilityForces(ndof);
    appliedMobilityForces.setToZero();
    /// appliedBodyForces (# bodies + ground)
    Vector_<SpatialVec> appliedBodyForces;
    int nbodies = model->getBodySet().getSize() + 1;
    appliedBodyForces.resize(nbodies);
    appliedBodyForces.setToZero();
    /// Set gravity
    Vec3 gravity(0);
    gravity[1] = -9.80665;
    /// Add weights to appliedBodyForces
    for (int i = 0; i < model->getBodySet().getSize(); ++i) {
        model->getMatterSubsystem().addInStationForce(*state,
            model->getBodySet().get(i).getMobilizedBodyIndex(),
            model->getBodySet().get(i).getMassCenter(),
            model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
    }
    /// Add contact forces to appliedBodyForces
    /// Right foot
    Array<osim_double_adouble> Force_values_1_r = HC_1_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_2_r = HC_2_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_3_r = HC_3_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_4_r = HC_4_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_5_r = HC_5_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_6_r = HC_6_r->getRecordValues(*state);
    SpatialVec GRF_1_r;
    GRF_1_r[0] = Vec3(Force_values_1_r[9], Force_values_1_r[10], Force_values_1_r[11]);
    GRF_1_r[1] = Vec3(Force_values_1_r[6], Force_values_1_r[7], Force_values_1_r[8]);
    SpatialVec GRF_2_r;
    GRF_2_r[0] = Vec3(Force_values_2_r[9], Force_values_2_r[10], Force_values_2_r[11]);
    GRF_2_r[1] = Vec3(Force_values_2_r[6], Force_values_2_r[7], Force_values_2_r[8]);
    SpatialVec GRF_3_r;
    GRF_3_r[0] = Vec3(Force_values_3_r[9], Force_values_3_r[10], Force_values_3_r[11]);
    GRF_3_r[1] = Vec3(Force_values_3_r[6], Force_values_3_r[7], Force_values_3_r[8]);
    SpatialVec GRF_4_r;
    GRF_4_r[0] = Vec3(Force_values_4_r[9], Force_values_4_r[10], Force_values_4_r[11]);
    GRF_4_r[1] = Vec3(Force_values_4_r[6], Force_values_4_r[7], Force_values_4_r[8]);
    SpatialVec GRF_5_r;
    GRF_5_r[0] = Vec3(Force_values_5_r[9], Force_values_5_r[10], Force_values_5_r[11]);
    GRF_5_r[1] = Vec3(Force_values_5_r[6], Force_values_5_r[7], Force_values_5_r[8]);
    SpatialVec GRF_6_r;
    GRF_6_r[0] = Vec3(Force_values_6_r[9], Force_values_6_r[10], Force_values_6_r[11]);
    GRF_6_r[1] = Vec3(Force_values_6_r[6], Force_values_6_r[7], Force_values_6_r[8]);
    int ncalcn_r = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
    int ntoes_r = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
    appliedBodyForces[ncalcn_r] = appliedBodyForces[ncalcn_r] + GRF_1_r + GRF_2_r + GRF_3_r + GRF_4_r;
    appliedBodyForces[ntoes_r] = appliedBodyForces[ntoes_r] + GRF_5_r + GRF_6_r;
    /// Left foot
    Array<osim_double_adouble> Force_values_1_l = HC_1_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_2_l = HC_2_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_3_l = HC_3_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_4_l = HC_4_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_5_l = HC_5_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_6_l = HC_6_l->getRecordValues(*state);
    SpatialVec GRF_1_l;
    GRF_1_l[0] = Vec3(Force_values_1_l[9], Force_values_1_l[10], Force_values_1_l[11]);
    GRF_1_l[1] = Vec3(Force_values_1_l[6], Force_values_1_l[7], Force_values_1_l[8]);
    SpatialVec GRF_2_l;
    GRF_2_l[0] = Vec3(Force_values_2_l[9], Force_values_2_l[10], Force_values_2_l[11]);
    GRF_2_l[1] = Vec3(Force_values_2_l[6], Force_values_2_l[7], Force_values_2_l[8]);
    SpatialVec GRF_3_l;
    GRF_3_l[0] = Vec3(Force_values_3_l[9], Force_values_3_l[10], Force_values_3_l[11]);
    GRF_3_l[1] = Vec3(Force_values_3_l[6], Force_values_3_l[7], Force_values_3_l[8]);
    SpatialVec GRF_4_l;
    GRF_4_l[0] = Vec3(Force_values_4_l[9], Force_values_4_l[10], Force_values_4_l[11]);
    GRF_4_l[1] = Vec3(Force_values_4_l[6], Force_values_4_l[7], Force_values_4_l[8]);
    SpatialVec GRF_5_l;
    GRF_5_l[0] = Vec3(Force_values_5_l[9], Force_values_5_l[10], Force_values_5_l[11]);
    GRF_5_l[1] = Vec3(Force_values_5_l[6], Force_values_5_l[7], Force_values_5_l[8]);
    SpatialVec GRF_6_l;
    GRF_6_l[0] = Vec3(Force_values_6_l[9], Force_values_6_l[10], Force_values_6_l[11]);
    GRF_6_l[1] = Vec3(Force_values_6_l[6], Force_values_6_l[7], Force_values_6_l[8]);
    int ncalcn_l = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
    int ntoes_l = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
    appliedBodyForces[ncalcn_l] = appliedBodyForces[ncalcn_l] + GRF_1_l + GRF_2_l + GRF_3_l + GRF_4_l;
    appliedBodyForces[ntoes_l] = appliedBodyForces[ntoes_l] + GRF_5_l + GRF_6_l;
    /// knownUdot
    Vector knownUdot(ndof);
    knownUdot.setToZero();
    for (int i = 0; i < ndof; ++i) knownUdot[i] = ua[i];
    /// Calculate residual forces
    Vector residualMobilityForces(ndof);
    residualMobilityForces.setToZero();
    model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
        appliedMobilityForces, appliedBodyForces, knownUdot,
        residualMobilityForces);

    const SimTK::Vec3 translation_pelvis_imu(-0.17831493454814973, 0.061817291986628886, -0.0039990680138157209);
    const SimTK::Vec3 translation_torso_imu(0.11158912948881572, 0.32840109856482269, -0.011892900970346884);
    const SimTK::Vec3 translation_femur_l_imu(0.058041799598382848, -0.1241014696164201, -0.10255918128954794);
    const SimTK::Vec3 translation_femur_r_imu(0.043874913441191876, -0.14956355643761021, 0.099324896496246318);
    const SimTK::Vec3 translation_tibia_l_imu(0.054506939608971572, -0.12677364542455188, -0.006659473119856818);
    const SimTK::Vec3 translation_tibia_r_imu(0.043870959504505369, -0.11730044513507276, 0.0058238104084262776);
    const SimTK::Vec3 translation_calcn_l_imu(0.15293776615302251, 0.042853173461828112, -0.044041159392253615);
    const SimTK::Vec3 translation_calcn_r_imu(0.14952620194055849, 0.03815674754123613, 0.035898882843990843);

    const SimTK::Vec3 orientation_XYZ_pelvis_imu(-1.3318579269542004, -1.1882396356672542, 0.2410527631261758);
    const SimTK::Vec3 orientation_XYZ_torso_imu(-2.5672866688671667, 1.0765026535809974, -2.112876169366428);
    const SimTK::Vec3 orientation_XYZ_femur_l_imu(2.958753028016885, 0.033743501028704409, -1.613193322664233);
    const SimTK::Vec3 orientation_XYZ_femur_r_imu(0.33819327849993686, 0.6953617170098324, 1.2178039717000142);
    const SimTK::Vec3 orientation_XYZ_tibia_l_imu(1.2676313222452209, 1.5311334336001192, 0.28036401233694658);
    const SimTK::Vec3 orientation_XYZ_tibia_r_imu(-3.0977214575709131, -0.36361683096997272, -1.5974165720549138);
    const SimTK::Vec3 orientation_XYZ_calcn_l_imu(-2.4249908747302187, 0.20622666528816341, -2.5363899633681739);
    const SimTK::Vec3 orientation_XYZ_calcn_r_imu(-1.1776059689067224, 0.72516183088458108, -2.8319498051171221);

    SimTK::Vec3 angVel_pelvis_imu_inG   = model->getBodySet().get("pelvis").getAngularVelocityInGround(*state);
    SimTK::Vec3 angVel_torso_imu_inG    = model->getBodySet().get("torso").getAngularVelocityInGround(*state);
    SimTK::Vec3 angVel_femur_l_imu_inG  = model->getBodySet().get("femur_l").getAngularVelocityInGround(*state);
    SimTK::Vec3 angVel_femur_r_imu_inG  = model->getBodySet().get("femur_r").getAngularVelocityInGround(*state);
    SimTK::Vec3 angVel_tibia_l_imu_inG  = model->getBodySet().get("tibia_l").getAngularVelocityInGround(*state);
    SimTK::Vec3 angVel_tibia_r_imu_inG  = model->getBodySet().get("tibia_r").getAngularVelocityInGround(*state);
    SimTK::Vec3 angVel_calcn_l_imu_inG  = model->getBodySet().get("calcn_l").getAngularVelocityInGround(*state);
    SimTK::Vec3 angVel_calcn_r_imu_inG  = model->getBodySet().get("calcn_r").getAngularVelocityInGround(*state);

    Vector_<SpatialVec> A_GB(nbodies);
    model->getMatterSubsystem().calcBodyAccelerationFromUDot(*state, knownUdot, A_GB);

    //SimTK::Vec3 linAcc_pelvis_imu_inG   = getLinearAccelerationIMUInGround(*model, *state, A_GB, "pelvis",  gravity, translation_pelvis_imu);
    //SimTK::Vec3 linAcc_torso_imu_inG    = getLinearAccelerationIMUInGround(*model, *state, A_GB, "torso",   gravity, translation_torso_imu);
    //SimTK::Vec3 linAcc_femur_l_imu_inG  = getLinearAccelerationIMUInGround(*model, *state, A_GB, "femur_l", gravity, translation_femur_l_imu);
    //SimTK::Vec3 linAcc_femur_r_imu_inG  = getLinearAccelerationIMUInGround(*model, *state, A_GB, "femur_r", gravity, translation_femur_r_imu);
    //SimTK::Vec3 linAcc_tibia_l_imu_inG  = getLinearAccelerationIMUInGround(*model, *state, A_GB, "tibia_l", gravity, translation_tibia_l_imu);
    //SimTK::Vec3 linAcc_tibia_r_imu_inG  = getLinearAccelerationIMUInGround(*model, *state, A_GB, "tibia_r", gravity, translation_tibia_r_imu);
    //SimTK::Vec3 linAcc_calcn_l_imu_inG  = getLinearAccelerationIMUInGround(*model, *state, A_GB, "calcn_l", gravity, translation_calcn_l_imu);
    //SimTK::Vec3 linAcc_calcn_r_imu_inG  = getLinearAccelerationIMUInGround(*model, *state, A_GB, "calcn_r", gravity, translation_calcn_r_imu);

    SimTK::Vec3 linAcc_pelvis_imu_inG2   = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "pelvis",  gravity, translation_pelvis_imu);
    SimTK::Vec3 linAcc_torso_imu_inG2    = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "torso",   gravity, translation_torso_imu);
    SimTK::Vec3 linAcc_femur_l_imu_inG2  = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "femur_l", gravity, translation_femur_l_imu);
    SimTK::Vec3 linAcc_femur_r_imu_inG2  = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "femur_r", gravity, translation_femur_r_imu);
    SimTK::Vec3 linAcc_tibia_l_imu_inG2  = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "tibia_l", gravity, translation_tibia_l_imu);
    SimTK::Vec3 linAcc_tibia_r_imu_inG2  = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "tibia_r", gravity, translation_tibia_r_imu);
    SimTK::Vec3 linAcc_calcn_l_imu_inG2  = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "calcn_l", gravity, translation_calcn_l_imu);
    SimTK::Vec3 linAcc_calcn_r_imu_inG2  = getLinearAccelerationIMUInGround2(*model, *state, A_GB, "calcn_r", gravity, translation_calcn_r_imu);

    // Extract quaternions describing imu orientations in the groud frame.


    const SimTK::Rotation orientation_rot_pelvis_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_pelvis_imu[0], SimTK::XAxis,
            orientation_XYZ_pelvis_imu[1], SimTK::YAxis,
            orientation_XYZ_pelvis_imu[2], SimTK::ZAxis);
    const SimTK::Rotation orientation_rot_torso_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_torso_imu[0], SimTK::XAxis,
            orientation_XYZ_torso_imu[1], SimTK::YAxis,
            orientation_XYZ_torso_imu[2], SimTK::ZAxis);
    const SimTK::Rotation orientation_rot_femur_l_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_femur_l_imu[0], SimTK::XAxis,
            orientation_XYZ_femur_l_imu[1], SimTK::YAxis,
            orientation_XYZ_femur_l_imu[2], SimTK::ZAxis);
    const SimTK::Rotation orientation_rot_femur_r_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_femur_r_imu[0], SimTK::XAxis,
            orientation_XYZ_femur_r_imu[1], SimTK::YAxis,
            orientation_XYZ_femur_r_imu[2], SimTK::ZAxis);
    const SimTK::Rotation orientation_rot_tibia_l_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_tibia_l_imu[0], SimTK::XAxis,
            orientation_XYZ_tibia_l_imu[1], SimTK::YAxis,
            orientation_XYZ_tibia_l_imu[2], SimTK::ZAxis);
    const SimTK::Rotation orientation_rot_tibia_r_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_tibia_r_imu[0], SimTK::XAxis,
            orientation_XYZ_tibia_r_imu[1], SimTK::YAxis,
            orientation_XYZ_tibia_r_imu[2], SimTK::ZAxis);
    const SimTK::Rotation orientation_rot_calcn_l_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_calcn_l_imu[0], SimTK::XAxis,
            orientation_XYZ_calcn_l_imu[1], SimTK::YAxis,
            orientation_XYZ_calcn_l_imu[2], SimTK::ZAxis);
    const SimTK::Rotation orientation_rot_calcn_r_imu(
            SimTK::BodyOrSpaceType::BodyRotationSequence,
            orientation_XYZ_calcn_r_imu[0], SimTK::XAxis,
            orientation_XYZ_calcn_r_imu[1], SimTK::YAxis,
            orientation_XYZ_calcn_r_imu[2], SimTK::ZAxis);

    SimTK::Rotation R_pelvis_GF = pelvis->getTransformInGround(*state).R();
    SimTK::Rotation R_torso_GF = torso->getTransformInGround(*state).R();
    SimTK::Rotation R_femur_l_GF = femur_l->getTransformInGround(*state).R();
    SimTK::Rotation R_femur_r_GF = femur_r->getTransformInGround(*state).R();
    SimTK::Rotation R_tibia_l_GF = tibia_l->getTransformInGround(*state).R();
    SimTK::Rotation R_tibia_r_GF = tibia_r->getTransformInGround(*state).R();
    SimTK::Rotation R_calcn_l_GF = calcn_l->getTransformInGround(*state).R();
    SimTK::Rotation R_calcn_r_GF = calcn_r->getTransformInGround(*state).R();

    SimTK::Rotation orientation_rot_pelvis_imu_G = R_pelvis_GF * orientation_rot_pelvis_imu;
    SimTK::Rotation orientation_rot_torso_imu_G = R_torso_GF * orientation_rot_torso_imu;
    SimTK::Rotation orientation_rot_femur_l_imu_G = R_femur_l_GF * orientation_rot_femur_l_imu;
    SimTK::Rotation orientation_rot_femur_r_imu_G = R_femur_r_GF * orientation_rot_femur_r_imu;
    SimTK::Rotation orientation_rot_tibia_l_imu_G = R_tibia_l_GF * orientation_rot_tibia_l_imu;
    SimTK::Rotation orientation_rot_tibia_r_imu_G = R_tibia_r_GF * orientation_rot_tibia_r_imu;
    SimTK::Rotation orientation_rot_calcn_l_imu_G = R_calcn_l_GF * orientation_rot_calcn_l_imu;
    SimTK::Rotation orientation_rot_calcn_r_imu_G = R_calcn_r_GF * orientation_rot_calcn_r_imu;

    // Residual forces in OpenSim order
    T res_os[ndof];
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);
    for (int i = 0; i < ndof; ++i) res_os[i] =
            value<T>(residualMobilityForces[indicesSimbodyInOS[i]]);
    // Extract results
    int nc = 3;
    int nVec9 = 9;
    /// Residual forces
    /// We do want to extract the pro_sup torques (last two -> until NU)
    for (int i = 0; i < NU; ++i) res[0][i] = res_os[i];
    for (int i = 0; i < nc; ++i) res[0][i + NU + 0*nc] = value<T>(angVel_pelvis_imu_inG[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 1*nc] = value<T>(angVel_torso_imu_inG[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 2*nc] = value<T>(angVel_femur_l_imu_inG[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 3*nc] = value<T>(angVel_femur_r_imu_inG[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 4*nc] = value<T>(angVel_tibia_l_imu_inG[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 5*nc] = value<T>(angVel_tibia_r_imu_inG[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 6*nc] = value<T>(angVel_calcn_l_imu_inG[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 7*nc] = value<T>(angVel_calcn_r_imu_inG[i]);

    for (int i = 0; i < nc; ++i) res[0][i + NU + 8*nc] = value<T>(linAcc_pelvis_imu_inG2[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 9*nc] = value<T>(linAcc_torso_imu_inG2[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 10*nc] = value<T>(linAcc_femur_l_imu_inG2[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 11*nc] = value<T>(linAcc_femur_r_imu_inG2[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 12*nc] = value<T>(linAcc_tibia_l_imu_inG2[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 13*nc] = value<T>(linAcc_tibia_r_imu_inG2[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 14*nc] = value<T>(linAcc_calcn_l_imu_inG2[i]);
    for (int i = 0; i < nc; ++i) res[0][i + NU + 15*nc] = value<T>(linAcc_calcn_r_imu_inG2[i]);

    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +0*nVec9] = value<T>(orientation_rot_pelvis_imu_G[i][j]);
    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +1*nVec9] = value<T>(orientation_rot_torso_imu_G[i][j]);
    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +2*nVec9] = value<T>(orientation_rot_femur_l_imu_G[i][j]);
    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +3*nVec9] = value<T>(orientation_rot_femur_r_imu_G[i][j]);
    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +4*nVec9] = value<T>(orientation_rot_tibia_l_imu_G[i][j]);
    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +5*nVec9] = value<T>(orientation_rot_tibia_r_imu_G[i][j]);
    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +6*nVec9] = value<T>(orientation_rot_calcn_l_imu_G[i][j]);
    for (int i = 0; i < nc; ++i) for (int j = 0; j < nc; ++j) res[0][i*nc + j + NU + 16*nc +7*nVec9] = value<T>(orientation_rot_calcn_r_imu_G[i][j]);

    return 0;

}

/* In main(), the Recorder is used to save the expression graph of function F.
This expression graph is saved as a MATLAB function named foo.m. From this
function, a c-code can be generated via CasADi and then compiled as a dll. This
dll is then imported in MATLAB as an external function. With this workflow,
CasADi can use algorithmic differentiation to differentiate the function F.
*/
int main() {

    Recorder x[NX];
    Recorder u[NU];
    Recorder tau[NR];

    for (int i = 0; i < NX; ++i) x[i] <<= 0;
    for (int i = 0; i < NU; ++i) u[i] <<= 0;

//x[0] <<= -0.0128633;
//x[1] <<= -0.14132;
//x[2] <<= 0.123489;
//x[3] <<= -0.293362;
//x[4] <<= -3.22788;
//x[5] <<= -0.320802;
//x[6] <<= 1.66542;
//x[7] <<= -1.38764;
//x[8] <<= 0.973503;
//x[9] <<= -0.0585952;
//x[10] <<= 0.220761;
//x[11] <<= 0.0929991;
//x[12] <<= 0.385783;
//x[13] <<= -0.433525;
//x[14] <<= -0.161761;
//x[15] <<= 0.421873;
//x[16] <<= -0.262731;
//x[17] <<= 1.30695;
//x[18] <<= -0.258876;
//x[19] <<= -0.854533;
//x[20] <<= 0.152544;
//x[21] <<= -0.900744;
//x[22] <<= 6.1432;
//x[23] <<= 0.831413;
//x[24] <<= 0.0652005;
//x[25] <<= -1.12437;
//x[26] <<= -0.140227;
//x[27] <<= -1.30672;
//x[28] <<= 0.0237354;
//x[29] <<= -0.349463;
//x[30] <<= 0.322588;
//x[31] <<= -0.815109;
//x[32] <<= 0.293959;
//x[33] <<= -2.83055;
//x[34] <<= 0.0532519;
//x[35] <<= 0.0591196;
//x[36] <<= 0;
//x[37] <<= 0;
//x[38] <<= 0;
//x[39] <<= 0;
//x[40] <<= -0.203754;
//x[41] <<= 0.0910814;
//x[42] <<= 0.105103;
//x[43] <<= -0.543406;
//x[44] <<= 0.14782;
//x[45] <<= 0.37648;
//
//u[0] <<= 1.42329;
//u[1] <<= -2.53815;
//u[2] <<= -0.59382;
//u[3] <<= -3.57874;
//u[4] <<= 0.244833;
//u[5] <<= 0.435324;
//u[6] <<= -0.545029;
//u[7] <<= 3.11474;
//u[8] <<= -1.09726;
//u[9] <<= 2.14638;
//u[10] <<= -3.67276;
//u[11] <<= 2.23905;
//u[12] <<= -1.33075;
//u[13] <<= -6.54512;
//u[14] <<= -11.752;
//u[15] <<= -5.55204;
//u[16] <<= -4.29331;
//u[17] <<= -0.088837;
//u[18] <<= 0;
//u[19] <<= 0;
//u[20] <<= 3.06574;
//u[21] <<= -3.02436;
//u[22] <<= 0.119186;

    const Recorder* Recorder_arg[n_in] = { x,u };
    Recorder* Recorder_res[n_out] = { tau };

    F_generic<Recorder>(Recorder_arg, Recorder_res);

    double res[NR];
    for (int i = 0; i < NR; ++i) {
        Recorder_res[0][i] >>= res[i];
        //std::cout << res[i] << std::endl;
    }

    Recorder::stop_recording();

    return 0;

}
