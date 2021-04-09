#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>
#include "SimTKcommon/internal/recorder.h"

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

constexpr int n_in = 3;
constexpr int n_out = 1;
constexpr int nCoordinates = 35;
constexpr int NX = nCoordinates * 2;
constexpr int NU = nCoordinates;
constexpr int NP = 12 * 3;
constexpr int NR = nCoordinates + 3 * 29 + NP;

template<typename T>
T value(const Recorder& e) { return e; };
template<>
double value(const Recorder& e) { return e.getValue(); };

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
				idxOSInSimbody[iy] = isv / 2;
				break;
			}
		}
	}
	return idxOSInSimbody;
}

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

template<typename T>
int F_generic(const T** arg, T** res) {
	OpenSim::Model* model;
	model = new OpenSim::Model();;

	// Definition of bodies
	OpenSim::Body* pelvis;
	pelvis = new OpenSim::Body("pelvis", 12.53462401183536201188, Vec3(-0.07781555017967077548, 0.00000000000000000000, 0.00000000000000000000), Inertia(0.10941320781325256095, 0.09270321401297954012, 0.06162475420610236121, 0., 0., 0.));
	model->addBody(pelvis);

	OpenSim::Body* femur_r;
	femur_r = new OpenSim::Body("femur_r", 9.89976664546874829398, Vec3(0.00000000000000000000, -0.20242095410419783108, 0.00000000000000000000), Inertia(0.20205532747833265805, 0.05296595963024254561, 0.21307103988006403927, 0., 0., 0.));
	model->addBody(femur_r);

	OpenSim::Body* tibia_r;
	tibia_r = new OpenSim::Body("tibia_r", 3.94600649773962874889, Vec3(0.00000000000000000000, -0.22009009022360648267, 0.00000000000000000000), Inertia(0.07454517074013587707, 0.00754326132489470218, 0.07558052033374887402, 0., 0., 0.));
	model->addBody(tibia_r);

	OpenSim::Body* talus_r;
	talus_r = new OpenSim::Body("talus_r", 0.10643308153040131891, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Inertia(0.00102354949748176402, 0.00102354949748176402, 0.00102354949748176402, 0., 0., 0.));
	model->addBody(talus_r);

	OpenSim::Body* calcn_r;
	calcn_r = new OpenSim::Body("calcn_r", 1.33041351913001637541, Vec3(0.09806546828421421058, 0.02941964048526425970, 0.00000000000000000000), Inertia(0.00143296929647446984, 0.00399184304017887945, 0.00419655293967523269, 0., 0., 0.));
	model->addBody(calcn_r);

	OpenSim::Body* toes_r;
	toes_r = new OpenSim::Body("toes_r", 0.23053405459484924145, Vec3(0.03393065202633811783, 0.00588392809705285281, -0.01716145694973748859), Inertia(0.00010235494974817643, 0.00020470989949635286, 0.00102354949748176402, 0., 0., 0.));
	model->addBody(toes_r);

	OpenSim::Body* femur_l;
	femur_l = new OpenSim::Body("femur_l", 9.89976664546874829398, Vec3(0.00000000000000000000, -0.20242095410419783108, 0.00000000000000000000), Inertia(0.20205532747833265805, 0.05296595963024254561, 0.21307103988006403927, 0., 0., 0.));
	model->addBody(femur_l);

	OpenSim::Body* tibia_l;
	tibia_l = new OpenSim::Body("tibia_l", 3.94600649773962874889, Vec3(0.00000000000000000000, -0.22009009022360648267, 0.00000000000000000000), Inertia(0.07454517074013587707, 0.00754326132489470218, 0.07558052033374887402, 0., 0., 0.));
	model->addBody(tibia_l);

	OpenSim::Body* talus_l;
	talus_l = new OpenSim::Body("talus_l", 0.10643308153040131891, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Inertia(0.00102354949748176402, 0.00102354949748176402, 0.00102354949748176402, 0., 0., 0.));
	model->addBody(talus_l);

	OpenSim::Body* calcn_l;
	calcn_l = new OpenSim::Body("calcn_l", 1.33041351913001637541, Vec3(0.09806546828421421058, 0.02941964048526425970, 0.00000000000000000000), Inertia(0.00143296929647446984, 0.00399184304017887945, 0.00419655293967523269, 0., 0., 0.));
	model->addBody(calcn_l);

	OpenSim::Body* toes_l;
	toes_l = new OpenSim::Body("toes_l", 0.23053405459484924145, Vec3(0.03393065202633811783, 0.00588392809705285281, 0.01716145694973748859), Inertia(0.00010235494974817643, 0.00020470989949635286, 0.00102354949748176402, 0., 0., 0.));
	model->addBody(toes_l);

	OpenSim::Body* torso;
	torso = new OpenSim::Body("torso", 28.55237704983463942199, Vec3(-0.03029129157016192953, 0.32310711008172726677, 0.00000000000000000000), Inertia(1.56935578716576729086, 0.80410193096218196551, 1.52348312902616456199, 0., 0., 0.));
	model->addBody(torso);

	OpenSim::Body* humerus_r;
	humerus_r = new OpenSim::Body("humerus_r", 2.16325238210540682360, Vec3(0.00000000000000000000, -0.17217736840018482702, 0.00000000000000000000), Inertia(0.01392864639848861129, 0.00480495159954558507, 0.01563445668485968429, 0., 0., 0.));
	model->addBody(humerus_r);

	OpenSim::Body* ulna_r;
	ulna_r = new OpenSim::Body("ulna_r", 0.64658097029718808457, Vec3(0.00000000000000000000, -0.13420723621273464299, 0.00000000000000000000), Inertia(0.00390894244458061483, 0.00081557273151614451, 0.00424018638569801428, 0., 0., 0.));
	model->addBody(ulna_r);

	OpenSim::Body* radius_r;
	radius_r = new OpenSim::Body("radius_r", 0.64658097029718808457, Vec3(0.00000000000000000000, -0.13420723621273464299, 0.00000000000000000000), Inertia(0.00390894244458061483, 0.00081557273151614451, 0.00424018638569801428, 0., 0., 0.));
	model->addBody(radius_r);

	OpenSim::Body* hand_r;
	hand_r = new OpenSim::Body("hand_r", 0.48693134800158605069, Vec3(0.00000000000000000000, -0.07582527898698333824, 0.00000000000000000000), Inertia(0.00117716970309449998, 0.00072187424618014729, 0.00176839394859487674, 0., 0., 0.));
	model->addBody(hand_r);

	OpenSim::Body* humerus_l;
	humerus_l = new OpenSim::Body("humerus_l", 2.16325238210540682360, Vec3(0.00000000000000000000, -0.17217736840018482702, 0.00000000000000000000), Inertia(0.01392864639848861129, 0.00480495159954558507, 0.01563445668485968429, 0., 0., 0.));
	model->addBody(humerus_l);

	OpenSim::Body* ulna_l;
	ulna_l = new OpenSim::Body("ulna_l", 0.64658097029718808457, Vec3(0.00000000000000000000, -0.13420723621273464299, 0.00000000000000000000), Inertia(0.00390894244458061483, 0.00081557273151614451, 0.00424018638569801428, 0., 0., 0.));
	model->addBody(ulna_l);

	OpenSim::Body* radius_l;
	radius_l = new OpenSim::Body("radius_l", 0.64658097029718808457, Vec3(0.00000000000000000000, -0.13420723621273464299, 0.00000000000000000000), Inertia(0.00390894244458061483, 0.00081557273151614451, 0.00424018638569801428, 0., 0., 0.));
	model->addBody(radius_l);

	OpenSim::Body* hand_l;
	hand_l = new OpenSim::Body("hand_l", 0.48693134800158605069, Vec3(0.00000000000000000000, -0.07582527898698333824, 0.00000000000000000000), Inertia(0.00117716970309449998, 0.00072187424618014729, 0.00176839394859487674, 0., 0., 0.));
	model->addBody(hand_l);

	// Definition of joints
	SpatialTransform st_ground_pelvis;
	st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
	st_ground_pelvis[0].setFunction(new LinearFunction());
	st_ground_pelvis[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("pelvis_list", 1, 1));
	st_ground_pelvis[1].setFunction(new LinearFunction());
	st_ground_pelvis[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_rotation", 1, 1));
	st_ground_pelvis[2].setFunction(new LinearFunction());
	st_ground_pelvis[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tx", 1, 1));
	st_ground_pelvis[3].setFunction(new LinearFunction());
	st_ground_pelvis[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("pelvis_ty", 1, 1));
	st_ground_pelvis[4].setFunction(new LinearFunction());
	st_ground_pelvis[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ground_pelvis[5].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tz", 1, 1));
	st_ground_pelvis[5].setFunction(new LinearFunction());
	st_ground_pelvis[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* ground_pelvis;
	ground_pelvis = new OpenSim::CustomJoint("ground_pelvis", model->getGround(), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *pelvis, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_ground_pelvis);

	SpatialTransform st_hip_r;
	st_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
	st_hip_r[0].setFunction(new LinearFunction());
	st_hip_r[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
	st_hip_r[1].setFunction(new LinearFunction());
	st_hip_r[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
	st_hip_r[2].setFunction(new LinearFunction());
	st_hip_r[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_hip_r[3].setFunction(new Constant(0));
	st_hip_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_r[4].setFunction(new Constant(0));
	st_hip_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_hip_r[5].setFunction(new Constant(0));
	st_hip_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* hip_r;
	hip_r = new OpenSim::CustomJoint("hip_r", *pelvis, Vec3(-0.07781555017967077548, -0.07275258651875868288, 0.10221907923123482731), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *femur_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_hip_r);

	SpatialTransform st_knee_r;
	st_knee_r[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	st_knee_r[0].setFunction(new LinearFunction());
	st_knee_r[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_knee_r[1].setCoordinateNames(OpenSim::Array<std::string>("knee_adduction_r", 1, 1));
	st_knee_r[1].setFunction(new LinearFunction());
	st_knee_r[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_r[2].setFunction(new Constant(0));
	st_knee_r[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_r[3].setFunction(new Constant(0));
	st_knee_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_r[4].setFunction(new Constant(0));
	st_knee_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_r[5].setFunction(new Constant(0));
	st_knee_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* knee_r;
	knee_r = new OpenSim::CustomJoint("knee_r", *femur_r, Vec3(-0.00535802999999999974, -0.47130824999999998415, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *tibia_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_knee_r);

	SpatialTransform st_ankle_r;
	st_ankle_r[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_r", 1, 1));
	st_ankle_r[0].setFunction(new LinearFunction());
	st_ankle_r[0].setAxis(Vec3(-0.10501354999999999718, -0.17402244999999999520, 0.97912631999999999444));
	st_ankle_r[1].setFunction(new Constant(0));
	st_ankle_r[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ankle_r[2].setFunction(new Constant(0));
	st_ankle_r[2].setAxis(Vec3(0.97912631999999999444, -0.00000000000000000000, 0.10501354999999999718));
	st_ankle_r[3].setFunction(new Constant(0));
	st_ankle_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_ankle_r[4].setFunction(new Constant(0));
	st_ankle_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ankle_r[5].setFunction(new Constant(0));
	st_ankle_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* ankle_r;
	ankle_r = new OpenSim::CustomJoint("ankle_r", *tibia_r, Vec3(0.00000000000000000000, -0.50690272520702073233, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *talus_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_ankle_r);

	SpatialTransform st_subtalar_r;
	st_subtalar_r[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_r", 1, 1));
	st_subtalar_r[0].setFunction(new LinearFunction());
	st_subtalar_r[0].setAxis(Vec3(0.78717961000000002958, 0.60474746000000001445, -0.12094949000000000672));
	st_subtalar_r[1].setFunction(new Constant(0));
	st_subtalar_r[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_subtalar_r[2].setFunction(new Constant(0));
	st_subtalar_r[2].setAxis(Vec3(-0.12094949000000000672, 0.00000000000000000000, -0.78717961000000002958));
	st_subtalar_r[3].setFunction(new Constant(0));
	st_subtalar_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_subtalar_r[4].setFunction(new Constant(0));
	st_subtalar_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_subtalar_r[5].setFunction(new Constant(0));
	st_subtalar_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* subtalar_r;
	subtalar_r = new OpenSim::CustomJoint("subtalar_r", *talus_r, Vec3(-0.04782652888221126941, -0.04113846394522786137, 0.00776678508810976515), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *calcn_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_subtalar_r);

	OpenSim::PinJoint* mtp_r;
	mtp_r = new OpenSim::PinJoint("mtp_r", *calcn_r, Vec3(0.17534105729217500103, -0.00196130936568428412, 0.00105910705746951341), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *toes_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));

	SpatialTransform st_hip_l;
	st_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
	st_hip_l[0].setFunction(new LinearFunction());
	st_hip_l[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
	st_hip_l[1].setFunction(new LinearFunction());
	st_hip_l[1].setAxis(Vec3(-1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
	st_hip_l[2].setFunction(new LinearFunction());
	st_hip_l[2].setAxis(Vec3(0.00000000000000000000, -1.00000000000000000000, 0.00000000000000000000));
	st_hip_l[3].setFunction(new Constant(0));
	st_hip_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_hip_l[4].setFunction(new Constant(0));
	st_hip_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_hip_l[5].setFunction(new Constant(0));
	st_hip_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* hip_l;
	hip_l = new OpenSim::CustomJoint("hip_l", *pelvis, Vec3(-0.07781555017967077548, -0.07275258651875868288, -0.10221907923123482731), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *femur_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_hip_l);

	SpatialTransform st_knee_l;
	st_knee_l[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	st_knee_l[0].setFunction(new LinearFunction());
	st_knee_l[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_knee_l[1].setCoordinateNames(OpenSim::Array<std::string>("knee_adduction_l", 1, 1));
	st_knee_l[1].setFunction(new LinearFunction());
	st_knee_l[1].setAxis(Vec3(-1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_l[2].setFunction(new Constant(0));
	st_knee_l[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_l[3].setFunction(new Constant(0));
	st_knee_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_knee_l[4].setFunction(new Constant(0));
	st_knee_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_knee_l[5].setFunction(new Constant(0));
	st_knee_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* knee_l;
	knee_l = new OpenSim::CustomJoint("knee_l", *femur_l, Vec3(-0.00535802999999999974, -0.47130824999999998415, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *tibia_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_knee_l);

	SpatialTransform st_ankle_l;
	st_ankle_l[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_l", 1, 1));
	st_ankle_l[0].setFunction(new LinearFunction());
	st_ankle_l[0].setAxis(Vec3(0.10501354999999999718, 0.17402244999999999520, 0.97912631999999999444));
	st_ankle_l[1].setFunction(new Constant(0));
	st_ankle_l[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ankle_l[2].setFunction(new Constant(0));
	st_ankle_l[2].setAxis(Vec3(0.97912631999999999444, 0.00000000000000000000, -0.10501354999999999718));
	st_ankle_l[3].setFunction(new Constant(0));
	st_ankle_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_ankle_l[4].setFunction(new Constant(0));
	st_ankle_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_ankle_l[5].setFunction(new Constant(0));
	st_ankle_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* ankle_l;
	ankle_l = new OpenSim::CustomJoint("ankle_l", *tibia_l, Vec3(0.00000000000000000000, -0.50690272520702073233, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *talus_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_ankle_l);

	SpatialTransform st_subtalar_l;
	st_subtalar_l[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_l", 1, 1));
	st_subtalar_l[0].setFunction(new LinearFunction());
	st_subtalar_l[0].setAxis(Vec3(-0.78717961000000002958, -0.60474746000000001445, -0.12094949000000000672));
	st_subtalar_l[1].setFunction(new Constant(0));
	st_subtalar_l[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_subtalar_l[2].setFunction(new Constant(0));
	st_subtalar_l[2].setAxis(Vec3(-0.12094949000000000672, 0.00000000000000000000, 0.78717961000000002958));
	st_subtalar_l[3].setFunction(new Constant(0));
	st_subtalar_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_subtalar_l[4].setFunction(new Constant(0));
	st_subtalar_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_subtalar_l[5].setFunction(new Constant(0));
	st_subtalar_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* subtalar_l;
	subtalar_l = new OpenSim::CustomJoint("subtalar_l", *talus_l, Vec3(-0.04782652888221126941, -0.04113846394522786137, -0.00776678508810976515), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *calcn_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_subtalar_l);

	OpenSim::PinJoint* mtp_l;
	mtp_l = new OpenSim::PinJoint("mtp_l", *calcn_l, Vec3(0.17534105729217500103, -0.00196130936568428412, -0.00105910705746951341), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *toes_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));

	SpatialTransform st_back;
	st_back[0].setCoordinateNames(OpenSim::Array<std::string>("lumbar_extension", 1, 1));
	st_back[0].setFunction(new LinearFunction());
	st_back[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_back[1].setCoordinateNames(OpenSim::Array<std::string>("lumbar_bending", 1, 1));
	st_back[1].setFunction(new LinearFunction());
	st_back[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_back[2].setCoordinateNames(OpenSim::Array<std::string>("lumbar_rotation", 1, 1));
	st_back[2].setFunction(new LinearFunction());
	st_back[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_back[3].setFunction(new Constant(0));
	st_back[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_back[4].setFunction(new Constant(0));
	st_back[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_back[5].setFunction(new Constant(0));
	st_back[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* back;
	back = new OpenSim::CustomJoint("back", *pelvis, Vec3(-0.11083487840301058103, 0.08970250834007310881, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *torso, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_back);

	SpatialTransform st_acromial_r;
	st_acromial_r[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_r", 1, 1));
	st_acromial_r[0].setFunction(new LinearFunction());
	st_acromial_r[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_acromial_r[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_r", 1, 1));
	st_acromial_r[1].setFunction(new LinearFunction());
	st_acromial_r[1].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_acromial_r[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_r", 1, 1));
	st_acromial_r[2].setFunction(new LinearFunction());
	st_acromial_r[2].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_acromial_r[3].setFunction(new Constant(0));
	st_acromial_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_acromial_r[4].setFunction(new Constant(0));
	st_acromial_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_acromial_r[5].setFunction(new Constant(0));
	st_acromial_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* acromial_r;
	acromial_r = new OpenSim::CustomJoint("acromial_r", *torso, Vec3(0.00318563416346202919, 0.37510716061050519698, 0.19965066549714699518), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *humerus_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_acromial_r);

	SpatialTransform st_elbow_r;
	st_elbow_r[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_r", 1, 1));
	st_elbow_r[0].setFunction(new LinearFunction());
	st_elbow_r[0].setAxis(Vec3(0.22604695999999999123, 0.02226900000000000060, 0.97386183000000003940));
	st_elbow_r[1].setFunction(new Constant(0));
	st_elbow_r[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_elbow_r[2].setFunction(new Constant(0));
	st_elbow_r[2].setAxis(Vec3(0.97386183000000003940, 0.00000000000000000000, -0.22604695999999999123));
	st_elbow_r[3].setFunction(new Constant(0));
	st_elbow_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_elbow_r[4].setFunction(new Constant(0));
	st_elbow_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_elbow_r[5].setFunction(new Constant(0));
	st_elbow_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* elbow_r;
	elbow_r = new OpenSim::CustomJoint("elbow_r", *humerus_r, Vec3(0.01375727547538649342, -0.29962998494866993626, -0.01004268549804727614), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *ulna_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_elbow_r);

	SpatialTransform st_radioulnar_r;
	st_radioulnar_r[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_r", 1, 1));
	st_radioulnar_r[0].setFunction(new LinearFunction());
	st_radioulnar_r[0].setAxis(Vec3(0.05639803000000000177, 0.99840645999999999560, 0.00195199999999999996));
	st_radioulnar_r[1].setFunction(new Constant(0));
	st_radioulnar_r[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_radioulnar_r[2].setFunction(new Constant(0));
	st_radioulnar_r[2].setAxis(Vec3(0.00195199999999999996, 0.00000000000000000000, -0.05639803000000000177));
	st_radioulnar_r[3].setFunction(new Constant(0));
	st_radioulnar_r[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_radioulnar_r[4].setFunction(new Constant(0));
	st_radioulnar_r[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_radioulnar_r[5].setFunction(new Constant(0));
	st_radioulnar_r[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* radioulnar_r;
	radioulnar_r = new OpenSim::CustomJoint("radioulnar_r", *ulna_r, Vec3(-0.00749066233564045763, -0.01448358034780368973, 0.02904399371198305643), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *radius_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_radioulnar_r);

	OpenSim::WeldJoint* radius_hand_r;
	radius_hand_r = new OpenSim::WeldJoint("radius_hand_r", *radius_r, Vec3(-0.00979565282691082125, -0.26261413645009379358, 0.01515503409960853622), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *hand_r, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));

	SpatialTransform st_acromial_l;
	st_acromial_l[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_l", 1, 1));
	st_acromial_l[0].setFunction(new LinearFunction());
	st_acromial_l[0].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	st_acromial_l[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_l", 1, 1));
	st_acromial_l[1].setFunction(new LinearFunction());
	st_acromial_l[1].setAxis(Vec3(-1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_acromial_l[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_l", 1, 1));
	st_acromial_l[2].setFunction(new LinearFunction());
	st_acromial_l[2].setAxis(Vec3(0.00000000000000000000, -1.00000000000000000000, 0.00000000000000000000));
	st_acromial_l[3].setFunction(new Constant(0));
	st_acromial_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_acromial_l[4].setFunction(new Constant(0));
	st_acromial_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_acromial_l[5].setFunction(new Constant(0));
	st_acromial_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* acromial_l;
	acromial_l = new OpenSim::CustomJoint("acromial_l", *torso, Vec3(0.00318563416346202919, 0.37510716061050519698, -0.19965066549714699518), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *humerus_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_acromial_l);

	SpatialTransform st_elbow_l;
	st_elbow_l[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_l", 1, 1));
	st_elbow_l[0].setFunction(new LinearFunction());
	st_elbow_l[0].setAxis(Vec3(-0.22604695999999999123, -0.02226900000000000060, 0.97386183000000003940));
	st_elbow_l[1].setFunction(new Constant(0));
	st_elbow_l[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_elbow_l[2].setFunction(new Constant(0));
	st_elbow_l[2].setAxis(Vec3(0.97386183000000003940, -0.00000000000000000000, 0.22604695999999999123));
	st_elbow_l[3].setFunction(new Constant(0));
	st_elbow_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_elbow_l[4].setFunction(new Constant(0));
	st_elbow_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_elbow_l[5].setFunction(new Constant(0));
	st_elbow_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* elbow_l;
	elbow_l = new OpenSim::CustomJoint("elbow_l", *humerus_l, Vec3(0.01375727547538649342, -0.29962998494866993626, 0.01004268549804727614), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *ulna_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_elbow_l);

	SpatialTransform st_radioulnar_l;
	st_radioulnar_l[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_l", 1, 1));
	st_radioulnar_l[0].setFunction(new LinearFunction());
	st_radioulnar_l[0].setAxis(Vec3(-0.05639803000000000177, -0.99840645999999999560, 0.00195199999999999996));
	st_radioulnar_l[1].setFunction(new Constant(0));
	st_radioulnar_l[1].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_radioulnar_l[2].setFunction(new Constant(0));
	st_radioulnar_l[2].setAxis(Vec3(0.00195199999999999996, -0.00000000000000000000, 0.05639803000000000177));
	st_radioulnar_l[3].setFunction(new Constant(0));
	st_radioulnar_l[3].setAxis(Vec3(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	st_radioulnar_l[4].setFunction(new Constant(0));
	st_radioulnar_l[4].setAxis(Vec3(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000));
	st_radioulnar_l[5].setFunction(new Constant(0));
	st_radioulnar_l[5].setAxis(Vec3(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000));
	OpenSim::CustomJoint* radioulnar_l;
	radioulnar_l = new OpenSim::CustomJoint("radioulnar_l", *ulna_l, Vec3(-0.00749066233564045763, -0.01448358034780368973, -0.02904399371198305643), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *radius_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), st_radioulnar_l);

	OpenSim::WeldJoint* radius_hand_l;
	radius_hand_l = new OpenSim::WeldJoint("radius_hand_l", *radius_l, Vec3(-0.00979565282691082125, -0.26261413645009379358, -0.01515503409960853622), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), *hand_l, Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000), Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));

	model->addJoint(ground_pelvis);
	model->addJoint(hip_l);
	model->addJoint(hip_r);
	model->addJoint(knee_l);
	model->addJoint(knee_r);
	model->addJoint(ankle_l);
	model->addJoint(ankle_r);
	model->addJoint(subtalar_l);
	model->addJoint(subtalar_r);
	model->addJoint(mtp_l);
	model->addJoint(mtp_r);
	model->addJoint(back);
	model->addJoint(acromial_l);
	model->addJoint(acromial_r);
	model->addJoint(elbow_l);
	model->addJoint(elbow_r);
	model->addJoint(radioulnar_l);
	model->addJoint(radioulnar_r);
	model->addJoint(radius_hand_l);
	model->addJoint(radius_hand_r);

	// Definition of contacts
	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s1_r;
	SmoothSphereHalfSpaceForce_s1_r = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s1_r", *calcn_r, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s1_r_location(-0.00045229752354858800, -0.01000000000000000021, -0.00536207341212043085);
	SmoothSphereHalfSpaceForce_s1_r->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s1_r_location);
	double SmoothSphereHalfSpaceForce_s1_r_radius = (0.03232000000000000151);
	SmoothSphereHalfSpaceForce_s1_r->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s1_r_radius);
	SmoothSphereHalfSpaceForce_s1_r->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s1_r->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s1_r->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s1_r->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s1_r->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s1_r->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s1_r->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s1_r->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s1_r->connectSocket_sphere_frame(*calcn_r);
	SmoothSphereHalfSpaceForce_s1_r->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s1_r);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s2_r;
	SmoothSphereHalfSpaceForce_s2_r = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s2_r", *calcn_r, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s2_r_location(0.06438093426863560065, -0.01000000000000000021, 0.02146138443844967936);
	SmoothSphereHalfSpaceForce_s2_r->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s2_r_location);
	double SmoothSphereHalfSpaceForce_s2_r_radius = (0.03232000000000000151);
	SmoothSphereHalfSpaceForce_s2_r->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s2_r_radius);
	SmoothSphereHalfSpaceForce_s2_r->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s2_r->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s2_r->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s2_r->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s2_r->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s2_r->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s2_r->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s2_r->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s2_r->connectSocket_sphere_frame(*calcn_r);
	SmoothSphereHalfSpaceForce_s2_r->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s2_r);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s3_r;
	SmoothSphereHalfSpaceForce_s3_r = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s3_r", *calcn_r, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s3_r_location(0.17704756923874792607, -0.01000000000000000021, 0.02272968884354180027);
	SmoothSphereHalfSpaceForce_s3_r->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s3_r_location);
	double SmoothSphereHalfSpaceForce_s3_r_radius = (0.02337399999999999894);
	SmoothSphereHalfSpaceForce_s3_r->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s3_r_radius);
	SmoothSphereHalfSpaceForce_s3_r->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s3_r->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s3_r->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s3_r->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s3_r->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s3_r->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s3_r->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s3_r->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s3_r->connectSocket_sphere_frame(*calcn_r);
	SmoothSphereHalfSpaceForce_s3_r->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s3_r);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s4_r;
	SmoothSphereHalfSpaceForce_s4_r = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s4_r", *calcn_r, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s4_r_location(0.17704756923874792607, -0.01000000000000000021, -0.01073015571143926851);
	SmoothSphereHalfSpaceForce_s4_r->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s4_r_location);
	double SmoothSphereHalfSpaceForce_s4_r_radius = (0.02050799999999999845);
	SmoothSphereHalfSpaceForce_s4_r->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s4_r_radius);
	SmoothSphereHalfSpaceForce_s4_r->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s4_r->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s4_r->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s4_r->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s4_r->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s4_r->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s4_r->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s4_r->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s4_r->connectSocket_sphere_frame(*calcn_r);
	SmoothSphereHalfSpaceForce_s4_r->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s4_r);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s5_r;
	SmoothSphereHalfSpaceForce_s5_r = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s5_r", *toes_r, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s5_r_location(0.05703506966858428590, -0.01000000000000000021, -0.00366681611127014093);
	SmoothSphereHalfSpaceForce_s5_r->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s5_r_location);
	double SmoothSphereHalfSpaceForce_s5_r_radius = (0.01624400000000000149);
	SmoothSphereHalfSpaceForce_s5_r->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s5_r_radius);
	SmoothSphereHalfSpaceForce_s5_r->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s5_r->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s5_r->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s5_r->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s5_r->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s5_r->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s5_r->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s5_r->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s5_r->connectSocket_sphere_frame(*toes_r);
	SmoothSphereHalfSpaceForce_s5_r->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s5_r);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s6_r;
	SmoothSphereHalfSpaceForce_s6_r = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s6_r", *toes_r, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s6_r_location(0.00000186500836420526, -0.01000000000000000021, 0.02392180914308270415);
	SmoothSphereHalfSpaceForce_s6_r->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s6_r_location);
	double SmoothSphereHalfSpaceForce_s6_r_radius = (0.01841399999999999981);
	SmoothSphereHalfSpaceForce_s6_r->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s6_r_radius);
	SmoothSphereHalfSpaceForce_s6_r->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s6_r->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s6_r->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s6_r->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s6_r->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s6_r->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s6_r->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s6_r->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s6_r->connectSocket_sphere_frame(*toes_r);
	SmoothSphereHalfSpaceForce_s6_r->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s6_r);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s1_l;
	SmoothSphereHalfSpaceForce_s1_l = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s1_l", *calcn_l, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s1_l_location(-0.00045229752354858800, -0.01000000000000000021, 0.00536207341212043085);
	SmoothSphereHalfSpaceForce_s1_l->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s1_l_location);
	double SmoothSphereHalfSpaceForce_s1_l_radius = (0.03232000000000000151);
	SmoothSphereHalfSpaceForce_s1_l->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s1_l_radius);
	SmoothSphereHalfSpaceForce_s1_l->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s1_l->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s1_l->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s1_l->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s1_l->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s1_l->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s1_l->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s1_l->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s1_l->connectSocket_sphere_frame(*calcn_l);
	SmoothSphereHalfSpaceForce_s1_l->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s1_l);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s2_l;
	SmoothSphereHalfSpaceForce_s2_l = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s2_l", *calcn_l, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s2_l_location(0.06438093426863560065, -0.01000000000000000021, -0.02146138443844967936);
	SmoothSphereHalfSpaceForce_s2_l->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s2_l_location);
	double SmoothSphereHalfSpaceForce_s2_l_radius = (0.03232000000000000151);
	SmoothSphereHalfSpaceForce_s2_l->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s2_l_radius);
	SmoothSphereHalfSpaceForce_s2_l->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s2_l->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s2_l->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s2_l->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s2_l->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s2_l->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s2_l->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s2_l->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s2_l->connectSocket_sphere_frame(*calcn_l);
	SmoothSphereHalfSpaceForce_s2_l->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s2_l);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s3_l;
	SmoothSphereHalfSpaceForce_s3_l = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s3_l", *calcn_l, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s3_l_location(0.17704756923874792607, -0.01000000000000000021, -0.02272968884354180027);
	SmoothSphereHalfSpaceForce_s3_l->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s3_l_location);
	double SmoothSphereHalfSpaceForce_s3_l_radius = (0.02337399999999999894);
	SmoothSphereHalfSpaceForce_s3_l->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s3_l_radius);
	SmoothSphereHalfSpaceForce_s3_l->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s3_l->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s3_l->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s3_l->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s3_l->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s3_l->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s3_l->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s3_l->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s3_l->connectSocket_sphere_frame(*calcn_l);
	SmoothSphereHalfSpaceForce_s3_l->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s3_l);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s4_l;
	SmoothSphereHalfSpaceForce_s4_l = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s4_l", *calcn_l, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s4_l_location(0.17704756923874792607, -0.01000000000000000021, 0.01073015571143926851);
	SmoothSphereHalfSpaceForce_s4_l->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s4_l_location);
	double SmoothSphereHalfSpaceForce_s4_l_radius = (0.02050799999999999845);
	SmoothSphereHalfSpaceForce_s4_l->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s4_l_radius);
	SmoothSphereHalfSpaceForce_s4_l->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s4_l->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s4_l->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s4_l->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s4_l->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s4_l->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s4_l->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s4_l->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s4_l->connectSocket_sphere_frame(*calcn_l);
	SmoothSphereHalfSpaceForce_s4_l->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s4_l);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s5_l;
	SmoothSphereHalfSpaceForce_s5_l = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s5_l", *toes_l, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s5_l_location(0.05703506966858428590, -0.01000000000000000021, 0.00366681611127014093);
	SmoothSphereHalfSpaceForce_s5_l->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s5_l_location);
	double SmoothSphereHalfSpaceForce_s5_l_radius = (0.01624400000000000149);
	SmoothSphereHalfSpaceForce_s5_l->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s5_l_radius);
	SmoothSphereHalfSpaceForce_s5_l->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s5_l->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s5_l->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s5_l->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s5_l->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s5_l->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s5_l->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s5_l->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s5_l->connectSocket_sphere_frame(*toes_l);
	SmoothSphereHalfSpaceForce_s5_l->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s5_l);

	OpenSim::SmoothSphereHalfSpaceForce* SmoothSphereHalfSpaceForce_s6_l;
	SmoothSphereHalfSpaceForce_s6_l = new SmoothSphereHalfSpaceForce("SmoothSphereHalfSpaceForce_s6_l", *toes_l, model->getGround());
	Vec3 SmoothSphereHalfSpaceForce_s6_l_location(0.00000186500836420526, -0.01000000000000000021, -0.02392180914308270415);
	SmoothSphereHalfSpaceForce_s6_l->set_contact_sphere_location(SmoothSphereHalfSpaceForce_s6_l_location);
	double SmoothSphereHalfSpaceForce_s6_l_radius = (0.01841399999999999981);
	SmoothSphereHalfSpaceForce_s6_l->set_contact_sphere_radius(SmoothSphereHalfSpaceForce_s6_l_radius);
	SmoothSphereHalfSpaceForce_s6_l->set_contact_half_space_location(Vec3(0.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000));
	SmoothSphereHalfSpaceForce_s6_l->set_contact_half_space_orientation(Vec3(0.00000000000000000000, 0.00000000000000000000, -1.57079632679489655800));
	SmoothSphereHalfSpaceForce_s6_l->set_stiffness(1000000.00000000000000000000);
	SmoothSphereHalfSpaceForce_s6_l->set_dissipation(2.00000000000000000000);
	SmoothSphereHalfSpaceForce_s6_l->set_static_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s6_l->set_dynamic_friction(0.80000000000000004441);
	SmoothSphereHalfSpaceForce_s6_l->set_viscous_friction(0.50000000000000000000);
	SmoothSphereHalfSpaceForce_s6_l->set_transition_velocity(0.20000000000000001110);
	SmoothSphereHalfSpaceForce_s6_l->connectSocket_sphere_frame(*toes_l);
	SmoothSphereHalfSpaceForce_s6_l->connectSocket_half_space_frame(model->getGround());
	model->addComponent(SmoothSphereHalfSpaceForce_s6_l);

	OpenSim::Station* C7_study;
	C7_study = new Station(*torso, Vec3(-0.08681274619587052432, 0.41083969332286307541, 0.00230095037987998463));
	model->addComponent(C7_study);
	OpenSim::Station* r_shoulder_study;
	r_shoulder_study = new Station(*torso, Vec3(0.00328830733296130162, 0.40795946985960140907, 0.16765374066746668857));
	model->addComponent(r_shoulder_study);
	OpenSim::Station* L_shoulder_study;
	L_shoulder_study = new Station(*torso, Vec3(0.00528285389291244245, 0.40620519841911839620, -0.16586735174637587398));
	model->addComponent(L_shoulder_study);
	OpenSim::Station* r_ASIS_study;
	r_ASIS_study = new Station(*pelvis, Vec3(0.00921272860510599645, 0.02545000226297133850, 0.16175854632766228924));
	model->addComponent(r_ASIS_study);
	OpenSim::Station* L_ASIS_study;
	L_ASIS_study = new Station(*pelvis, Vec3(0.02594630707872350173, 0.02373104487509558602, -0.15117592544239696895));
	model->addComponent(L_ASIS_study);
	OpenSim::Station* r_PSIS_study;
	r_PSIS_study = new Station(*pelvis, Vec3(-0.18094401365695686468, 0.00997736817812655552, 0.05538936236356023501));
	model->addComponent(r_PSIS_study);
	OpenSim::Station* L_PSIS_study;
	L_PSIS_study = new Station(*pelvis, Vec3(-0.17559706296666471870, 0.00746998887501759690, -0.06787009730909107641));
	model->addComponent(L_PSIS_study);
	OpenSim::Station* r_knee_study;
	r_knee_study = new Station(*femur_r, Vec3(0.03312960030274941792, -0.47005272742466897018, 0.06484542680883770283));
	model->addComponent(r_knee_study);
	OpenSim::Station* L_knee_study;
	L_knee_study = new Station(*femur_l, Vec3(0.02218657739040408305, -0.47409685915404586876, -0.06273817396647354827));
	model->addComponent(L_knee_study);
	OpenSim::Station* r_mknee_study;
	r_mknee_study = new Station(*femur_r, Vec3(0.04226605025674570748, -0.46646701590468453436, -0.06534127224107266763));
	model->addComponent(r_mknee_study);
	OpenSim::Station* L_mknee_study;
	L_mknee_study = new Station(*femur_l, Vec3(0.03045399747339558316, -0.47606573398518736706, 0.06257386092657690746));
	model->addComponent(L_mknee_study);
	OpenSim::Station* r_ankle_study;
	r_ankle_study = new Station(*tibia_r, Vec3(-0.02422336057505186657, -0.48429043496801954527, 0.05632701329369216303));
	model->addComponent(r_ankle_study);
	OpenSim::Station* L_ankle_study;
	L_ankle_study = new Station(*tibia_l, Vec3(-0.02802845384359825553, -0.48114181425833429939, -0.05794925020763955215));
	model->addComponent(L_ankle_study);
	OpenSim::Station* r_mankle_study;
	r_mankle_study = new Station(*tibia_r, Vec3(0.01119571493339055479, -0.48161709412007530640, -0.04113130955760740948));
	model->addComponent(r_mankle_study);
	OpenSim::Station* L_mankle_study;
	L_mankle_study = new Station(*tibia_l, Vec3(-0.00068512946777295192, -0.47315125465569746188, 0.04323196079900953892));
	model->addComponent(L_mankle_study);
	OpenSim::Station* r_calc_study;
	r_calc_study = new Station(*calcn_r, Vec3(-0.03394398469505254673, 0.05497357384564566241, -0.01132627517128487193));
	model->addComponent(r_calc_study);
	OpenSim::Station* L_calc_study;
	L_calc_study = new Station(*calcn_l, Vec3(-0.04539262001179502487, 0.03092448933896219110, 0.00911486812230265231));
	model->addComponent(L_calc_study);
	OpenSim::Station* r_toe_study;
	r_toe_study = new Station(*calcn_r, Vec3(0.16625592047769841009, -0.01210179213456646918, 0.01000896294944031428));
	model->addComponent(r_toe_study);
	OpenSim::Station* L_toe_study;
	L_toe_study = new Station(*calcn_l, Vec3(0.17206518761656780514, 0.02100159787725158766, -0.00777170701649748663));
	model->addComponent(L_toe_study);
	OpenSim::Station* r_5meta_study;
	r_5meta_study = new Station(*calcn_r, Vec3(0.12020808599072516087, -0.00610556421281549522, 0.06591159213097924630));
	model->addComponent(r_5meta_study);
	OpenSim::Station* L_5meta_study;
	L_5meta_study = new Station(*calcn_l, Vec3(0.12541661801050635328, 0.01507749521196449782, -0.06464155376482133564));
	model->addComponent(L_5meta_study);
	OpenSim::Station* r_lelbow_study;
	r_lelbow_study = new Station(*humerus_r, Vec3(0.01491553195110957575, -0.30581103126670727832, 0.04766281817534867571));
	model->addComponent(r_lelbow_study);
	OpenSim::Station* L_lelbow_study;
	L_lelbow_study = new Station(*humerus_l, Vec3(0.00943287930360106608, -0.30992929585702988504, -0.04159933287040940009));
	model->addComponent(L_lelbow_study);
	OpenSim::Station* r_melbow_study;
	r_melbow_study = new Station(*humerus_r, Vec3(0.00185435838713632339, -0.31096697390310801268, -0.05329273634341968879));
	model->addComponent(r_melbow_study);
	OpenSim::Station* L_melbow_study;
	L_melbow_study = new Station(*humerus_l, Vec3(0.00588642356048940663, -0.31107786721970076549, 0.04998433567263402266));
	model->addComponent(L_melbow_study);
	OpenSim::Station* r_lwrist_study;
	r_lwrist_study = new Station(*radius_r, Vec3(0.00255622705069702416, -0.25973004202872163049, 0.06172073178254400272));
	model->addComponent(r_lwrist_study);
	OpenSim::Station* L_lwrist_study;
	L_lwrist_study = new Station(*radius_l, Vec3(-0.00706686115584845287, -0.25580123359352313628, -0.05890523641479583628));
	model->addComponent(L_lwrist_study);
	OpenSim::Station* r_mwrist_study;
	r_mwrist_study = new Station(*radius_r, Vec3(-0.02517298571141834973, -0.26365875820709239186, -0.03158983081180088970));
	model->addComponent(r_mwrist_study);
	OpenSim::Station* L_mwrist_study;
	L_mwrist_study = new Station(*radius_l, Vec3(-0.01960639994598788183, -0.26595070550902599749, 0.02915329911721242784));
	model->addComponent(L_mwrist_study);

	// Initialize system.
	SimTK::State* state;
	state = new State(model->initSystem());

	// Read inputs.
	std::vector<T> x(arg[0], arg[0] + NX);
	std::vector<T> u(arg[1], arg[1] + NU);
	std::vector<T> p(arg[2], arg[2] + NP);

	// States and controls.
	T ua[NU];
	Vector QsUs(NX);
	T up[NP];
	/// States
	for (int i = 0; i < NX; ++i) QsUs[i] = x[i];
	/// Controls
	/// OpenSim and Simbody have different state orders.
	auto indicesOSInSimbody = getIndicesOSInSimbody(*model);
	for (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];

	// Set state variables and realize.
	model->setStateVariableValues(*state, QsUs);
	model->realizeVelocity(*state);

	// Compute residual forces.
	/// Set appliedMobilityForces (# mobilities).
	Vector appliedMobilityForces(nCoordinates);
	appliedMobilityForces.setToZero();
	/// Set appliedBodyForces (# bodies + ground).
	Vector_<SpatialVec> appliedBodyForces;
	int nbodies = model->getBodySet().getSize() + 1;
	appliedBodyForces.resize(nbodies);
	appliedBodyForces.setToZero();
	/// Set gravity.
	Vec3 gravity(0);
	gravity[1] = -9.80664999999999942304;
	/// Add weights to appliedBodyForces.
	for (int i = 0; i < model->getBodySet().getSize(); ++i) {
		model->getMatterSubsystem().addInStationForce(*state,
			model->getBodySet().get(i).getMobilizedBodyIndex(),
			model->getBodySet().get(i).getMassCenter(),
			model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
	}
	/// Contact forces
	Vec3 AppliedPointForce_s1_l, AppliedPointForce_s2_l;
	Vec3 AppliedPointForce_s3_l, AppliedPointForce_s4_l;
	Vec3 AppliedPointForce_s5_l, AppliedPointForce_s6_l;
	Vec3 AppliedPointForce_s1_r, AppliedPointForce_s2_r;
	Vec3 AppliedPointForce_s3_r, AppliedPointForce_s4_r;
	Vec3 AppliedPointForce_s5_r, AppliedPointForce_s6_r;
	int nc = 3;
	for (int i = 0; i < nc; ++i) {
		AppliedPointForce_s1_l[i] = up[i];
		AppliedPointForce_s2_l[i] = up[i + nc];
		AppliedPointForce_s3_l[i] = up[i + nc + nc];
		AppliedPointForce_s4_l[i] = up[i + nc + nc + nc];
		AppliedPointForce_s5_l[i] = up[i + nc + nc + nc + nc];
		AppliedPointForce_s6_l[i] = up[i + nc + nc + nc + nc + nc];
		AppliedPointForce_s1_r[i] = up[i + nc + nc + nc + nc + nc + nc];
		AppliedPointForce_s2_r[i] = up[i + nc + nc + nc + nc + nc + nc + nc];
		AppliedPointForce_s3_r[i] = up[i + nc + nc + nc + nc + nc + nc + nc + nc];
		AppliedPointForce_s4_r[i] = up[i + nc + nc + nc + nc + nc + nc + nc + nc + nc];
		AppliedPointForce_s5_r[i] = up[i + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc];
		AppliedPointForce_s6_r[i] = up[i + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc];
	}
	/// Compute contact point positions in body frames
	Vec3 normal = Vec3(0, 1, 0);
	/// sphere 1 left
	Vec3 pos_InGround_HC_s1_l = calcn_l->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s1_l_location);
	Vec3 contactPointpos_InGround_HC_s1_l = pos_InGround_HC_s1_l - SmoothSphereHalfSpaceForce_s1_r_radius * normal;
	Vec3 contactPointpos_InGround_HC_s1_l_adj = contactPointpos_InGround_HC_s1_l - 0.5*contactPointpos_InGround_HC_s1_l[1] * normal;
	Vec3 contactPointPos_InBody_HC_s1_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s1_l_adj, *calcn_l);
	/// sphere 2 left
	Vec3 pos_InGround_HC_s2_l = calcn_l->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s2_l_location);
	Vec3 contactPointpos_InGround_HC_s2_l = pos_InGround_HC_s2_l - SmoothSphereHalfSpaceForce_s2_r_radius * normal;
	Vec3 contactPointpos_InGround_HC_s2_l_adj = contactPointpos_InGround_HC_s2_l - 0.5*contactPointpos_InGround_HC_s2_l[1] * normal;
	Vec3 contactPointPos_InBody_HC_s2_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s2_l_adj, *calcn_l);
	/// sphere 3 left
	Vec3 pos_InGround_HC_s3_l = calcn_l->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s3_l_location);
	Vec3 contactPointpos_InGround_HC_s3_l = pos_InGround_HC_s3_l - SmoothSphereHalfSpaceForce_s3_r_radius * normal;
	Vec3 contactPointpos_InGround_HC_s3_l_adj = contactPointpos_InGround_HC_s3_l - 0.5*contactPointpos_InGround_HC_s3_l[1] * normal;
	Vec3 contactPointPos_InBody_HC_s3_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s3_l_adj, *calcn_l);
	/// sphere 4 left
	Vec3 pos_InGround_HC_s4_l = calcn_l->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s4_l_location);
	Vec3 contactPointpos_InGround_HC_s4_l = pos_InGround_HC_s4_l - SmoothSphereHalfSpaceForce_s4_r_radius * normal;
	Vec3 contactPointpos_InGround_HC_s4_l_adj = contactPointpos_InGround_HC_s4_l - 0.5*contactPointpos_InGround_HC_s4_l[1] * normal;
	Vec3 contactPointPos_InBody_HC_s4_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s4_l_adj, *calcn_l);
	/// sphere 5 left
	Vec3 pos_InGround_HC_s5_l = toes_l->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s5_l_location);
	Vec3 contactPointpos_InGround_HC_s5_l = pos_InGround_HC_s5_l - SmoothSphereHalfSpaceForce_s5_r_radius * normal;
	Vec3 contactPointpos_InGround_HC_s5_l_adj = contactPointpos_InGround_HC_s5_l - 0.5*contactPointpos_InGround_HC_s5_l[1] * normal;
	Vec3 contactPointPos_InBody_HC_s5_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s5_l_adj, *toes_l);
	/// sphere 6 left
	Vec3 pos_InGround_HC_s6_l = toes_l->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s6_l_location);
	Vec3 contactPointpos_InGround_HC_s6_l = pos_InGround_HC_s6_l - SmoothSphereHalfSpaceForce_s6_r_radius * normal;
	Vec3 contactPointpos_InGround_HC_s6_l_adj = contactPointpos_InGround_HC_s6_l - 0.5*contactPointpos_InGround_HC_s6_l[1] * normal;
	Vec3 contactPointPos_InBody_HC_s6_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s6_l_adj, *toes_l);
	/// sphere 1 right
	Vec3 pos_InGround_HC_s1_r = calcn_r->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s1_r_location);
	Vec3 contactPointpos_InGround_HC_s1_r = pos_InGround_HC_s1_r - SmoothSphereHalfSpaceForce_s1_l_radius * normal;
	Vec3 contactPointpos_InGround_HC_s1_r_adj = contactPointpos_InGround_HC_s1_r - 0.5*contactPointpos_InGround_HC_s1_r[1] * normal;
	Vec3 contactPointPos_InBody_HC_s1_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s1_r_adj, *calcn_r);
	/// sphere 2 right
	Vec3 pos_InGround_HC_s2_r = calcn_r->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s2_r_location);
	Vec3 contactPointpos_InGround_HC_s2_r = pos_InGround_HC_s2_r - SmoothSphereHalfSpaceForce_s2_l_radius * normal;
	Vec3 contactPointpos_InGround_HC_s2_r_adj = contactPointpos_InGround_HC_s2_r - 0.5*contactPointpos_InGround_HC_s2_r[1] * normal;
	Vec3 contactPointPos_InBody_HC_s2_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s2_r_adj, *calcn_r);
	/// sphere 3 right
	Vec3 pos_InGround_HC_s3_r = calcn_r->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s3_r_location);
	Vec3 contactPointpos_InGround_HC_s3_r = pos_InGround_HC_s3_r - SmoothSphereHalfSpaceForce_s3_l_radius * normal;
	Vec3 contactPointpos_InGround_HC_s3_r_adj = contactPointpos_InGround_HC_s3_r - 0.5*contactPointpos_InGround_HC_s3_r[1] * normal;
	Vec3 contactPointPos_InBody_HC_s3_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s3_r_adj, *calcn_r);
	/// sphere 4 right
	Vec3 pos_InGround_HC_s4_r = calcn_r->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s4_r_location);
	Vec3 contactPointpos_InGround_HC_s4_r = pos_InGround_HC_s4_r - SmoothSphereHalfSpaceForce_s4_l_radius * normal;
	Vec3 contactPointpos_InGround_HC_s4_r_adj = contactPointpos_InGround_HC_s4_r - 0.5*contactPointpos_InGround_HC_s4_r[1] * normal;
	Vec3 contactPointPos_InBody_HC_s4_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s4_r_adj, *calcn_r);
	/// sphere 5 right
	Vec3 pos_InGround_HC_s5_r = toes_r->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s5_r_location);
	Vec3 contactPointpos_InGround_HC_s5_r = pos_InGround_HC_s5_r - SmoothSphereHalfSpaceForce_s5_l_radius * normal;
	Vec3 contactPointpos_InGround_HC_s5_r_adj = contactPointpos_InGround_HC_s5_r - 0.5*contactPointpos_InGround_HC_s5_r[1] * normal;
	Vec3 contactPointPos_InBody_HC_s5_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s5_r_adj, *toes_r);
	/// sphere 6 right
	Vec3 pos_InGround_HC_s6_r = toes_r->findStationLocationInGround(*state, SmoothSphereHalfSpaceForce_s6_r_location);
	Vec3 contactPointpos_InGround_HC_s6_r = pos_InGround_HC_s6_r - SmoothSphereHalfSpaceForce_s6_l_radius * normal;
	Vec3 contactPointpos_InGround_HC_s6_r_adj = contactPointpos_InGround_HC_s6_r - 0.5*contactPointpos_InGround_HC_s6_r[1] * normal;
	Vec3 contactPointPos_InBody_HC_s6_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s6_r_adj, *toes_r);
	/// Add contact forces to appliedBodyForces
	model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s1_l, AppliedPointForce_s1_l, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s2_l, AppliedPointForce_s2_l, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s3_l, AppliedPointForce_s3_l, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s4_l, AppliedPointForce_s4_l, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, toes_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s5_l, AppliedPointForce_s5_l, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, toes_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s6_l, AppliedPointForce_s6_l, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s1_r, AppliedPointForce_s1_r, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s2_r, AppliedPointForce_s2_r, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s3_r, AppliedPointForce_s3_r, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s4_r, AppliedPointForce_s4_r, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, toes_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s5_r, AppliedPointForce_s5_r, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, toes_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s6_r, AppliedPointForce_s6_r, appliedBodyForces);

	/// Extract contact forces from model
	Array<osim_double_adouble> Force_0 = SmoothSphereHalfSpaceForce_s1_r->getRecordValues(*state);
	Vec3 GRF_0;
	//GRF_0[0] = Vec3(Force_0[3], Force_0[4], Force_0[5]);
	GRF_0 = Vec3(Force_0[0], Force_0[1], Force_0[2]);
	/*int c_idx_0 = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_0] += GRF_0;*/

	Array<osim_double_adouble> Force_1 = SmoothSphereHalfSpaceForce_s2_r->getRecordValues(*state);
	Vec3 GRF_1;
	//GRF_1[0] = Vec3(Force_1[3], Force_1[4], Force_1[5]);
	GRF_1 = Vec3(Force_1[0], Force_1[1], Force_1[2]);
	/*int c_idx_1 = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_1] += GRF_1;*/

	Array<osim_double_adouble> Force_2 = SmoothSphereHalfSpaceForce_s3_r->getRecordValues(*state);
	Vec3 GRF_2;
	//GRF_2[0] = Vec3(Force_2[3], Force_2[4], Force_2[5]);
	GRF_2 = Vec3(Force_2[0], Force_2[1], Force_2[2]);
	/*int c_idx_2 = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_2] += GRF_2;*/

	Array<osim_double_adouble> Force_3 = SmoothSphereHalfSpaceForce_s4_r->getRecordValues(*state);
	Vec3 GRF_3;
	//GRF_3[0] = Vec3(Force_3[3], Force_3[4], Force_3[5]);
	GRF_3 = Vec3(Force_3[0], Force_3[1], Force_3[2]);
	/*int c_idx_3 = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_3] += GRF_3;*/

	Array<osim_double_adouble> Force_4 = SmoothSphereHalfSpaceForce_s5_r->getRecordValues(*state);
	Vec3 GRF_4;
	//GRF_4[0] = Vec3(Force_4[3], Force_4[4], Force_4[5]);
	GRF_4 = Vec3(Force_4[0], Force_4[1], Force_4[2]);
	/*int c_idx_4 = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_4] += GRF_4;*/

	Array<osim_double_adouble> Force_5 = SmoothSphereHalfSpaceForce_s6_r->getRecordValues(*state);
	Vec3 GRF_5;
	//GRF_5[0] = Vec3(Force_5[3], Force_5[4], Force_5[5]);
	GRF_5 = Vec3(Force_5[0], Force_5[1], Force_5[2]);
	/*int c_idx_5 = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_5] += GRF_5;*/

	Array<osim_double_adouble> Force_6 = SmoothSphereHalfSpaceForce_s1_l->getRecordValues(*state);
	Vec3 GRF_6;
	//GRF_6[0] = Vec3(Force_6[3], Force_6[4], Force_6[5]);
	GRF_6 = Vec3(Force_6[0], Force_6[1], Force_6[2]);
	/*int c_idx_6 = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_6] += GRF_6;*/

	Array<osim_double_adouble> Force_7 = SmoothSphereHalfSpaceForce_s2_l->getRecordValues(*state);
	Vec3 GRF_7;
	//GRF_7[0] = Vec3(Force_7[3], Force_7[4], Force_7[5]);
	GRF_7 = Vec3(Force_7[0], Force_7[1], Force_7[2]);
	/*int c_idx_7 = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_7] += GRF_7;*/

	Array<osim_double_adouble> Force_8 = SmoothSphereHalfSpaceForce_s3_l->getRecordValues(*state);
	Vec3 GRF_8;
	//GRF_8[0] = Vec3(Force_8[3], Force_8[4], Force_8[5]);
	GRF_8 = Vec3(Force_8[0], Force_8[1], Force_8[2]);
	/*int c_idx_8 = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_8] += GRF_8;*/

	Array<osim_double_adouble> Force_9 = SmoothSphereHalfSpaceForce_s4_l->getRecordValues(*state);
	Vec3 GRF_9;
	//GRF_9[0] = Vec3(Force_9[3], Force_9[4], Force_9[5]);
	GRF_9 = Vec3(Force_9[0], Force_9[1], Force_9[2]);
	/*int c_idx_9 = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_9] += GRF_9;*/

	Array<osim_double_adouble> Force_10 = SmoothSphereHalfSpaceForce_s5_l->getRecordValues(*state);
	Vec3 GRF_10;
	//GRF_10[0] = Vec3(Force_10[3], Force_10[4], Force_10[5]);
	GRF_10 = Vec3(Force_10[0], Force_10[1], Force_10[2]);
	/*int c_idx_10 = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_10] += GRF_10;*/

	Array<osim_double_adouble> Force_11 = SmoothSphereHalfSpaceForce_s6_l->getRecordValues(*state);
	Vec3 GRF_11;
	//GRF_11[0] = Vec3(Force_11[3], Force_11[4], Force_11[5]);
	GRF_11 = Vec3(Force_11[0], Force_11[1], Force_11[2]);
	/*int c_idx_11 = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
	appliedBodyForces[c_idx_11] += GRF_11;*/

	/// knownUdot.
	Vector knownUdot(nCoordinates);
	knownUdot.setToZero();
	for (int i = 0; i < nCoordinates; ++i) knownUdot[i] = ua[i];
	/// Calculate residual forces.
	Vector residualMobilityForces(nCoordinates);
	residualMobilityForces.setToZero();
	model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
		appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);

	/// Marker positions.
	Vec3 C7_study_location = C7_study->getLocationInGround(*state);
	Vec3 r_shoulder_study_location = r_shoulder_study->getLocationInGround(*state);
	Vec3 L_shoulder_study_location = L_shoulder_study->getLocationInGround(*state);
	Vec3 r_ASIS_study_location = r_ASIS_study->getLocationInGround(*state);
	Vec3 L_ASIS_study_location = L_ASIS_study->getLocationInGround(*state);
	Vec3 r_PSIS_study_location = r_PSIS_study->getLocationInGround(*state);
	Vec3 L_PSIS_study_location = L_PSIS_study->getLocationInGround(*state);
	Vec3 r_knee_study_location = r_knee_study->getLocationInGround(*state);
	Vec3 L_knee_study_location = L_knee_study->getLocationInGround(*state);
	Vec3 r_mknee_study_location = r_mknee_study->getLocationInGround(*state);
	Vec3 L_mknee_study_location = L_mknee_study->getLocationInGround(*state);
	Vec3 r_ankle_study_location = r_ankle_study->getLocationInGround(*state);
	Vec3 L_ankle_study_location = L_ankle_study->getLocationInGround(*state);
	Vec3 r_mankle_study_location = r_mankle_study->getLocationInGround(*state);
	Vec3 L_mankle_study_location = L_mankle_study->getLocationInGround(*state);
	Vec3 r_calc_study_location = r_calc_study->getLocationInGround(*state);
	Vec3 L_calc_study_location = L_calc_study->getLocationInGround(*state);
	Vec3 r_toe_study_location = r_toe_study->getLocationInGround(*state);
	Vec3 L_toe_study_location = L_toe_study->getLocationInGround(*state);
	Vec3 r_5meta_study_location = r_5meta_study->getLocationInGround(*state);
	Vec3 L_5meta_study_location = L_5meta_study->getLocationInGround(*state);
	Vec3 r_lelbow_study_location = r_lelbow_study->getLocationInGround(*state);
	Vec3 L_lelbow_study_location = L_lelbow_study->getLocationInGround(*state);
	Vec3 r_melbow_study_location = r_melbow_study->getLocationInGround(*state);
	Vec3 L_melbow_study_location = L_melbow_study->getLocationInGround(*state);
	Vec3 r_lwrist_study_location = r_lwrist_study->getLocationInGround(*state);
	Vec3 L_lwrist_study_location = L_lwrist_study->getLocationInGround(*state);
	Vec3 r_mwrist_study_location = r_mwrist_study->getLocationInGround(*state);
	Vec3 L_mwrist_study_location = L_mwrist_study->getLocationInGround(*state);

	/// Residual forces.
	/// OpenSim and Simbody have different state orders so we need to adjust
	auto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);
	for (int i = 0; i < NU; ++i) res[0][i] =
		value<T>(residualMobilityForces[indicesSimbodyInOS[i]]);
	/// Marker positions.
	for (int i = 0; i < nc; ++i) res[0][i + NU + 0 * nc] = value<T>(C7_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 1 * nc] = value<T>(r_shoulder_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 2 * nc] = value<T>(L_shoulder_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 3 * nc] = value<T>(r_ASIS_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 4 * nc] = value<T>(L_ASIS_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 5 * nc] = value<T>(r_PSIS_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 6 * nc] = value<T>(L_PSIS_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 7 * nc] = value<T>(r_knee_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 8 * nc] = value<T>(L_knee_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 9 * nc] = value<T>(r_mknee_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 10 * nc] = value<T>(L_mknee_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 11 * nc] = value<T>(r_ankle_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 12 * nc] = value<T>(L_ankle_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 13 * nc] = value<T>(r_mankle_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 14 * nc] = value<T>(L_mankle_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 15 * nc] = value<T>(r_calc_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 16 * nc] = value<T>(L_calc_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 17 * nc] = value<T>(r_toe_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 18 * nc] = value<T>(L_toe_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 19 * nc] = value<T>(r_5meta_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 20 * nc] = value<T>(L_5meta_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 21 * nc] = value<T>(r_lelbow_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 22 * nc] = value<T>(L_lelbow_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 23 * nc] = value<T>(r_melbow_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 24 * nc] = value<T>(L_melbow_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 25 * nc] = value<T>(r_lwrist_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 26 * nc] = value<T>(L_lwrist_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 27 * nc] = value<T>(r_mwrist_study_location[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 28 * nc] = value<T>(L_mwrist_study_location[i]);
	/// Contact forces.
	for (int i = 0; i < nc; ++i) res[0][i + NU + 29 * nc] = value<T>(GRF_6[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 30 * nc] = value<T>(GRF_7[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 31 * nc] = value<T>(GRF_8[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 32 * nc] = value<T>(GRF_9[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 33 * nc] = value<T>(GRF_10[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 34 * nc] = value<T>(GRF_11[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 35 * nc] = value<T>(GRF_0[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 36 * nc] = value<T>(GRF_1[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 37 * nc] = value<T>(GRF_2[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 38 * nc] = value<T>(GRF_3[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 39 * nc] = value<T>(GRF_4[i]);
	for (int i = 0; i < nc; ++i) res[0][i + NU + 40 * nc] = value<T>(GRF_5[i]);

	return 0;
}

int main() {
	Recorder x[NX];
	Recorder u[NU];
	Recorder p[NP];
	Recorder tau[NR];
	for (int i = 0; i < NX; ++i) x[i] <<= 0;
	for (int i = 0; i < NU; ++i) u[i] <<= 0;
	for (int i = 0; i < NP; ++i) p[i] <<= 0;
	const Recorder* Recorder_arg[n_in] = { x,u,p };
	Recorder* Recorder_res[n_out] = { tau };
	F_generic<Recorder>(Recorder_arg, Recorder_res);
	double res[NR];
	for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];
	Recorder::stop_recording();
	return 0;
}