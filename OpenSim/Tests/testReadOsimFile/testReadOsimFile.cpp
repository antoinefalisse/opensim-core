/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testExampleMain.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Cassidy Kelly                                                   *
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

// Author: Cassidy Kelly

//==============================================================================
//==============================================================================

//#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main()
{
	try {
		Model model("arm26.osim");
		std::cout << "model arm 26 loaded" << std::endl;
		State state = model.initSystem();
		std::cout << "state created" << std::endl;
		int nmob = model.getMatterSubsystem().getNumMobilities();
		std::cout << "num mobilities" << nmob << std::endl;
		Matrix M(nmob, nmob);
		M.setToZero();
		std::cout << "Mass Matrix created" << std::endl;
		Vector QsUs(nmob);
		QsUs.setToZero();
		model.setStateVariableValues(state, QsUs);
		model.realizePosition(state);
		std::cout << "model realized position" << std::endl;
		model.getMatterSubsystem().calcM(state, M);
		std::cout << "inertia matrix of arm26 is: \n" << M << std::endl;
		Matrix Minv(nmob, nmob);
		M.setToZero();
		model.getMatterSubsystem().calcMInv(state, Minv);
		std::cout << "inverse of inertia matrix of arm26 is: \n" << Minv << std::endl;
		std::cout << "num bodies=" << model.getBodySet().getSize() << std::endl;
		Inertia I_arm1 = model.getBodySet().get(0).getInertia();
		std::cout << "Inertia of the 1st body" <<  I_arm1 << std::endl;
		Inertia I_arm2 = model.getBodySet().get(1).getInertia();
		std::cout << "Inertia of the 2nd body" << I_arm2 << std::endl;
	}
	catch (const std::exception& ex) {
		std::cout << "error loading arm 26" << std::endl;
		std::cout << ex.what() << std::endl;
	}

	try {
		Model model("gait2354_simbody_noMuscles_nokneeYamag.osim");
		State state = model.initSystem();
		std::cout << "state created" << std::endl;
		int nmob = model.getMatterSubsystem().getNumMobilities();
		std::cout << "num mobilities" << nmob << std::endl;
		Matrix M(nmob, nmob);
		M.setToZero();
		std::cout << "Mass Matrix created" << std::endl;
		Vector QsUs(nmob);
		QsUs.setToZero();
		model.setStateVariableValues(state, QsUs);
		model.realizePosition(state);
		std::cout << "model realized position" << std::endl;
		model.getMatterSubsystem().calcM(state, M);
		Array<string> statevarnames=model.getStateVariableNames();
		std::cout << statevarnames[20*2] << std::endl;
		std::cout << "inertia matrix of gait2354 is: \n" << M << std::endl;
		std::cout << "model gait2354 loaded" << std::endl;
	}
	catch (const std::exception& ex) {
		std::cout << "error loading gait2354 " << std::endl;
		std::cout << ex.what() << std::endl;
		return 1;
	}

    cout << "Done" << endl;
    return 0;
}
