#ifndef OPENSIM_PRINT_XML_H_
#define OPENSIM_PRINT_XML_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  opensim_print_xml.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib, Chris Dembia                    *
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

#include <iostream>

#include <docopt.h>

#include <OpenSim/OpenSim.h>

static const char HELP_PRINT_XML[] =
R"(Print a template XML file for a Tool or class.

Usage:
  opensim [options]... print-xml <tool-or-class> [<output-file>]
  opensim print-xml -h | --help

Options:
  -L <path>, --library <path>  Load a plugin.

Description:
  The argument <tool-or-class> can be the name of a Tool
  
            scale  ik  id  rra  cmc  forward  analyze

  or the name of any registered OpenSim class (even from a plugin).

  The template file is written to <output-file> if provided; otherwise, the
  file is written with the name `default_<class-name>.xml` to the current
  directory.

Examples:
  opensim print-xml cmc
  opensim print-xml Millard2012EquilibriumMuscle 
)";

int print_xml(int argc, const char** argv) {

    using namespace OpenSim;

    std::map<std::string, docopt::value> args = docopt::docopt(
            HELP_PRINT_XML, { argv + 1, argv + argc },
            true); // show help if requested

    // Parse arguments.
    // ----------------

    // Tool or class.
    std::string toolOrClass = args["<tool-or-class>"].asString();
    std::string className;
    if      (toolOrClass == "scale")   className = "ScaleTool";
    else if (toolOrClass == "ik")      className = "InverseKinematicsTool";
    else if (toolOrClass == "id")      className = "InverseDynamicsTool";
    else if (toolOrClass == "rra")     className = "RRATool";
    else if (toolOrClass == "cmc")     className = "CMCTool";
    else if (toolOrClass == "forward") className = "ForwardTool";
    else if (toolOrClass == "analyze") className = "AnalyzeTool";
    else                               className = toolOrClass;

    // Output file.
    std::string outputFile;
    if (args["<output-file>"]) outputFile = args["<output-file>"].asString();
    else                       outputFile = "default_" + className + ".xml";


    // Print the XML file.
    // -------------------
    const auto* obj = Object::getDefaultInstanceOfType(className);

    if (!obj) {
        throw Exception("There is no tool or class named '" + className + "'.\n"
                "Did you intend to load a plugin (with --library)?");
    }

    std::cout << "Printing '" << outputFile << "'." << std::endl;
    obj->print(outputFile);

    return EXIT_SUCCESS;
}

#endif // OPENSIM_PRINT_XML_H_