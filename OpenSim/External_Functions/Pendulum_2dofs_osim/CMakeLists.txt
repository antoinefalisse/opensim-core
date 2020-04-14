set(TEST_TARGET Pendulum_2dofs_osim)
add_executable(${TEST_TARGET} ${TEST_TARGET}.cpp)
target_link_libraries(${TEST_TARGET} osimSimulation)
set_target_properties(${TEST_TARGET} PROPERTIES
    FOLDER "External_Functions")
OpenSimCopySharedTestFiles(Pendulum2dof.osim)