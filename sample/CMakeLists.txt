choreonoid_add_simple_controller(Jaxon2WalkPatternController
                                 Jaxon2WalkPatternController.cpp)
choreonoid_add_simple_controller(Jaxon2WalkController Jaxon2WalkController.cpp)

install(FILES Jaxon2Walk.cnoid Jaxon2Stabilizer.cnoid
        DESTINATION ${CNOID_SHARE_SUBDIR}/project)
