choreonoid_add_simple_controller(Jaxon2StandingStabilizer
                                 Jaxon2StandingStabilizer.cpp)
choreonoid_add_simple_controller(Jaxon2WalkPatternController
                                 Jaxon2WalkPatternController.cpp)
choreonoid_add_simple_controller(Jaxon2WalkStabilizer Jaxon2WalkStabilizer.cpp)

install(FILES Jaxon2Walk.cnoid Jaxon2WalkStabilizer.cnoid
              Jaxon2StandingStabilizer.cnoid
        DESTINATION ${CNOID_SHARE_SUBDIR}/project)
