# Choreonoid JAXON2 draft version

## Overview

This repository provides JAXON2 model files and samples for Choreonoid.

JAXON2 is a biped humanoid robot developed by the JSK laboratory at the University of Tokyo.
The original JAXON2 model files written in the OpenHRP3 format were developed by JSK and are
published on the following GitHub repository.  
https://github.com/start-jsk/rtmros_choreonoid

The model files of JAXON2 in this repository are reconstructed as body files for Choreonoid,
and are distributed with the permission of JSK. The license of this version follows the
original license, Creative Commons Attribution-ShareAlike 4.0 International License.

This is a draft version, and the official version will be shipped with Choreonoid as a sample modelf of it.

Also, we are developing a sample walking controller to make this model walk.

## Installation

Clone this repository into the ext directory of the Choreonoid source, and turn on BUILD_JAXON2_SAMPLES in the CMake build configuration, then a set of files is installed in the Choreonoid share directory.
