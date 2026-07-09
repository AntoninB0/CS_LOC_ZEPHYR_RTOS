# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator")
  file(MAKE_DIRECTORY "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator")
endif()
file(MAKE_DIRECTORY
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/initiator"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/_sysbuild/sysbuild/images/initiator-prefix"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/_sysbuild/sysbuild/images/initiator-prefix/tmp"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/_sysbuild/sysbuild/images/initiator-prefix/src/initiator-stamp"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/_sysbuild/sysbuild/images/initiator-prefix/src"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/_sysbuild/sysbuild/images/initiator-prefix/src/initiator-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/_sysbuild/sysbuild/images/initiator-prefix/src/initiator-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/initiator/build/_sysbuild/sysbuild/images/initiator-prefix/src/initiator-stamp${cfgdir}") # cfgdir has leading slash
endif()
