# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector")
  file(MAKE_DIRECTORY "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector")
endif()
file(MAKE_DIRECTORY
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/reflector"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/_sysbuild/sysbuild/images/reflector-prefix"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/_sysbuild/sysbuild/images/reflector-prefix/tmp"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/_sysbuild/sysbuild/images/reflector-prefix/src/reflector-stamp"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/_sysbuild/sysbuild/images/reflector-prefix/src"
  "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/_sysbuild/sysbuild/images/reflector-prefix/src/reflector-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/_sysbuild/sysbuild/images/reflector-prefix/src/reflector-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/antoninbo/Bureau/260709/cs_localisation_v3_8_3_capsguard/cs_localisation_fixed/reflector/build/_sysbuild/sysbuild/images/reflector-prefix/src/reflector-stamp${cfgdir}") # cfgdir has leading slash
endif()
