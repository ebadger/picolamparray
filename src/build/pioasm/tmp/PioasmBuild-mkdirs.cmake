# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/msys64/home/ebadger/pico-sdk/tools/pioasm"
  "C:/msys64/home/ebadger/picolamparray/src/build/pioasm"
  "C:/msys64/home/ebadger/picolamparray/src/build/pioasm"
  "C:/msys64/home/ebadger/picolamparray/src/build/pioasm/tmp"
  "C:/msys64/home/ebadger/picolamparray/src/build/pioasm/src/PioasmBuild-stamp"
  "C:/msys64/home/ebadger/picolamparray/src/build/pioasm/src"
  "C:/msys64/home/ebadger/picolamparray/src/build/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/msys64/home/ebadger/picolamparray/src/build/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
