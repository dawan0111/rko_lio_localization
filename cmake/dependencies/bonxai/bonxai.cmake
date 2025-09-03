# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
if(RKO_LIO_ENABLE_FETCHCONTENT)
  include(FetchContent)
  FetchContent_Declare(
    bonxai
    GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
    GIT_TAG 47ffd0a2917c899f6199dfa71445481164298006
    SOURCE_SUBDIR
    bonxai_core
    SYSTEM
    EXCLUDE_FROM_ALL
    OVERRIDE_FIND_PACKAGE)
  FetchContent_MakeAvailable(bonxai)
else()
  add_subdirectory(
    ${CMAKE_CURRENT_LIST_DIR}/bonxai/bonxai_core
    ${CMAKE_BINARY_DIR}/bonxai-oosbuild
    EXCLUDE_FROM_ALL
    SYSTEM)
endif()
