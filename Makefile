# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
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

.PHONY: cpp
cpp:
	cmake -G Ninja -S cpp -B build_cpp -DCMAKE_BUILD_TYPE=Debug -DCMAKE_VERBOSE_MAKEFILE=On; touch build_cpp/COLCON_IGNORE
	cmake --build build_cpp; touch build_cpp/COLCON_IGNORE

.PHONY: release_build
release_build:
	cmake -G Ninja -S . -B build_cpp -DCMAKE_BUILD_TYPE=Release; touch build_cpp/COLCON_IGNORE
	cmake --build build_cpp; touch build_cpp/COLCON_IGNORE

.PHONY: clean
clean:
	rm -rf build_cpp

.PHONY: clean_all
clean_all:
	rm -rf build build_cpp
