# Lepton_Driver
`Lepton_Driver` is a C++ Library for use with the FLIR Lepton Camera over a USB connection. 

`Lepton_Driver` is based on [GetThermal](https://github.com/groupgets/GetThermal), with significant changes, removing `Qt` and their custom `libuvc` fork from dependencies. This allows easy integration into other projects.

`Lepton_Driver` is compatible with the
[PureThermal Mini](https://groupgets.com/manufacturers/getlab/products/purethermal-mini-flir-lepton-smart-i-o-module)
breakout board from GroupGets, and it should be compatible with any breakout board that provides a USB connection for accessing the camera, including the
[PureThermal 1](https://groupgets.com/manufacturers/getlab/products/purethermal-1-flir-lepton-smart-i-o-module) or the
[PureThermal 2](https://groupgets.com/manufacturers/getlab/products/purethermal-2-flir-lepton-smart-i-o-module), also from GroupGets.

# Requirements

`Lepton_Driver` Requires [CMake](http://www.cmake.org/), `libusb-1.0` `OpenCV`, and `libuvc`.

# Build

`Lepton_Driver` is a CMake project, so building is easy
```asm
cd /path/to/lepton_driver
mkdir build
cd build
cmake ..
make

# If you want lepton_driver to be available system wide, run
make install
```
# Example

An example of how to use `Lepton_Driver` is included. See `/example/lepton_example.cpp`

# Documentation

`Lepton_Driver` is documented with doxygen. 

You will need to install doxygen to generate documentation, but the library will work without generating documentation.

To generate documentation, in the root directory run:
```asm
doxygen doxyfile
```
A folder named `html` will be generated, open `html/index.html` documentation.