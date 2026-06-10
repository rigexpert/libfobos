# libfobos RigExpert Fobos SDR API

This is the Fobos SDR receiver host software API shared library. Lightweight and easy to start. Full source code. Application examples.

## Versions

See [versions.txt](versions.txt)

## Platforms tested on

- Linux (Ubuntu 18.04 LTS, Ubuntu 22.04 LTS, Raspbian ...)
- Windows (7, 8.1, 10, 11) x86, x64

## Requirements

- git v.2.31 or later (otherwise download the repository manualy: Code->Download ZIP)
- any **c** compiler (tested on gcc, g++, mingw, msvc) 

## Dependencies

- libusb-1.0-0 v.1.0.21 or higher

## How to build and evaluate

### Linux
```
git clone [this repo]
cd libfobos
mkdir build
cd build
cmake ..
make
sudo make install
sudo udevadm control --reload-rules
sudo udevadm trigger
```
### Windows
```
git clone [this repo]
cd libfobos
mkdir build
cd build
cmake ..
```
to build Win32 binaries (legacy software) use

```
cmake .. -A Win32
```

Visit https://github.com/libusb/libusb/releases<br />
Download any libusb release 7z pack, for example  libusb-1.0.27.7z<br />
Unpack content of **libusb-1.0.27.7z** to **libusb** directory<br />
```
cmake --build . --config Release
```
or<br />
open **fobos.sln** in your favorite **MS VisualStudio IDE**, build, debug, trace, evaluate.<br />

### MacOS
Make sure you have libusb installed: `brew install libusb`
```
git clone [this repo]
cd libfobos
mkdir build
cd build
cmake ..
make
```
Verify the device is recognized: `build/mac/fobos_devinfo`

Receive some sample: `build/mac/fobos_recorder`. Then you can open the wav file with inspectrum (`brew install inspectrum`): `inspectrum rx.iq.wav` 

## How to use Fobos SDR API elsewhere

- build, install, include header **fobos.h**, link library **libfobos.so**, **fobos.dll**  
- see **eval/fobos_devinfo_main.c**  for simple device enumeration example
- see **eval/fobos_recorder_main.c** for basic application example 
- feel free to evaluate

## How it looks like

<img src="./showimg/Screenshot001.png" scale="100%"/><br />
<img src="./showimg/Screenshot002.png" scale="100%"/><br />
<img src="./showimg/Screenshot003.png" scale="100%"/><br />

## What is actually Fobos SDR

For more info visit the main product page

https://rigexpert.com/en/products/kits-en/fobos-sdr/
