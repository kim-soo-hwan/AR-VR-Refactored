# AR/VR (Capstone Design)

## 0. Source Code 사용
```bash
# clone the repository
$ git clone git@github.com:kim-soo-hwan/AR-VR.git
$ cd AR-VR

# init the submodules
$ git submodule init

# update the submodules
$ git submodule update --recursive
```

<br>

## 1. C++ Compiler 설치
1. Windows <br>
    > [LLVM Download](https://releases.llvm.org/download.html) > LLVM-win64.exe
2. macOS <br>
    > clang is already installed.
3. Linux <br>
    > g++ is already installed.

<br>

## 2. CMake 설치
1. Windows <br>
    > [CMake Download](https://cmake.org/download/)
2. macOS<br>
    > `$ brew install cmake`
3. Linux<br>
    > `$ sudo apt install cmake`

<br>

## 3. Visual Studio Code 설치
1. Windows/macOS/Linux <br>
    > [VSCode Download](https://code.visualstudio.com/)
2. Extensions 설치
   1. C/C++
   2. C/C++ Extension Pack
   3. CMake Tools

<br>

## 4. CMake Tools 사용
1. Ctrl + Shift + P
2. CMake: Select a Kit
3. CMake: Configure
4. CMake: Select a Variant
5. CMake: Build

<br>

## 5. Package Manager 설치

1. Windows<br>
    > ```
    > PS> git clone https://github.com/Microsoft/vcpkg.git
    > PS> cd vcpkg
    > PS> .\bootstrap-vcpkg.sh
    > PS> .\vcpkg integrate install
    > ```
2. macOS<br>
    > `$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`
3. Linux<br>
    > `$ apt -V`

## 6. PCL 설치

1. Windows<br>
    > `PS> .\vcpkg install pcl`
2. macOS<br>
    > `$ brew install pcl`
3. Linux<br>
    > `$ sudo apt install libpcl-dev`

ref) https://pointclouds.org/downloads/

<br>

## 7. Intel RealSense 설치

1. Windows<br>
    > `PS> .\vcpkg install realsense2`
2. macOS<br>
    > `$ brew install librealsense`
3. Linux<br>
    > `$ sudo apt install libpcl-dev`

ref) https://github.com/IntelRealSense/librealsense

<br>

## 주의사항
- 폴더이름에 한글 X

<br>

## 