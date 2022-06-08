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
    > PS> .\bootstrap-vcpkg.bat
    > PS> .\vcpkg integrate install
    > ```
    >
    > settings.json
    > ```
    > {
    >   "cmake.configureSettings": {
    >      "CMAKE_TOOLCHAIN_FILE": "[vcpkg root]/scripts/buildsystems/vcpkg.cmake"
    >   }
    > }
    > ```
  }
}
2. macOS<br>
    > `$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`
3. Linux<br>
    > `$ apt -V`

## 6. PCL 설치

1. Windows<br>
    > `PS> .\vcpkg install boost:x64-windows`
    > `PS> .\vcpkg install eigen3:x64-windows`
    > `PS> .\vcpkg install flann:x64-windows`
    > `PS> .\vcpkg install qt:x64-windows`
    > `PS> .\vcpkg install vtk:x64-windows`
    > `PS> .\vcpkg install pcl:x64-windows`
2. macOS<br>
    > `$ brew install boost`
    > `$ brew install eigen`
    > `$ brew install flann`
    > `$ brew install qt`
    > `$ brew install vtk`
    > `$ brew install pcl`
3. Linux<br>
    > `$ sudo apt install libpcl-dev`

ref) https://pointclouds.org/downloads/

<br>

## 7. Intel RealSense 설치

1. Windows<br>
    > `PS> .\vcpkg install libusb:x64-windows`
    > `PS> .\vcpkg install curl:x64-windows`
    > `PS> .\vcpkg install zlib:x64-windows`
    > `PS> .\vcpkg install realsense2:x64-windows`
2. macOS<br>
    > `$ brew install libusb`
    > `$ brew install pkg-config`
    > `$ brew install glfw`
    > `$ brew install openssl`
    > `$ brew install librealsense` # not working

```
cmake .. -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" \
-DCMAKE_THREAD_LIBS_INIT="-lpthread" \
-DCMAKE_BUILD_TYPE=RELEASE \
-DBUILD_SHARED_LIBS=ON \
-DBUILD_EXAMPLES=false \
-DBUILD_WITH_OPENMP=false \
-DHWM_OVER_XU=false \
-DOPENSSL_ROOT_DIR=/opt/homebrew/opt/openssl \
-DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
```
1. Linux<br>
    > `$ sudo apt install libpcl-dev`

ref) https://github.com/IntelRealSense/librealsense
ref) https://ikaros79.tistory.com/entry/Intel-RealSense-SDK-macOS%EC%97%90-%EC%84%A4%EC%B9%98%ED%95%98%EA%B8%B0

<br>

## 주의사항
- 폴더이름에 한글 X

<br>

## 