# Installationshinweise

Hier sind für einige wichtige Programme installationshinweise aufgelistet

## Installation von CUDA, CUDNN
Auf diesen beiden Seiten wird ganz gut erklärt, wie CUDA und CUDNN zu installieren sind:

- [hier](https://medium.com/repro-repo/install-cuda-and-cudnn-for-tensorflow-gpu-on-ubuntu-79306e4ac04e)
- [hier](https://www.thomas-krenn.com/de/wiki/CUDA_Installation_unter_Ubuntu)

Falls die Seiten nicht mehr gehen, sind unten die wichtigsten Schritte dargestellt

## Nvidia Treiber
```bash
sudo apt purge nvidia*
sudo add-apt-repository ppa:graphics-drivers
sudo apt update
sudo apt install nvidia-390
```

Gucken ob alter treiber weg ist und neuer drauf
```bash
lsmod | grep nouveau
lsmod | grep nvidia
```
Treiber als fest markieren und restart
```bash
sudo apt-mark hold nvidia-390
restart
```

## Install CUDA 9.0

```bash
cd
wget https://developer.nvidia.com/compute/cuda/9.0/Prod/local_installers/cuda_9.0.176_384.81_linux-run

chmod +x cuda_9.0.176_384.81_linux-run
./cuda_9.0.176_384.81_linux-run --extract=$HOME

sudo ./cuda-linux.9.0.176-22781540.run
sudo ./cuda-samples.9.0.176-22781540-linux.run
```

In bashrc hinzufügen:
```bash
export PATH=/usr/local/cuda-9.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

Reboot!!!

```bash
cd /usr/local/cuda-9.0/samples
sudo make
```

Wenn compile-error kommt mit /usr/bin/ld: cannot find -lnvcuvid
```bash
sudo ln -s /usr/lib/nvidia-390/libnvcuvid.so /usr/lib/libnvcuvid.so
sudo ln -s /usr/lib/nvidia-390/libnvcuvid.so.1 /usr/lib/libnvcuvid.so.1
sudo make
```
Überprüfen ob alles geklappt hat:
```bash
cd /usr/local/cuda/samples/bin/x86_64/linux/release
./deviceQuery
```
Output sollte sein:
```bash
./deviceQuery Starting...

 CUDA Device Query (Runtime API) version (CUDART static linking)

Detected 1 CUDA Capable device(s)

Device 0: "GeForce GTX 1050 Ti"
  CUDA Driver Version / Runtime Version          9.1 / 9.0
  CUDA Capability Major/Minor version number:    6.1
  Total amount of global memory:                 4040 MBytes (4236312576 bytes)
  ( 6) Multiprocessors, (128) CUDA Cores/MP:     768 CUDA Cores
  GPU Max Clock rate:                            1620 MHz (1.62 GHz)
  Memory Clock rate:                             3504 Mhz
  Memory Bus Width:                              128-bit
  L2 Cache Size:                                 1048576 bytes
  Maximum Texture Dimension Size (x,y,z)         1D=(131072), 2D=(131072, 65536), 3D=(16384, 16384, 16384)
  Maximum Layered 1D Texture Size, (num) layers  1D=(32768), 2048 layers
  Maximum Layered 2D Texture Size, (num) layers  2D=(32768, 32768), 2048 layers
  Total amount of constant memory:               65536 bytes
  Total amount of shared memory per block:       49152 bytes
  Total number of registers available per block: 65536
  Warp size:                                     32
  Maximum number of threads per multiprocessor:  2048
  Maximum number of threads per block:           1024
  Max dimension size of a thread block (x,y,z): (1024, 1024, 64)
  Max dimension size of a grid size    (x,y,z): (2147483647, 65535, 65535)
  Maximum memory pitch:                          2147483647 bytes
  Texture alignment:                             512 bytes
  Concurrent copy and kernel execution:          Yes with 2 copy engine(s)
  Run time limit on kernels:                     Yes
  Integrated GPU sharing Host Memory:            No
  Support host page-locked memory mapping:       Yes
  Alignment requirement for Surfaces:            Yes
  Device has ECC support:                        Disabled
  Device supports Unified Addressing (UVA):      Yes
  Supports Cooperative Kernel Launch:            Yes
  Supports MultiDevice Co-op Kernel Launch:      Yes
  Device PCI Domain ID / Bus ID / location ID:   0 / 1 / 0
  Compute Mode:
     < Default (multiple host threads can use ::cudaSetDevice() with device simultaneously) >

deviceQuery, CUDA Driver = CUDART, CUDA Driver Version = 9.1, CUDA Runtime Version = 9.0, NumDevs = 1
Result = PASS
```
## Installation CuDNN

Go to the cuDNN download page (need registration) and select the latest cuDNN 7.0.* version made for CUDA 9.0.
Download all 3 .deb files: the runtime library, the developer library, and the code samples library for Ubuntu 16.04.
In your download folder, install them in the same order:

```bash
sudo dpkg -i libcudnn7_7.0.5.15–1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-dev_7.0.5.15–1+cuda9.0_amd64.deb
sudo dpkg -i libcudnn7-doc_7.0.5.15–1+cuda9.0_amd64.deb
cp -r /usr/src/cudnn_samples_v7/ ~/
cd ~/cudnn_samples_v7/mnistCUDNN
```

## Installation Spencer GroundHOG:
```bash
$cd cmr_social_guiding/spencer_people_tracking/detection/monocular_detectors/3rd_party/
$mkdir build
$cd build
$cmake ..
$make
```
Falls Fehler aufkommen, hier die Schritte die ausgeführt werden müssen:

- If there is an error "nvcc fatal : Value 'sm_11' is not defined for option 'gpu-architecture'", please make sure the CUDA SDK has been installed and use the command nvcc --help|grep "Allowed values for this option" -n to see which gpu architecture is supported (eg. 'compute_20' or 'sm_20'), and change 'sm_11' to others in the files (use searchmonkey)

- If there is an error "/usr/bin/ld: cannot find -lboost_program_options-mt", please make sure you have "libboost_program_options*." in your "/usr/lib" directory (use command locate libboost_program_options). If you have not yet installed Boost, you can try command sudo apt-get install libboost_program_options-dev. Else if you have "libboost_program_options.*", change 'boost_program_options-mt' to 'boost_program_options' in your Makefile and other files (use command grep 'boost_program_options-mt' -nr in the build dir to find these files)

- If there is an error "undefined reference to 'QString::fromAscii_helper" while running "make", edit "build/libcudaHOG/src/libcudaHOG/cudaHOG.pro" and remove the lines with the "cudaHOGDetect" and "cudaHOGDump" subdirs!

## Installation Face_Recognition mit GPU:

dlib mit CUDA installieren:

```bash
git clone https://github.com/davisking/dlib
cd dlib
python setup.py install
```
Zur Überprüfung:

```bash
python
import dlib
dlib.DLIB_USE_CUDA
```
Wenn die Meldung True kommt, hat man dlib mit CUDA installiert

Installation Face Reco:

```bash
pip intall face_recognition --user
pip3 install tensorflow-gpu --user
sudo apt-get install protobuf-compiler python-pil python-lxml python-tk
pip install --user Cython
pip install --user contextlib2
pip install --user jupyter
pip install --user matplotlib
```

## Installation opencv mit CUDA

```bash
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_extra.git
cd opencv
mkdir build
cd build

cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_EXAMPLES=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_CUDA=ON \
    -DWITH_GTK=ON \
    -DWITH_TBB=ON \
    -DWITH_CUDEV=ON \
    -DINSTALL_C_EXAMPLES=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -DOPENCV_ENABLE_NONFREE:BOOL=ON \
    -DOPENCV_TEST_DATA_PATH=../opencv_extra/testdata \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    ..

make
sudo make install
```
