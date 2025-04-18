#!/bin/bash

wget https://gitee.com/loongson-edu/la32r-toolchains/releases/download/v0.0.3/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0.tar.xz
tar Jxvf loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0.tar.xz
mkdir picolibc
cd picolibc
wget https://gitee.com/ffshff/la32r-picolibc/releases/download/V1.0/picolibc.tar.gz
tar zxvf picolibc.tar.gz
cd ..
mkdir newlib
cd newlib
wget https://gitee.com/ffshff/newlib-la32r/releases/download/V1.0/newlib.tar.gz
tar zxvf newlib.tar.gz
cd ..

current_dir=$(pwd)
echo "export PATH=$current_dir/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:\$PATH" >> ~/.bashrc
echo "export CICIEC_WINDOWS_HOME=$PRJ_PATH" >> ~/.bashrc
source ~/.bashrc
