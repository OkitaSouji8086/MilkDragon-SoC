#!/bin/bash

cur_path=$(pwd)
export PRJ_PATH=$cur_path

init(){
    cd "$PRJ_PATH/sdk/toolchains"
    ./init.sh
    source ~/.bashrc
}
rduv(){
    cd "$PRJ_PATH/rtl/$1" && ls
}
dcase(){
    cd "$PRJ_PATH/sdk/software/examples/$1" && ls
}
prj(){
    cd "$PRJ_PATH" && ls
}
comp(){
    cd "$PRJ_PATH/sdk/software/examples/$1" && make clean && make
}