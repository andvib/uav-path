#!/bin/bash

path=$1
resultpath=$2

cd build
make
cd ..

mkdir results

for i in {10..150..10};
do
    echo $i
    ./run $path $resultpath $i
done

exit 0
