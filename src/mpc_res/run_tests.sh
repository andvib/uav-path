#!/bin/bash

path=$1

cd build
make
cd ..

mkdir results

for i in {10..15};
do
    echo $i
    ./run $path $i
done

exit 0
