#!/bin/bash

path1=$1
res1=$2
path2=$3
res2=$4

cd build
make
cd ..

mkdir results
cd results
mkdir 150m
mkdir 200m
cd ..

#for i in {10..140..10};
#do
#    echo $i
#    ./run $path1 $res1 $i
#done

for i in {10..140..10};
do
    echo $i
    ./run $path2 $res2 $i
done

exit 0
