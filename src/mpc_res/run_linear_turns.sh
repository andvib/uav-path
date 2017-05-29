#!/bin/bash

#path=$1
#res=$2

cd build
make
cd ..

mkdir results_turns

#for filename in path/*.txt;
#do
#    echo $filename
#    NUMLINES=$(wc -l < $filename);
#    echo $NUMLINES    
#    ./run $filename $filename $NUMLINES
#done

#FILENAME="path/lin_45deg.txt";
#NUMLINES=$(wc -l < $FILENAME);
#echo $NUMLINES
#./run $FILENAME $FILENAME $NUMLINES
#
#exit 0

FILENAME="path/lin_90deg.txt";
RESNAME="lin_90deg";
NUMLINES=$(wc -l < $FILENAME);
INTERVALS=60;
./run $FILENAME $RESNAME $NUMLINES $INTERVALS

