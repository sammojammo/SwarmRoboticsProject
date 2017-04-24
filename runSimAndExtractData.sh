#!/bin/bash

#Script to run the bioinst2 simulator a set number of times, specified in the command line arguments
#Arguments required are: the range of random seeds to be simulated for (lowend and highend), the feature depth, swarm behaviour and faulty robots behaviour

#check for correct arguments
if [ "$#" -eq "0" -o "$1" = "help" ]; then
    echo " "
    echo "Arguments: [seed range lowend] [seed range high end] [feature_depth] [swarmbehav] [errorbehav] [k] [lineq Threshold]"
    echo " "
    echo "swarmbehav can be: DISPERSION, AGGREGATION, FLOCKING, HOMING"
    echo "errorbehav can be: RNDWK, STOP, CIRCLE, STRLN"
    exit 0
fi

if [ "$#" -lt 7 -o "$#" -gt 7 ]; then
  echo "Incorrect arguments, try 'help' for usage"
exit 0
fi


num1=$1
num2=$2
#featuredepth is $3
#swarmbehav is $4
#errorbehav is $5
#get substrings for filename
swarmbehavShort=${4:0:4}
errorbehavShort=${5:0:5}

#make raw log directory if it doesnt exist
mkdir --parents /home/sam/Documents/FinalYearProject/DataLogs/RawLogs

#run simulations for range of seeds specified in arguments - output logs to RawLogs directory
for((i=num1; i<=num2; i++));
do
    /home/sam/simulator/bioinstsim2 -a sizex=50,sizey=50,resx=10,resy=10,help -e name=TEST,swarmbehav=${4},errorbehav=${5},misbehavestep=0,tracknormalagent=1,trackabnormalagent=15 -T maxspeed=0.1,count=20,fvsenserange=10,featuresenserange=6,bitflipprob=0.0,featuredepth=${3},numvotingnbrs=10,selectnumnearestnbrs=10 -M cross-affinity=${6},euclidean_threshold=${7},numberoffeatures=6 -s ${i} -n 10000 -z >"/home/sam/Documents/FinalYearProject/DataLogs/RawLogs/${3}depth_${swarmbehavShort}_${errorbehavShort}_seed${i}.log"

    echo "Simulation $i finished"
done

#run extract data script on raw logs
for((i=num1; i<=num2; i++));
do
    f="/home/sam/Documents/FinalYearProject/DataLogs/RawLogs/${3}depth_${swarmbehavShort}_${errorbehavShort}_seed${i}.log"
    cat $f | grep Id:  | awk  '{print $2 " " $4 " " $5 " " $6 " " $7 " " $8 " " $9 " " $11 " " $13 " " $15 " " $17}' | sed  's/,/ /g' > ${f}_extractedData

    echo "Log ${i} data extracted"
done

#make new directory if it doesnt exist
mkdir --parents /home/sam/Documents/FinalYearProject/DataLogs/ExtractedData/${swarmbehavShort}_${errorbehavShort}/

#move extracted log data to relevant directory
mv /home/sam/Documents/FinalYearProject/DataLogs/RawLogs/*.log_extractedData /home/sam/Documents/FinalYearProject/DataLogs/ExtractedData/${swarmbehavShort}_${errorbehavShort}/

