#!/bin/bash          
WORKING_DIR=temp
OUTPUT=output

#create working directory
if [[ -d $WORKING_DIR ]]
then
   echo "Directory '$WORKING_DIR' already exists"
   exit
else
   mkdir $WORKING_DIR
fi

# create symbolic links
TIME=0
for IMAGE in $(ls *.png | sort -n)
do
   IMAGE_TIMESTAMP=$(echo $IMAGE | cut -c 4-9) 
   while [ $TIME -lt $IMAGE_TIMESTAMP ]
   do
      ln -s ../$IMAGE $WORKING_DIR/$(echo $TIME | awk -F, '{ printf "%05d\n", $1 }').png
      TIME=$(bc <<< $TIME+1)
   done
done

# create the sequence
avconv -r 100 -f image2 -i $WORKING_DIR/%05d.png -c:v h264 -tune stillimage -crf 0 $OUTPUT.mp4

# remove working directory
rm -r temp




