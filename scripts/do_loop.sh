#!/bin/sh
  
trap "kill 0" 2

for i in `seq 7`
do
echo $i
echo "next loop..."
./loop.sh
done

echo "all loop end"














