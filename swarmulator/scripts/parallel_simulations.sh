# Bash script to run parallel batches of simulations (it launches a batch every five seconds)
# $1: How many parallel simulations
# $2: How many agents in the simulation
cd ..
 
f=logs/3dplot
mkdir $f

e=logs/3dplot/dynamic
mkdir $e

for (( t = 1; t <= 5; t++ )); do
g=logs/3dplot/dynamic/''$t''quad
mkdir $g

  xmlstarlet ed -P -L\
    -u 'parameters/dynamic_beacons' -v 1 \
  	-u 'parameters/beacon_1_en' -v 0 \
    -u 'parameters/beacon_2_en' -v 0 \
    -u 'parameters/beacon_3_en' -v 0 \
    -u 'parameters/beacon_4_en' -v 0 \
   conf/parameters.xml 

# change number of beacons 0 - 4
for (( j = 1; j <= 4; j++ )); do

  d=logs/3dplot/dynamic/''$t''quad/beacon$j
  mkdir $d

   xmlstarlet ed -P -L\
  	-u 'parameters/beacon_'$j'_en' -v 1 \
   conf/parameters.xml 

   cd scripts 

   for (( i = 1; i <= $1; i++ )); do
    ./run_n_simulations_1.sh $t $i $d & sleep 5
   done
   wait
   cd ..
  done
 wait
done



e=logs/3dplot/static
mkdir $e

for (( t = 1; t <= 1; t++ )); do
g=logs/3dplot/static/''$t''quad
mkdir $g

  xmlstarlet ed -P -L\
    -u 'parameters/dynamic_beacons' -v 0 \
  	-u 'parameters/beacon_1_en' -v 0 \
    -u 'parameters/beacon_2_en' -v 0 \
    -u 'parameters/beacon_3_en' -v 0 \
    -u 'parameters/beacon_4_en' -v 0 \
   conf/parameters.xml 

# change number of beacons 0 - 4
for (( j = 1; j <= 4; j++ )); do

  d=logs/3dplot/static/''$t''quad/beacon$j
  mkdir $d

   xmlstarlet ed -P -L\
  	-u 'parameters/beacon_'$j'_en' -v 1 \
   conf/parameters.xml 

   cd scripts 

   for (( i = 1; i <= $1; i++ )); do
    ./run_n_simulations_1.sh $t $i $d & sleep 5
   done
   wait
   cd ..
  done
 wait
done