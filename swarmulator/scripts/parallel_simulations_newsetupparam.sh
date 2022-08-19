# Bash script to run parallel batches of simulations (it launches a batch every five seconds)
# $1: How many parallel simulations
# $2: How many agents in the simulation
cd ..
 
f=logs/3dplot
mkdir $f

e=logs/3dplot/dynamic
mkdir $e

for (( t = 4; t <= 4; t++ )); do
g=logs/3dplot/dynamic/''$t''quad
mkdir $g

  xmlstarlet ed -P -L\
    -u 'parameters/dynamic_beacons' -v 1 \
  	-u 'parameters/beacon_1_en' -v 1 \
    -u 'parameters/beacon_2_en' -v 1 \
    -u 'parameters/beacon_3_en' -v 1 \
    -u 'parameters/beacon_4_en' -v 1 \
    -u 'parameters/beacon_6_en' -v 1 \
    -u 'parameters/beacon_8_en' -v 1 \
   conf/parameters.xml 

for (( j = 0; j <= 11; j++ )); do
  p=$(($j*4))
  d=logs/3dplot/dynamic/''$t''quad/dynfreq$p
  mkdir $d

   xmlstarlet ed -P -L\
  	-u 'parameters/UWB_D_frequency' -v $p \
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

