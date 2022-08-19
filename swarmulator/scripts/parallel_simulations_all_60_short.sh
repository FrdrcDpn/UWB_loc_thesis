# Bash script to run parallel batches of simulations (it launches a batch every five seconds)
# $1: How many parallel simulations
# $2: How many agents in the simulation
cd ..
 
f=logs/3dplot
mkdir $f

e=logs/3dplot/dynamic
mkdir $e


for (( t = 8; t >= 2; --t )); do
  echo "starting batch with $t agents"
  g=logs/3dplot/dynamic/''$t''quad
  mkdir $g
  for (( e = 1; e <= 1; e++ )); do
  s=$(($e*50))

  g=logs/3dplot/dynamic/''$t''quad/''$s''statfreq
  mkdir $g

    xmlstarlet ed -P -L\
      -u 'parameters/UWB_frequency' -v $s \
      -u 'parameters/dynamic_beacons' -v 1 \
      -u 'parameters/beacon_1_en' -v 1 \
      -u 'parameters/beacon_2_en' -v 1 \
      -u 'parameters/beacon_3_en' -v 1 \
      -u 'parameters/beacon_4_en' -v 1 \
      -u 'parameters/beacon_6_en' -v 1 \
      -u 'parameters/beacon_8_en' -v 1 \
    conf/parameters.xml 

  for (( j = 1; j <= 10; j++ )); do
    p=$(($j*2))
    d=logs/3dplot/dynamic/''$t''quad/''$s''statfreq/dynfreq$p
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
wait 
done


for (( t = 1; t <= 1; t++ )); do
  echo "starting batch with $t agents"
  g=logs/3dplot/dynamic/''$t''quad
  mkdir $g

  for (( e = 1; e <= 1; e++ )); do
  s=$(($e*50))

  g=logs/3dplot/dynamic/''$t''quad/''$s''statfreq
  mkdir $g

    xmlstarlet ed -P -L\
      -u 'parameters/UWB_frequency' -v $s \
      -u 'parameters/dynamic_beacons' -v 0 \
      -u 'parameters/beacon_1_en' -v 1 \
      -u 'parameters/beacon_2_en' -v 1 \
      -u 'parameters/beacon_3_en' -v 1 \
      -u 'parameters/beacon_4_en' -v 1 \
      -u 'parameters/beacon_6_en' -v 1 \
      -u 'parameters/beacon_8_en' -v 1 \
    conf/parameters.xml 

  for (( j = 1; j <= 10; j++ )); do
    p=$(($j*2))
    d=logs/3dplot/dynamic/''$t''quad/''$s''statfreq/dynfreq$p
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
wait 
done
