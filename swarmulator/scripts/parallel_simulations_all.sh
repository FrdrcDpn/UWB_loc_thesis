# Bash script to run parallel batches of simulations (it launches a batch every five seconds)
# $1: How many parallel simulations
# $2: How many agents in the simulation
cd ..
 
f=logs/3dplot
mkdir $f

e=logs/3dplot/dynamic
mkdir $e


for (( t = 8; t >= 1; --t )); do
  echo "starting batch with $t agents"
  g=logs/3dplot/dynamic/''$t''quad
  mkdir $g
  for (( e = 1; e <= 5; e++ )); do
  s=$(($e*2))

  d=logs/3dplot/dynamic/''$t''quad/''$s''statfreq
  mkdir $d

    xmlstarlet ed -P -L\
      -u 'parameters/kR' -v $s \
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

