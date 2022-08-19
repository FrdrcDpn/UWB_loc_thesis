# Bash script to run a simulation multiple times and save all logs in a folder
# This script expects swarmulator to have a kill-switch activated somewhere,
# although manual quitting (pressing the 'q' key) is also fine.
# $1 = number of trials
# $2 = number of agents

cd ..


for (( k = 1; k <= 10; k++ )); do
	p=$(($k*10))
	xmlstarlet ed -P -L\
	-u 'parameters/dynamic_beacons' -v 1 \
	-u 'parameters/UWB_frequency' -v $p \
    conf/parameters.xml 

	f=logs/cooltest
	mkdir $f

	d=logs/cooltest/batchtest_1$k
	mkdir $d
	for (( i = 1; i <= $1; i++ )); do
		
		# Bash text
		echo "Running trial $i out of $1 for a simulation with $2 agents"

		# Run code
		md=$(date +%Y-%m-%d-%T);
		ma=$(date +%Y-%m-%d-%T -d "+1 seconds") #backup time

		./swarmulator $2  
		
		sleep 1
		 # Give it some time to close

		# Move latest log to directory
		fn=$(ls -t logs| head -n1)
		if mv -f -- logs/log_$md.txt $d/log_$i.txt; then
			echo "Successfully moved file"
		else
			#if it doesn't work it's because it skipped a second
			echo "Trying again........!" 
			mv -f -- logs/log_$ma.txt $d/log_$i.txt
		fi

	done

done



for (( k = 1; k <= 10; k++ )); do
	p=$(($k*10))
	xmlstarlet ed -P -L\
	-u 'parameters/dynamic_beacons' -v 0 \
	-u 'parameters/UWB_frequency' -v $p \
    conf/parameters.xml 


	d=logs/cooltest/batchtest_0$k
	mkdir $d

	for (( i = 1; i <= $1; i++ )); do
		
		# Bash text
		echo "Running trial $i out of $1 for a simulation with $2 agents"

		# Run code
		md=$(date +%Y-%m-%d-%T);
		ma=$(date +%Y-%m-%d-%T -d "+1 seconds") #backup time

		./swarmulator $2  
		
		sleep 1
		 # Give it some time to close

		# Move latest log to directory
		fn=$(ls -t logs| head -n1)
		if mv -f -- logs/log_$md.txt $d/log_$i.txt; then
			echo "Successfully moved file"
		else
			#if it doesn't work it's because it skipped a second
			echo "Trying again........!" 
			mv -f -- logs/log_$ma.txt $d/log_$i.txt
		fi

	done

done





