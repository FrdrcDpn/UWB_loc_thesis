# Bash script to run a simulation multiple times and save all logs in a folder
# This script expects swarmulator to have a kill-switch activated somewhere,
# although manual quitting (pressing the 'q' key) is also fine.
# $1 = number of trials
# $2 = number of agents

cd ..

# Run code
md=$(date +%Y-%m-%d-%T);
ma=$(date +%Y-%m-%d-%T -d "+1 seconds") #backup time
# Bash text
		echo "STARTING ANOTHA ONE"
./swarmulator $1
	
sleep 1 # Give it some time to close

# Move latest log to directory
fn=$(ls -t logs| head -n1)
if mv -f -- logs/log_$md.txt $3/log_$2.txt; then
	echo "Successfully moved file"
else
	#if it doesn't work it's because it skipped a second
	echo "Trying again........!" 
	mv -f -- logs/log_$ma.txt $3/log_$2.txt

fi




