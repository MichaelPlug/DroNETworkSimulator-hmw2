#-----------------------------------------------------------#
#           _  _ ___  ___  ___  _  _ ___ ___                #
#          | \| |   \| _ \/ _ \| \| | __/ __|               #
#          | .` | |) |   / (_) | .` | _|\__ \               #
#          |_|\_|___/|_|_\\___/|_|\_|___|___/               #
#                                                           #
#-----------------------------------------------------------#

#test baselines
for nd in "35" "40";
do
    for alg in "AI";
    # if you experienced too much time to run experiments, remove "GEO" and "RND"
    do 
        echo "run: ${alg} - ndrones ${nd} "
        #python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 1 -e_s 10 -alg ${alg} &
        #python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        #python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
    done;
done; 
wait

#Questa riga di codie serve a rinominare i file che vengono creati, AI viene sostituito da GWN_TMG_GRA nell'esempio 
cd data/experiments
#for f in *AI.json; do mv "$f" "$(echo "$f" | sed s/AI/GRID/)"; done
cd ../..
#for f in *AISG_IP*; do mv "$f" "$(echo "$f" | sed s/AISG_IP/AISG_IP/)"; done
python3 -m src.experiments.json_and_plot -nd 2 -nd 5 -nd 10 -nd 15 -nd 20 -nd 25 -nd 30 -nd 35 -nd 40 -i_s 1 -e_s 30 -exp_suffix AISG_UP -exp_suffix MGEO 


#for f in *AI.json; do mv "$f" "$(echo "$f" | sed s/AI/3A_TMGEO/)"; done
