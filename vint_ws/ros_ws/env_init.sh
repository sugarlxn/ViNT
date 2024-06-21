# make 
# attach to container vint2
# activate conda env

# docker exec -it vint2 /bin/bash
conda activate nomad_train
echo "conda activate nomad_train"
export DISPLAY=192.168.100.104:0.0
#export DISPLAY=192.168.100.223:0.0
echo $DISPLAY
cd ~/vint_ws/ros_ws
pwd
