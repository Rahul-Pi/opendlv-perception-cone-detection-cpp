# opendlv-perception-cone-detection-cpp

How to get the program running:

################################### Part A: ##############################################

This is to remove any images and containers that can create a conflict with the program.

Caution: This will wipe out all the containers so do it only if it is necessary.

1. Stop all running containers using:     docker stop $(docker ps -a -q)
2. Clear all containers:                  docker rm $(docker ps -a -q)
3. Remove all images:                     docker rmi -f $(docker images -a -q)


################################### Part B: ##############################################

This is to get all the files needed to the local system.

1. Pull the code to your PC/Laptop using
   git clone https://github.com/Rahul-Pi/opendlv-perception-cone-detection-cpp.git

################################### Part C: ##############################################

This is to build the docker from the files.

1. Open a terminal in the folder "opendlv-perception-cone-detection-cpp" and run the following command:
   docker build -t opendlv-perception-cone-detection-cpp .


################################### Part D: ##############################################

This part is to run the file and record the output.

1. Open a terminal in the folder "opendlv-perception-cone-detection-cpp" and run the following code to start the replay viewer.
   docker-compose -f h264-replay-viewer.yml up
2. Launch another terminal in the same folder "opendlv-perception-cone-detection-cpp" and run the following code to start the cone detection.
   docker-compose -f simulate-cone-detection.yml up
