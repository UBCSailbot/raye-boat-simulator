%Where applicable, need to delete the following before running script:
% 1. ~/catkin_ws/src/matlab_gen
% 2. ~/catkin_ws/src/boat_controller

%replace with your catkin_ws/src folder path

folderpath = "~/catkin_ws/src/";
rosgenmsg(folderpath)
addpath(folderpath + "/matlab_gen/msggen")

%todo setup a test to see if can create sailbot message
%todo also verifiy if updating sailbot message updates atuomatically