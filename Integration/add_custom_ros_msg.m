%replace with your catkin_ws/src folder path

%note you likely have to run this script
%each time you make an update to saibot custom message

catkinSRCFolderpath = "~/catkin_ws/src/";
system("git clone https://github.com/UBCSailbot/sailbot-msg.git " + catkinSRCFolderpath + "sailbot_msg");

rosgenmsg(catkinSRCFolderpath)
addpath(catkinSRCFolderpath + "/matlab_gen/msggen")


