%replace with your catkin_ws/src folder path

catkinSRCFolderpath = "~/catkin_ws/src/";
system("git clone https://github.com/UBCSailbot/sailbot-msg.git " + catkinSRCFolderpath + "sailbot_msg");

rosgenmsg(catkinSRCFolderpath)
addpath(catkinSRCFolderpath + "/matlab_gen/msggen")


