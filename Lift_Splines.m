%Generation of Sail lift and drag coefficients cubic spline
%Nicolas Navarre
%UBC Sailbot
%Jan 2019


%Adding path to run plospline2 from Math 307
addpath '/Users/Nicolas/Documents/Courses/3rd Year/MATH 307/Homework'


lift_angles_deg = [27 50 80 100 180];
lift_angles_rad = lift_angles_deg.*pi./180;
jib_lift = [1.5 0.5 0.3 0 0];
main_lift = [1.5 1.5 0.95 0.85 0];

plotspline2(lift_angles_deg,main_lift)
hold on
plotspline2(lift_angles_deg,jib_lift)