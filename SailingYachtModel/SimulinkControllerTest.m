%MATLAB's [runtests] command fails for this test due to not being able to
%resolve the [par] global variable. Will fix this in this task: 
%https://ubcsailbot.atlassian.net/browse/BSC-37

MIN_ANGLE_DEVIATE_FROM_UP_WIND = pi/6;

%run the simulation first so global variables defined
%in the Simulink model loads
 sim('SailingYachtModel'); 

for desiredHeading = linspace(0,2*pi,20)
    % (par.alpha_w + pi) because wind blows from alpha_aw towards boat
    if isAngleBetween((par.alpha_w+pi)-MIN_ANGLE_DEVIATE_FROM_UP_WIND, ...
            (par.alpha_w+pi) + MIN_ANGLE_DEVIATE_FROM_UP_WIND, desiredHeading)
        continue;
    end
    disp("attempting to assert desiredheading = " + string(desiredHeading) + " radians is reached");
    sim('SailingYachtModel'); 
    assert(abs(finalHeadingErrorFromSim)<0.05);
end
