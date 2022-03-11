function [lateralError, headingError] = fcn(currPose, refPose, wheelbase)

%   Copyright 2018 The MathWorks, Inc.

currPose(:,3) = deg2rad(currPose(:,3));
refPose(:,3)  = deg2rad(refPose(:,3));

%Convert current pose to front wheel coordinate
currPoseF = coder.nullcopy(zeros(1, 3));

currPoseF = driving.internal.control.rearPoseToFrontPose(currPose, wheelbase);

lateralError = norm(currPoseF(1:2) - refPose(1:2));

headingError = rad2deg(currPose(:, 3) - refPose(:,3)); 