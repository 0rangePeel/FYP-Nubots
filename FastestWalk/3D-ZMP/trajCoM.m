function [CoM_star] = trajCoM(tspan,control,param,model,CoM)
%TRAJCOM Summary of this function goes here
%   Detailed explanation goes here

    % Unpack
    base = model.base;
    zc = param.zc;
    stepSize = control.stepSize;
    stepNum = control.stepNum;
    timestep = control.timestep;
    tspan0 = tspan(1:2/timestep);
    tspan = tspan(2/timestep+1:end);

    stepTime = round(length(tspan)/stepNum);

    %% Create CoMX*
    A = zeros(stepNum,stepTime);
    for i=1:stepNum
        A(i,:) = (stepSize)*(i - 1);
    end

    CoM_starX = A(1,:);
    for i=2:stepNum
        CoM_starX = [CoM_starX A(i,:)];
    end

    %  Align CoMX* length to tspan length
    while length(CoM_starX) ~= length(tspan)
        CoM_starX = [CoM_starX CoM_starX(:,end)];
    end

    %% Create CoMZ*

    % Right foot base
    if base == 1
        A = zeros(stepNum,stepTime);
        for i=2:stepNum+1
            A(i-1,:) = (param.P - 0.02)*(-1)^i;
        end
    else %Left foot base 
        A = zeros(stepNum,stepTime);
        for i=1:stepNum
            A(i,:) = (param.P - 0.02)*(-1)^i;
        end
    end

    CoM_starZ = A(1,:);
    for i=2:stepNum
        CoM_starZ = [CoM_starZ A(i,:)];
    end

    %  Align CoMZ* length to tspan length
    while length(CoM_starZ) ~= length(tspan)
        CoM_starZ = [CoM_starZ CoM_starZ(:,end)];
    end

    %% Create CoMY*
    CoM_starY = [zc*ones(1,length(tspan0)) zc*ones(1,length(tspan))];

    %% Create CoM*
%     CoM_star = [CoM(1) + 0.015 + zeros(1,length(tspan0)) CoM(1) + 0.015 + CoM_starX;
%                 CoM_starY;
%                 CoM(3) - 0.025 + zeros(1,length(tspan0)) CoM(3) - 0.01 + CoM_starZ];%0.01
%     CoM_star = [CoM(1) + 0.015 + zeros(1,length(tspan0)) CoM(1) + 0.015 + CoM_starX;
%                 CoM_starY;
%                 CoM(3) - 0.025 + zeros(1,length(tspan0)) CoM(3) - 0.01 + CoM_starZ];%0.01
%     CoM_star = [CoM(1) + 0.025 + zeros(1,length(tspan0)) CoM(1) + 0.025 + CoM_starX;
%                 CoM_starY;
%                 CoM(3) - 0.025 + zeros(1,length(tspan0)) CoM(3) - 0.01 + CoM_starZ];%0.01
%     CoM_star = [CoM(1) + 0.005 + zeros(1,length(tspan0)) CoM(1) + 0.005 + CoM_starX;
%                 CoM_starY;
%                 CoM(3) - 0.025 + zeros(1,length(tspan0)) CoM(3) - 0.00 + CoM_starZ];%0.01
% THIS WORKS WELL AT 0.020 ms and 0.1 dp and at all step size
%     CoM_star = [CoM(1) + 0.015 + zeros(1,length(tspan0)) CoM(1) + 0.015 + CoM_starX; 
%                 CoM_starY;
%                 CoM(3) - 0.025 + zeros(1,length(tspan0)) CoM(3) - 0.00 + CoM_starZ];%0.01
    CoM_star = [CoM(1) + 0.015 + zeros(1,length(tspan0)) CoM(1) + 0.015 + CoM_starX; 
                CoM_starY;
                CoM(3) - 0.00 + zeros(1,length(tspan0)) CoM(3) - 0.00 + CoM_starZ];%0.01

end

