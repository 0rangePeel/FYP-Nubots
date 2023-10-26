function [base,eeB] = trajBase(tspan,control,param,model)
%TRAJBASE Summary of this function goes here
%   Detailed explanation goes here

    % Unpack
    base = model.base;
    P = param.P;
    CoM_star = model.CoM_star;
    stepSize = control.stepSize;
    stepNum = control.stepNum;
    timestep = control.timestep;
    tspan0 = tspan(1:2/timestep);
    tspan = tspan(2/timestep+1:end);

    stepTime = round(length(tspan)/stepNum);

    %% Create Base
    A = zeros(stepNum,stepTime);
    for i=2:stepNum+1
        A((i-1),:) = (-1)^i;
    end

    base = A(1,:);
    for i=2:stepNum
        base = [base A(i,:)];
    end

    %  Align CoM* length to tspan length
    while length(base) ~= length(tspan)
        base = [base base(:,end)];
    end

    % Left or Right foot start 
    if model.base == 1 % Right foot base
        % Already set up to accept right base first
        base = [ones(1,length(tspan0)) base];
    end
    if model.base == -1 % Left foot base
        base = [-ones(1,length(tspan0)) -base];
    end

    %% Create eeB
    eeB = [CoM_star(1,:);
           zeros(1,length(base));
           (param.P/2)*base];



end

