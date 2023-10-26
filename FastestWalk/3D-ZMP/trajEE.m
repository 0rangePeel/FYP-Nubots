function [eeL,eeR,ee] = trajEE(tspan,control,param,model)
%TRAJEE Summary of this function goes here
%   Detailed explanation goes here

    % Unpack
    base = model.base;
    stepSize = control.stepSize;
    stepNum = control.stepNum;
    timestep = control.timestep;
    tspan0 = tspan(1:2/timestep);
    tspan = tspan(2/timestep+1:end);

    stepTime = round(length(tspan)/stepNum);

    %% Create End Effector pose
    % First Step - Half Step
    stepSize = stepSize*2;
    x = [linspace(0,stepSize/2,stepTime)];
%     dP = 0.1;
    % dP = 2*0.085;
    dP = 0.1;
    y = (2*dP*x)/(stepSize/2) - (2*dP*x.^2)/(stepSize/2)^2;

    z = [model.rLBb(3)*ones(1,stepTime);
         model.rRBb(3)*ones(1,stepTime)];

    % Start construction
    eeL = [x;y;z(2,:)];
    eeR = [zeros(1,stepTime);zeros(1,stepTime);z(1,:)];

    % Initialise end effector array
    ee = eeL;
    
    % Full step
    x = [linspace(0,stepSize,stepTime)];
    % y = (2*stepSize*x)/(stepSize) - (2*stepSize*x.^2)/(stepSize)^2;
    y = (2*dP*x)/stepSize - (2*dP*x.^2)/stepSize^2;

    if mod(stepNum,2) == 1 % Odd number of steps
        for i=1:(idivide(int16(stepNum),2,'floor'))
            temp = [x + eeL(1,end);y;z(2,:)];
            eeL = [eeL [zeros(1,stepTime) + eeL(1,end);zeros(1,stepTime);z(2,:)] temp];    
        end
        for i=1:(idivide(int16(stepNum),2,'floor'))
            temp = [x + eeR(1,end);y;z(1,:)];
            eeR = [eeR temp [zeros(1,stepTime) + temp(1,end);zeros(1,stepTime);z(1,:)]];    
        end

        % Create the array which is used for inverse kinematics 
        % Not used for plotting purposes
        for i=1:(idivide(int16(stepNum),2,'floor'))
            ee = [ee eeR(:,((i-1)*2 + 1)*stepTime+1:2*i*stepTime) eeL(:,2*i*stepTime+1:(i*2 + 1)*stepTime)];
        end
    else % Even Number of steps
        for i=1:(idivide(int16(stepNum),2,'floor') - 1)
            temp = [x + eeL(1,end);y;z(2,:)];
            eeL = [eeL [zeros(1,stepTime) + eeL(1,end);zeros(1,stepTime);z(2,:)] temp];    
        end

        eeL = [eeL [zeros(1,stepTime) + eeL(1,end);zeros(1,stepTime)]];

        for i=1:(idivide(int16(stepNum),2,'floor') - 1)
            temp = [x + eeR(1,end);y;z(1,:)];
            eeR = [eeR temp [zeros(1,stepTime) + temp(1,end);zeros(1,stepTime);z(1,:)]];    
        end
        eeR = [eeR [x + eeR(1,end);y;z(1,:)]];

        % Create the array which is used for inverse kinematics 
        % Not used for plotting purposes
        ee = [ee eeR(:,stepTime+1:2*stepTime)];
        for i=1:(idivide(int16(stepNum),2,'floor'))-1
            ee = [ee  eeL(:,2*i*stepTime+1:(i*2 + 1)*stepTime) eeR(:,(i*2 + 1)*stepTime+1:2*(i+1)*stepTime)];
        end
    end  

    % Left or Right foot start 
    if model.base == 1 % Right foot base
        % Already set up to accept right base first
    end
    if model.base == -1 % Left foot base
        A = eeR;
        eeR = eeL;
        eeL = A;
    end

    % Make tspan the same size as eeL and eeR
    while length(eeR) ~= length(tspan)
        eeR = [eeR eeR(:,end)];
    end
    while length(eeL) ~= length(tspan)
        eeL = [eeL eeL(:,end)];
    end  
    while length(ee) ~= length(tspan)
        ee = [ee ee(:,end)];
    end

    %% Create Final end effector
    tempR = [zeros(2,length(tspan0));model.rRBb(3)*ones(1,length(tspan0))];
    tempL = [zeros(2,length(tspan0));model.rRBb(3)*ones(1,length(tspan0))];
    eeR = [tempR eeR];
    eeL = [tempL eeL];
    % Right foot base
    if base == 1
        ee = [tempL ee];
    else % Left foot base
        ee = [tempR ee];
    end

end

