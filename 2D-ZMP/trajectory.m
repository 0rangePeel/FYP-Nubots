function [CoM_star,eeL,eeR,ee,base,eeB] = trajectory(tspan,control,param,model,xe)
    % Trajectory Generation
    %   Generates the CoM* reference to be followed and end effector
    %   position to be used in inverse kinematics and position of base
    
    % Unpack
    stepSize = control.stepSize;
    stepNum = control.stepNum;
    
    stepTime = round(length(tspan)/stepNum);

    A = zeros(stepNum,stepTime);

    %% Create CoM*
    for i=1:stepNum
        A(i,:) = (stepSize)*(i - 1) - (control.stepSize/2);
    end

    CoM_star = A(1,:);
    for i=2:stepNum
        CoM_star = [CoM_star A(i,:)];
    end

    %  Align CoM* length to tspan length
    while length(CoM_star) ~= length(tspan)
        CoM_star = [CoM_star CoM_star(:,end)];
    end

    %% Create End Effector pose
    % First Step - Half Step
    stepSize = stepSize*2;
    x = [linspace(0,stepSize/2,stepTime)];
    y = (2*stepSize*x)/(stepSize/2) - (2*stepSize*x.^2)/(stepSize/2)^2;

    % Start construction
    eeL = [x;y];
    eeR = [zeros(1,stepTime);zeros(1,stepTime)];

    % Initialise end effector array
    ee = eeL;
    
    % Full step
    x = [linspace(0,stepSize,stepTime)];
    y = (2*stepSize*x)/(stepSize) - (2*stepSize*x.^2)/(stepSize)^2;

    if mod(stepNum,2) == 1 % Odd number of steps
        for i=1:(idivide(int16(stepNum),2,'floor'))
            temp = [x + eeL(1,end);y];
            eeL = [eeL [zeros(1,stepTime) + eeL(1,end);zeros(1,stepTime)] temp];    
        end
        for i=1:(idivide(int16(stepNum),2,'floor'))
            temp = [x + eeR(1,end);y];
            eeR = [eeR temp [zeros(1,stepTime) + temp(1,end);zeros(1,stepTime)]];    
        end

        % Create the array which is used for inverse kinematics 
        % Not used for plotting purposes
        for i=1:(idivide(int16(stepNum),2,'floor'))
            ee = [ee eeR(:,((i-1)*2 + 1)*stepTime+1:2*i*stepTime) eeL(:,2*i*stepTime+1:(i*2 + 1)*stepTime)];
        end
    else % Even Number of steps
        for i=1:(idivide(int16(stepNum),2,'floor') - 1)
            temp = [x + eeL(1,end);y];
            eeL = [eeL [zeros(1,stepTime) + eeL(1,end);zeros(1,stepTime)] temp];    
        end

        eeL = [eeL [zeros(1,stepTime) + eeL(1,end);zeros(1,stepTime)]];

        for i=1:(idivide(int16(stepNum),2,'floor') - 1)
            temp = [x + eeR(1,end);y];
            eeR = [eeR temp [zeros(1,stepTime) + temp(1,end);zeros(1,stepTime)]];    
        end
        eeR = [eeR [x + eeR(1,end);y]];

        % Create the array which is used for inverse kinematics 
        % Not used for plotting purposes
        ee = [ee eeR(:,stepTime+1:2*stepTime)];
        for i=1:(idivide(int16(stepNum),2,'floor'))-1
            ee = [ee  eeL(:,2*i*stepTime+1:(i*2 + 1)*stepTime) eeR(:,(i*2 + 1)*stepTime+1:2*(i+1)*stepTime)];
        end
    end

    %% Create Base
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
    end
    if model.base == -1 % Left foot base
        A = eeR;
        eeR = eeL;
        eeL = A;

        base = -base;

    end

    %% Create eeB
    eeB = [CoM_star + param.A/2;
           zeros(1,length(tspan));
           (param.P/2)*base];

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
end

