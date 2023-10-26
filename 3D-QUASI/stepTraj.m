function [xe_n,CoM_star] = stepTraj(xe,dS,CoM,tf,step,param,model)

dP = 0.1;

x = [zeros(1,tf*0.4) linspace(0,dS,tf*0.5) dS*ones(1,tf*0.1)];

y = (2*dP*x)/dS - (2*dP*x.^2)/dS^2;

footOffsetZ = 0.000;
footOffsetX = 0.028; % increasing this seems to work better


if model.base == 1
    % Right foot base
    xeZ = model.rLBb(3)*ones(1,tf);
    CoM_starZ = [linspace(CoM(3),model.rRBb(3) - footOffsetZ,tf*0.4) (model.rRBb(3) - footOffsetZ)*ones(1,tf*0.6)];
    CoM_starX = [linspace(CoM(1),model.rRBb(1) + footOffsetX,tf*0.4) (model.rRBb(1) + footOffsetX)*ones(1,tf*0.6)];
elseif model.base == -1 
    % Left foot base
    xeZ = model.rRBb(3)*ones(1,tf);
    CoM_starZ = [linspace(CoM(3),model.rLBb(3) + footOffsetZ,tf*0.4) (model.rLBb(3) + footOffsetZ)*ones(1,tf*0.6)];
    CoM_starX = [linspace(CoM(1),model.rLBb(1) + footOffsetX,tf*0.4) (model.rLBb(1) + footOffsetX)*ones(1,tf*0.6)];

else
    disp('ERROR-Incorrect Base Parameter')
end

if step == 0
    if model.base == 1
        % Right foot base
        %xeZ = (model.rLBb(3))*ones(1,tf);
        CoM_starZ = [linspace(CoM(3),model.rRBb(3) - footOffsetZ,tf*0.4) (model.rRBb(3) - footOffsetZ)*ones(1,tf*0.6)];
    elseif model.base == -1 
        % Left foor base
        %xeZ = (model.rRBb(3))*ones(1,tf);
        CoM_starZ = [linspace(CoM(3),model.rLBb(3) + footOffsetZ,tf*0.4) (model.rLBb(3) + footOffsetZ)*ones(1,tf*0.6)];
    else
        disp('ERROR-Incorrect Base Parameter')
    end
end

xe_n = [x + xe(1);y;xeZ;zeros(2,tf);xe(6)*ones(1,tf)];
CoM_star = [CoM_starX;CoM(2)*ones(1,tf);CoM_starZ];

end

