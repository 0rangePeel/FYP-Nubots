function [HT] = kinematics(q,param,model)
%% Homogenous Transforms Setup

Rz = @(psi) [cos(psi) -sin(psi) 0 0;
             sin(psi)  cos(psi) 0 0;
                       0  0   1   0;
                       0  0   0   1];

HTf = @(X,Y,Z) [1 0 0 X;
                0 1 0 Y;
                0 0 1 Z;
                0 0 0 1];

%% Static stationairy
if model.base == 1
    %disp('BASE RIGHT')
    % From right foot to pelvic girdle
    HT.AbRB = HTf(model.rRBb(1), model.rRBb(2), model.rRBb(3));
    HT.Ab3 = HT.AbRB*HTf(-param.A,0,0);
    HT.Ab2 = HT.Ab3*Rz(q(3))*HTf(0,param.T,0);
    HT.Ab1 = HT.Ab2*Rz(q(2))*HTf(0,param.F,0);

    % Pelvic girdle
    HT.Ab0 = HT.Ab1*Rz(q(1))*HTf(0,0,-param.P/2);

    % From pelvic girdle to left foot
    HT.Ab4 = HT.Ab0*HTf(0,0,-param.P/2);
    HT.Ab5 = HT.Ab4*Rz(q(4))*HTf(0,-param.F,0);
    HT.Ab6 = HT.Ab5*Rz(q(5))*HTf(0,-param.T,0);
    HT.AbLB = HT.Ab6*Rz(q(6))*HTf(param.A,0,0);
    
elseif model.base == -1
    %disp('BASE LEFT')
    % From left foot to pelvic girdle
    HT.AbLB = HTf(model.rLBb(1), model.rLBb(2), model.rLBb(3));
    HT.Ab6 = HT.AbLB*HTf(-param.A,0,0);
    HT.Ab5 = HT.Ab6*Rz(-q(6))*HTf(0,param.T,0);
    HT.Ab4 = HT.Ab5*Rz(-q(5))*HTf(0,param.F,0);

    % Pelvic girdle
    HT.Ab0 = HT.Ab4*Rz(-q(4))*HTf(0,0,param.P/2);

    % From pelvic girdle to right foot
    HT.Ab1 = HT.Ab0*HTf(0,0,param.P/2);
    HT.Ab2 = HT.Ab1*Rz(-q(1))*HTf(0,-param.F,0);
    HT.Ab3 = HT.Ab2*Rz(-q(2))*HTf(0,-param.T,0);
    HT.AbRB = HT.Ab3*Rz(-q(3))*HTf(param.A,0,0);

else
    disp('ERROR-Incorrect Base Parameter')
end
end

