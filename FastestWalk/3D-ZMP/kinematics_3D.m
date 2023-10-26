function [HT] = kinematics_3D(q,param,model)
%% Homogenous Transforms Setup

Rx = @(psi) [1  0        0          0;
             0  cos(psi) -sin(psi)  0;
             0  sin(psi) cos(psi)   0;
             0  0        0          1];

Ry = @(psi) [cos(psi)  0  sin(psi)  0;
             0         1  0         0;
             -sin(psi) 0  cos(psi)  0;
             0         0  0         1];

Rz = @(psi) [cos(psi) -sin(psi)  0  0;
             sin(psi)  cos(psi)  0  0;
             0         0         1  0;
             0         0         0  1];

HTf = @(X,Y,Z) [1 0 0 X;
                0 1 0 Y;
                0 0 1 Z;
                0 0 0 1];

%% Static stationairy
if model.base == 1
    %disp('BASE RIGHT')
    % From right foot to pelvic girdle origin
    HT.AbRB = HTf(model.rRBb(1), model.rRBb(2), model.rRBb(3));
    HT.Ab6 = HT.AbRB*HTf(0,param.A,0);
    HT.Ab5 = HT.Ab6*Rx(q(6))*HTf(param.S,0,0);
    HT.Ab4 = HT.Ab5*Rz(q(5))*HTf(0,param.T,0);
    HT.Ab3 = HT.Ab4*Rz(q(4))*HTf(0,param.F,0);
    HT.Ab2 = HT.Ab3*Rz(q(3))*HTf(-param.S,0,0);
    HT.Ab1 = HT.Ab2*Rx(q(2))*HTf(0,param.S,0)*HTf(param.S/2,0,0);
    HT.Ab0 = HT.Ab1*Ry(q(1))*HTf(0,0,-param.P/2);

    HT.Ab7 = HT.Ab0*HTf(0,0,-param.P/2);
    HT.Ab8 = HT.Ab7*Ry(q(7))*HTf(0,-param.S,0)*HTf(-param.S/2,0,0);
    HT.Ab9 = HT.Ab8*Rx(q(8))*HTf(param.S,0,0);
    HT.Ab10 = HT.Ab9*Rz(q(9))*HTf(0,-param.F,0);
    HT.Ab11 = HT.Ab10*Rz(q(10))*HTf(0,-param.T,0);
    HT.Ab12 = HT.Ab11*Rz(q(11))*HTf(-param.S,0,0);
    HT.AbLB = HT.Ab12*Rx(q(12))*HTf(0,-param.A,0);

elseif model.base == -1
    %disp('BASE LEFT')
    % From left foot to pelvic girdle origin
    HT.AbLB = HTf(model.rLBb(1), model.rLBb(2), model.rLBb(3));
    HT.Ab12 = HT.AbLB*HTf(0,param.A,0);
    HT.Ab11 = HT.Ab12*Rx(-q(12))*HTf(param.S,0,0);
    HT.Ab10 = HT.Ab11*Rz(-q(11))*HTf(0,param.T,0);
    HT.Ab9 = HT.Ab10*Rz(-q(10))*HTf(0,param.F,0);
    HT.Ab8 = HT.Ab9*Rz(-q(9))*HTf(-param.S,0,0);
    HT.Ab7 = HT.Ab8*Rx(-q(8))*HTf(0,param.S,0)*HTf(param.S/2,0,0);
    HT.Ab0 = HT.Ab7*Ry(-q(7))*HTf(0,0,param.P/2);

    HT.Ab1 = HT.Ab0*HTf(0,0,param.P/2);
    HT.Ab2 = HT.Ab1*Ry(-q(1))*HTf(0,-param.S,0)*HTf(-param.S/2,0,0);
    HT.Ab3 = HT.Ab2*Rx(-q(2))*HTf(param.S,0,0);
    HT.Ab4 = HT.Ab3*Rz(-q(3))*HTf(0,-param.F,0);
    HT.Ab5 = HT.Ab4*Rz(-q(4))*HTf(0,-param.T,0);
    HT.Ab6 = HT.Ab5*Rz(-q(5))*HTf(-param.S,0,0);
    HT.AbRB = HT.Ab6*Rx(-q(6))*HTf(0,-param.A,0);

else
    disp('ERROR-Incorrect Base Parameter')
end

end



