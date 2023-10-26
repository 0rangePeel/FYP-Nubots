function [Output,CoM] = CoM(HT,param,model,q)
% Inputs Homogenous Transforms and Paramters and q
% Outputs Centre of Mass

%% Homogenous Transforms Setup

Rz = @(psi) [cos(psi) -sin(psi) 0 0;
             sin(psi)  cos(psi) 0 0;
                       0  0   1   0;
                       0  0   0   1];

HTf = @(X,Y,Z) [1 0 0 X;
                0 1 0 Y;
                0 0 1 Z;
                0 0 0 1];

CoMPy = 0.1;

if model.base == 1
    CoM.AbP = HT.Ab1*Rz(q(1))*HTf(0,CoMPy,-param.P/2);
    CoM.AbRA = HT.AbRB*HTf(-param.A/2,0,0);
    CoM.AbRT = HT.Ab3*Rz(q(3))*HTf(0,param.T/2,0);
    CoM.AbRF = HT.Ab2*Rz(q(2))*HTf(0,param.F/2,0);
    CoM.AbLF = HT.Ab4*Rz(q(4))*HTf(0,-param.F/2,0);
    CoM.AbLT = HT.Ab5*Rz(q(5))*HTf(0,-param.T/2,0);
    CoM.AbLA = HT.Ab6*Rz(q(6))*HTf(param.A/2,0,0);
elseif model.base == -1    
    CoM.AbP = HT.Ab4*Rz(-q(4))*HTf(0,CoMPy,param.P/2);
    CoM.AbLA = HT.AbLB*HTf(-param.A/2,0,0);
    CoM.AbLT = HT.Ab6*Rz(-q(6))*HTf(0,param.T/2,0);
    CoM.AbLF = HT.Ab5*Rz(-q(5))*HTf(0,param.F/2,0);
    CoM.AbRF = HT.Ab1*Rz(-q(1))*HTf(0,-param.F/2,0);
    CoM.AbRT = HT.Ab2*Rz(-q(2))*HTf(0,-param.T/2,0);
    CoM.AbRA = HT.Ab3*Rz(-q(3))*HTf(param.A/2,0,0);
else
    disp('ERROR-Incorrect Base Parameter')
end

CoM.Output = (CoM.AbP*param.mP + CoM.AbRA*param.mA + CoM.AbRT*param.mT + CoM.AbRF*param.mF + ...
              CoM.AbLF*param.mF + CoM.AbLT*param.mT + CoM.AbLA*param.mA)/(param.mP + ...
              2*param.mA + 2*param.mT + 2*param.mF);
Output = CoM.Output(1:3,4);

end