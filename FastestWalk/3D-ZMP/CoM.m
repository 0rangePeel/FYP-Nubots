function [FinalCoM,CoM] = CoM(HT,param,model,q)
% Inputs Homogenous Transforms and Paramters and q
% Outputs Centre of Mass

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

if model.base == 1
    
    CoM.Ab01 = HT.Ab0*HTf(0,param.PH,param.P/4);
    CoM.Ab12 = HT.Ab1*HTf(0,-param.S/2,0);
    CoM.Ab23 = HT.Ab2*HTf(param.S/2,0,0);
    CoM.Ab34 = HT.Ab3*HTf(0,-param.F/2,0);
    CoM.Ab45 = HT.Ab4*HTf(0,-param.T/2,0);
    CoM.Ab56 = HT.Ab5*HTf(-param.S/2,0,0);
    CoM.Ab6RB = HT.Ab6*HTf(0,-param.A/2,0);

    CoM.Ab07 = HT.Ab0*HTf(0,param.PH,-param.P/4);
    CoM.Ab78 = HT.Ab7*Rz(q(7))*HTf(0,-param.S/2,0);
    CoM.Ab89 = HT.Ab8*Rz(q(8))*HTf(param.S/2,0,0);
    CoM.Ab910 = HT.Ab9*Rz(q(9))*HTf(0,-param.F/2,0);
    CoM.Ab1011 = HT.Ab10*Rz(q(10))*HTf(0,-param.T/2,0);
    CoM.Ab1112 = HT.Ab11*Rz(q(11))*HTf(-param.S/2,0,0);
    CoM.Ab12LB = HT.Ab12*Rx(q(12))*HTf(0,-param.A/2,0);

elseif model.base == -1    

    CoM.Ab01 = HT.Ab0*HTf(0,param.PH,param.P/4);
    CoM.Ab12 = HT.Ab1*Rz(-q(1))*HTf(0,-param.S/2,0);
    CoM.Ab23 = HT.Ab2*Rz(-q(2))*HTf(param.S/2,0,0);
    CoM.Ab34 = HT.Ab3*Rz(-q(3))*HTf(0,-param.F/2,0);
    CoM.Ab45 = HT.Ab4*Rz(-q(4))*HTf(0,-param.T/2,0);
    CoM.Ab56 = HT.Ab5*Rz(-q(5))*HTf(-param.S/2,0,0);
    CoM.Ab6RB = HT.Ab6*Rx(-q(6))*HTf(0,-param.A/2,0);

    CoM.Ab07 = HT.Ab0*HTf(0,param.PH,-param.P/4);
    CoM.Ab78 = HT.Ab7*HTf(0,-param.S/2,0);
    CoM.Ab89 = HT.Ab8*HTf(param.S/2,0,0);
    CoM.Ab910 = HT.Ab9*HTf(0,-param.F/2,0);
    CoM.Ab1011 = HT.Ab10*HTf(0,-param.T/2,0);
    CoM.Ab1112 = HT.Ab11*HTf(-param.S/2,0,0);
    CoM.Ab12LB = HT.Ab12*HTf(0,-param.A/2,0);

else
    disp('ERROR-Incorrect Base Parameter')
end



massTotal = (2*param.mP + 2*param.mF + 2*param.mT + 8*param.mS);

CoM.Output = (CoM.Ab01*param.mP + CoM.Ab07*param.mP + ...
              CoM.Ab12*param.mS + CoM.Ab78*param.mS + ...
              CoM.Ab23*param.mS + CoM.Ab89*param.mS + ...
              CoM.Ab34*param.mF + CoM.Ab910*param.mF + ...
              CoM.Ab45*param.mT + CoM.Ab1011*param.mT + ...
              CoM.Ab56*param.mS + CoM.Ab1112*param.mS + ...
              CoM.Ab6RB*param.mS + CoM.Ab12LB*param.mS)/massTotal;

FinalCoM = CoM.Output(1:3,4);

end