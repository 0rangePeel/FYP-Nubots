function [qstar] = invKinematics(q,param,model,desired,Cp)

q0 = q;

endEffectorTune = 1e4;
CoMTune = 1e3;
qAngleTune = 1e-1;


fun = @(Q) endEffectorTune*norm(endEffector(kinematics(Q,param,model),model) - desired) + CoMTune*norm(CoM(kinematics(Q,param,model),param,model,Q).*[1;0;0] - Cp) + qAngleTune*norm(q0 - Q);
options = optimoptions('fmincon',...
    'Algorithm','sqp',...
    'Display','none',... % use 'iter' to display optimisation diagnostic
    'FunctionTolerance',1e-3,...
    'OptimalityTolerance',1e-3,...
    'StepTolerance',1e-4,...
    'MaxFunctionEvaluations',1e5,...
    'MaxIterations',20);

lb = [   -pi/2;
         pi/720;
         -pi/2;
         -pi/2;
         -pi/2;
         -pi/2];

ub = [  pi/2;
        pi/2;
        pi/2;
        pi/2;
        -pi/720;
        pi/2];


qstar = fmincon(fun,q,[],[],[],[],lb,ub,[],options);

end