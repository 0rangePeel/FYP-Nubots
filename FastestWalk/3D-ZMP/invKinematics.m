function [q_star] = invKinematics(q,param,model,desired,CoM_star)

q0 = q;

% % Cost function Tuning Parameters
endEffectorTune = 1.30e-2;
qAngleTune = 9e-4;

% Cost Function
fun = @(Q) endEffectorTune*norm(endEffector(kinematics_3D(Q,param,model),model) - desired) + qAngleTune*norm(q0 - Q);
options = optimoptions('fmincon',...
    'Algorithm','sqp',...
    'Display','none',... % use 'iter' to display optimisation diagnostic
    'FunctionTolerance',1e-3,...
    'OptimalityTolerance',1e-3,...
    'StepTolerance',1e-4,...
    'MaxFunctionEvaluations',1e5,...
    'MaxIterations',20);

nlconst = @(Q) nlcon(Q,param,model,CoM_star);

% % Lower Bound limit angle
% lb = [  -pi/16;     % θ₁
%         -pi/32;     % θ₂
%         -pi/4;      % θ₃
%          0;         % θ₄
%         -pi/6;      % θ₅
%         -pi/16;     % θ₆
%         -pi/16;     % θ₇
%         -pi/32;     % θ₈
%         -pi/12;     % θ₉
%         -pi/2;      % θ₁₀
%         -pi/6;      % θ₁₁
%         -pi/16];    % θ₁₂
% 
% % Upper Bound limit angle
% ub = [  pi/4;       % θ₁
%         pi/16;      % θ₂
%         pi/12;      % θ₃
%         pi/2;       % θ₄
%         pi/6;       % θ₅
%         pi/16;      % θ₆
%         pi/4;       % θ₇
%         pi/16;      % θ₈
%         pi/4;       % θ₉
%         0;          % θ₁₀
%         pi/6;       % θ₁₁
%         pi/16;];    % θ₁₂

q_star = fmincon(fun,q,[],[],[],[],[],[],nlconst,options);

end