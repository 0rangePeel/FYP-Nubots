function [c,ceq] = nlcon(q,param,model,CoM_star)
    CoM_value = CoM(kinematics_3D(q, param, model), param, model, q) .* [1; 1; 1];

    HT = kinematics_3D(q,param,model);

    tolerance = 0.005;
    % tolerance = 0.005;

    torsoTolerance = 1;

%     heightTop = 0.515;
%     heightLow = 0.46;
    heightTop = 0.475;
    heightLow = 0.4385;

    chestAngle = 0.995;

    % c = hypot(CoM_value(1) - CoM_star(1) , CoM_value(3) - CoM_star(3)) - tolerance;
%     c = [hypot(CoM_value(1) - CoM_star(1) , CoM_value(3) - CoM_star(3)) - tolerance, CoM_value(2) - heightTop, heightLow - CoM_value(2)];
    c = [hypot(CoM_value(1) - CoM_star(1),CoM_value(3) - CoM_star(3)) - tolerance,...
         CoM_value(2) - heightTop, heightLow - CoM_value(2)...;
         chestAngle - HT.Ab0(1,1), chestAngle - HT.Ab0(2,2)];
% HT.Ab0(2,2) - 1 + torsoTolerance, 1 - torsoTolerance - HT.Ab0(2,2)

    ceq = [];
end

