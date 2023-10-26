function param = parametersSystem

    param.zc             =  0.6;    % m
    param.g              =  9.81;    % ms⁻² 
    param.m              =  10;  % kg
    
    % Leg Length params
    param.P = 0.2;    % Pelvis
    param.F = 0.3;    % Femur
    param.T = 0.3;    % Tibia
    param.A = 0.1;    % Ankle
    
    % Leg Mass params
    param.mP = 10;    % Pelvis
    param.mF = 1;    % Femur
    param.mT = 0.75; % Tibia
    param.mA = 0.5;  % Ankle
    
end

