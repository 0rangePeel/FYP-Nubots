function param = parametersSystem
    %% This function contains system parameters which do not change with time
    % Leg Length params
    param.P = 0.11;    % Pelvis
    param.F = 0.19954;    % Femur
    param.T = 0.19942;    % Tibia
    param.A = 0.1;    % Ankle
    param.S = 0.05;   % Servo Length

    % Leg Mass params
    param.mP = (0.135 + 0.419 + 2.539 + (0.02 + 0.3 + 0.26)*2)/2;   % Pelvis
    param.mF = 0.3423;    % Femur
    param.mT = 0.1719; % Tibia
    param.mA = 0.2023;  % Ankle
    param.mS = 0.3; % Servo

    param.PH = 0.102150; % Rough Height of pelvis from ground

end

