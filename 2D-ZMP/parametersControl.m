function control = parametersControl

    control.timeHorizon  =  1.5;                                     % Seconds 
    control.timestep     =  0.02;                                    % Seconds
    control.NL           =  control.timeHorizon / control.timestep;    % INTEGER
    control.stepSize     =  0.1;
    control.stepNum      =  6;
    % Weights
    control.Qe = 1;
    control.Qx = zeros(3);
    control.R = 1e-4;
end

