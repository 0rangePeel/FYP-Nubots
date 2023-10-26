function control = parametersControl
    %% Contains ZMP parameters - These do not change over time
    control.timeHorizon  =  1.0;                                     % Seconds 
%     control.timestep     =  0.0050;                                    % Seconds
    control.timestep     =  0.0200; 
%     control.timestep     =  0.0100; 
    control.NL           =  control.timeHorizon / control.timestep;    % INTEGER
    % control.stepSize     =  0.05;
%     control.stepSize     =  0.065;
    control.stepSize     =  0.1;
%     control.stepNum      =  13;
    control.stepNum      =  13;
    % Weights
    control.Qe = diag([5e1 5e1]);
    control.Qx = zeros(6);
    control.R = diag([5e-5 5e-5]);
end

