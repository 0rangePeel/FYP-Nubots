function model = parametersModel(t,param,control)

    %% Length of Simulation
    model.tspan         = 1:control.timestep:t;
    
    %% Model
    % Initialise arrays
    model.x             = [-(control.stepSize/2)*ones(1,length(model.tspan)); zeros(2,length(model.tspan))];
    model.y             = -(control.stepSize/2)*ones(1,length(model.tspan));
    model.pREF          = zeros(1,length(model.tspan));
    model.u             = zeros(1,length(model.tspan));
    
    % Position of Right and Left feet
    model.rRBb = [0;0;param.P/2];
    model.rLBb = [0;0;-param.P/2];

    % Leg base param
    model.base = 1;  % Right Base
    %model.base = -1; % Left Base

    %% Initial Foot Position
    model.x0 = [-control.stepSize/2; 0; 0];

end

