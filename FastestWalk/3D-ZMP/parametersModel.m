function model = parametersModel(t,param,control)
    %% Model paramters which do change over time
    %% Length of Simulation
    model.tspan         = 0:control.timestep:t;

    % Initialise arrays    
    model.pREF          = zeros(2,length(model.tspan));
    model.u             = zeros(2,length(model.tspan));
    
    % Position of Right and Left feet
    model.rRBb = [0;0;0];
    model.rLBb = [0;0;0];
    

    % Initial Leg base param
    % model.base = 1;  % Right Base
    model.base = -1; % Left Base
end

