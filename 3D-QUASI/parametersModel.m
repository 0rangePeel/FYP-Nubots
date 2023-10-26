function model = parametersModel
    %% Model Parameters which change over time
    % Position of Right and Left feet
    model.rRBb = [0;0;0];
    model.rLBb = [0;0;0];

    % Initial Leg base param
    % model.base = 1;  % Right Base
    model.base = -1; % Left Base

end

