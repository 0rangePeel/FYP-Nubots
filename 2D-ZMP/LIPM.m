function [ZMP, CoM, model, control] = LIPM(param,control,model,k)
    %UNTITLED3 Summary of this function goes here
    %   Detailed explanation goes here
    
    % Params
        T       = control.timestep;
        NL      = control.NL;
        g       = param.g;
        zc      = param.zc;

    % Current State
    x0 = model.x(:,k);
    
    %LIPM Discretised State Equations
    Ad = [1   T   T^2/2;
          0   1   T;
          0   0   1]; 
    Bd = [T^3/6; T^2/2; T];
    Cd = [1 0 -zc/g];
    
    % Weights
    Qe = control.Qe;
    Qx = control.Qx;
    R = control.R;

    % Dimensions
    [n, ~] = size(Ad);
    [~, r] = size(Bd);
    [p, ~] = size(Cd);

    B_tilde = [Cd*Bd; Bd];
    I_tilde = [eye(p);zeros(n,p)];
    F_tilde = [Cd*Ad; Ad];
    Q_tilde = [Qe zeros(p,n); zeros(n,p) Qx];
    A_tilde = [I_tilde F_tilde];
    K_tilde = dare(A_tilde,B_tilde,Q_tilde,R);

    %% Gains
    %% Integral Action
    GI = (R + B_tilde'*K_tilde*B_tilde)\(B_tilde'*K_tilde*I_tilde);

    %% State Feedback
    Gx = (R + B_tilde'*K_tilde*B_tilde)\(B_tilde'*K_tilde*F_tilde);

    %% Differential
    
    % Initialise Differential Gain
    Gd = zeros(p,NL);
    % First column of Gd is - GI
    Gd(1) = -GI;

    % Building block
    Ac_tilde = A_tilde - B_tilde*((R + B_tilde'*K_tilde*B_tilde)\(B_tilde'*K_tilde*A_tilde));

    % Initialise future state predictions matrix 
    X_tilde = zeros(p+n,NL);
    % First column of X_tilde preficted based off -Ac*K*I
    X_tilde(:,1) = -Ac_tilde'*K_tilde*I_tilde;

    % Loop through predictions for NL horizon
    for l=2:NL
        % Gd predicted using similar equation to GI and the X_tilde values
        Gd(l) = ((R + B_tilde'*K_tilde*B_tilde)\B_tilde)'*X_tilde(:,(l - 1));
        % X_tilde predicted using old X_tilde and Ac_tilde system matrix
        X_tilde(:,l) = Ac_tilde'*X_tilde(:,(l - 1));
    end

    %% Optimal Incremental Control 
    % Refernce Signal to follow
    if k <= (length(model.CoM_star)-NL)
        y_demand = model.CoM_star(k:k+(NL-1)); 
    else
        % At end of the cycle to keep prediction horizon same size
        y_demand = [model.CoM_star(k:end) (control.stepSize*(control.stepNum-1) - (control.stepSize/2))*ones(1,(NL-1)-(length(model.CoM_star) - k))];
    end

    % Tracking Error for all previous 
    sigmaError  = model.y(1:k) - model.CoM_star(1:k);
    
    % Calculated Updated gains of controller
    control.GI(k) = GI*sum(sigmaError);
    control.Gx(k) = Gx*x0;
    control.Gd(k) = sum(Gd.*y_demand);
    
    % Optimal Incremental Control Input
    u = -GI*sum(sigmaError) - Gx*x0 - sum(Gd.*y_demand);

    % Save model - exclude last entry of x
    model.u(:,k) = u;
    model.y(:,k) = Cd*x0;
    x = Ad*x0 + Bd*u;
    if  model.tspan(k) < model.tspan(end)
        model.x(:,k+1) = x;            % X discrete
    end  

    % Outputs for FKM & FKM⁻¹
    ZMP = model.y(1,k);
    CoM = model.x(1,k);
end