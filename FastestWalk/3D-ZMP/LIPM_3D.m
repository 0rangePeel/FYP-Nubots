function [ZMP, CoM, model, control] = LIPM_3D(param,control,model,k)
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
    Ad_temp = [1   T   T^2/2;
               0   1   T;
               0   0   1]; 
    Ad = [Ad_temp zeros(3);
          zeros(3) Ad_temp];

    Bd_temp = [T^3/6; T^2/2; T];
    Bd = [Bd_temp zeros(3,1); zeros(3,1) Bd_temp];

    Cd_temp = [1 0 -zc/g];
    Cd = [Cd_temp zeros(1,3);
          zeros(1,3) Cd_temp];
    
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
    %GI = (R + B_tilde'*K_tilde*B_tilde)\(B_tilde'*K_tilde*I_tilde);
    GI = (((R+B_tilde'*K_tilde*B_tilde)\eye(2))*B_tilde')*K_tilde*I_tilde;

    %% State Feedback
    Gx = (((R+B_tilde'*K_tilde*B_tilde)\eye(2))*B_tilde')*K_tilde*F_tilde;

    %% Differential
    
    % Initialise Differential Gain
    Gd = [-[GI(1,1);GI(2,2)] zeros(p,NL-1)];
    % First column of Gd is - GI
    %Gd = -GI;

    % Building block
    Ac_tilde = A_tilde - B_tilde*(((R+B_tilde'*K_tilde*B_tilde)\eye(2))*B_tilde')*K_tilde*A_tilde;

    % Initialise future state predictions matrix 
    X_tilde = -Ac_tilde'*K_tilde*I_tilde;
    X_tilde1 = [X_tilde(:,1) zeros(p+n,NL-1)];
    X_tilde2 = [X_tilde(:,1) zeros(p+n,NL-1)];
    % First column of X_tilde preficted based off -Ac*K*I
    %X_tilde(:,1) = -Ac_tilde'*K_tilde*I_tilde;

    % Loop through predictions for NL horizon
    for l=2:NL
        % Gd predicted using similar equation to GI and the X_tilde values
        A = (((R+B_tilde'*K_tilde*B_tilde)\eye(2))*B_tilde')*X_tilde1(:,(l - 1));
        B = (((R+B_tilde'*K_tilde*B_tilde)\eye(2))*B_tilde')*X_tilde2(:,(l - 1));

        Gd(:,l) = [A(1) B(1)];


        %Gd(:,l) = (((R+B_tilde'*K_tilde*B_tilde)\eye(2))*B_tilde')*X_tilde1(:,(l - 1));
        % X_tilde predicted using old X_tilde and Ac_tilde system matrix
        X_tilde1(:,l) = Ac_tilde'*X_tilde1(:,(l - 1));
        X_tilde2(:,l) = Ac_tilde'*X_tilde2(:,(l - 1));
    end

    %% Optimal Incremental Control 
    % Refernce Signal to follow
    if k <= (length(model.CoM_star)-NL)
        y_demand = [model.CoM_star(1,k:k+(NL-1));model.CoM_star(2,k:k+(NL-1))]; 
    else
        % At end of the cycle to keep prediction horizon same size
        y_demand = [model.CoM_star(1,k:end) (control.stepSize*(control.stepNum-1))*ones(1,(NL-1)-(length(model.CoM_star) - k));
                    model.CoM_star(2,k:end) 0.1*(-1)^(control.stepNum+1)*ones(1,(NL-1)-(length(model.CoM_star) - k))];
    end

    % Tracking Error for all previous 
    sigmaError  = model.y(:,1:k) - model.CoM_star(:,1:k);
    
    % Optimal Incremental Control Input
    u = -GI*[sum(sigmaError(1,:));sum(sigmaError(2,:))] - Gx*x0 - [sum(Gd(1,:).*y_demand(1,:));sum(Gd(2,:).*y_demand(2,:))];

    % Save model - exclude last entry of x
    model.u(:,k) = u;
    model.y(:,k) = Cd*x0;
    x = Ad*x0 + Bd*u;
    if  model.tspan(k) < model.tspan(end)
        model.x(:,k+1) = x;            % X discrete
    end  

    % Outputs for FKM & FKM⁻¹
    ZMP = model.y(:,k);
    CoM = model.x(:,k);

    % Calculated Updated gains of controller
    % control.GI(k) = GI*sum(sigmaError);
    % control.Gx(k) = Gx*x0;
    % control.Gd(k) = sum(Gd.*y_demand);
end