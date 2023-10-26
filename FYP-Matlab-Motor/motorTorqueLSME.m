function [phi,Kw] = motorTorqeueSME(velocity, Vin, torqueInterp)
    %% Least Squares Method - Torque
    % Gear Ratio
    N = 225;
    
    num = [torqueInterp/N velocity];
    
    [num2] = lsqr(num,Vin);
    
    phi = num2(1); % phi = Ra * (1 / K_I)
    Kw = num2(2);
end