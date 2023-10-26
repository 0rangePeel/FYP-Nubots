function [Ra, Kw] = motorCurrentLSME(velocity, Vin, current)
    %% Least Squares Method - Current
    num = [current velocity];

    [num2] = lsqr(num,Vin);
    
    Ra = num2(1);
    Kw = num2(2);
end