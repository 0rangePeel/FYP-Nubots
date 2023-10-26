function [time,current,velocity,Vin,torqueInterp] = motorDataUnpacker2(motorData)

    %% Load motor data
    load(motorData);

    %% Unpack Data
    time            = SysIDData.time;
    currentRAW      = SysIDData.current;
    velocityRAW     = SysIDData.velocity;
    pwmRAW          = SysIDData.pwm;
    voltageRAW      = SysIDData.voltage;

    %% Remove Unrecorded Data
    for i = 1:size(time)
        if time(i) == 0
            time = time(1:i-1,:);
            currentRAW = currentRAW(1:i-1,:);
            velocityRAW = velocityRAW(1:i-1,:);
            pwmRAW = pwmRAW(1:i-1,:);
            voltageRAW = voltageRAW(1:i-1,:);
            break;       
        end
    end

    %% Perform Necessary Conversions
    currentUnit = 3.36; % mA
    current = (currentRAW * currentUnit) / 1000;
    
    velocityUnit = 0.229; % rpm
    velocity = velocityRAW * velocityUnit * (pi / 30);
    
    pwmUnit = 0.113; % Unit Percentage
    pwm = (pwmRAW * pwmUnit)/100;
    
    Vbat = 12.0;
    Vin = pwm * Vbat;
    

    %% Interpolate Data - This is for NaN results from Torque Rig
    % Find the indices of NaN values
    nan_indices = isnan(voltageRAW);
    
    % Create a vector of indices for non-NaN values
    non_nan_indices = find(~nan_indices);
    
    % Linearly interpolate the NaN values
    voltageRAWInterp = voltageRAW;
    voltageRAWInterp(nan_indices) = interp1(non_nan_indices, voltageRAW(non_nan_indices), find(nan_indices), 'linear');

    % Measure Torque
    Vsen = 1.2462;
    Voffset = 1.6645;
    %Voffset = mean(voltageRAWInterp(1:50));
    torqueInterp = -Vsen * (voltageRAWInterp - Voffset);


end