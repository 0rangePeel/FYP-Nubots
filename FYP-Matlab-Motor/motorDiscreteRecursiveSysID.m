clc
clear all

%% Create set of data to perform SysID and Test against

motorData = {'SYS_ID_DATA_LOAD7.mat', ...
             'SYS_ID_DATA_LOAD8.mat', ...
             'SYS_ID_DATA_LOAD9.mat', ...
             'SYS_ID_DATA_LOAD10.mat', ...
             'SYS_ID_DATA_LOAD11.mat', ...
             'SYS_ID_DATA_LOAD12.mat'};

rmse_Validation_VinTorque_Avg_opt = 1; % Some large std

%% Torque Recursive Validation
for i = 1:length(motorData)
    % Get SysID values for i .mat
    [~,~,velocity,Vin,torqueInterp] = motorDataUnpacker(motorData{i});
    
    % Least Squares Method - Torque
    [phi, Kw_T] = motorTorqueLSME(velocity, Vin, torqueInterp);
    
    % Initialise Array
    rmse_Validation_VinTorque_Total = zeros(1,length(motorData));
    
    % Loop for all data sets
    for j = 1:length(motorData)
        [rmse_Validation_VinTorque_Total(j)] = motorValidationTorque(motorData{j}, phi, Kw_T);
    end
    
    % If data set has better RMSE then store the SYSID values
    if (mean(rmse_Validation_VinTorque_Total) < rmse_Validation_VinTorque_Avg_opt)
        phi_opt = phi;
        Kw_T_opt = Kw_T;
        rmse_Validation_VinTorque_Avg_opt = mean(rmse_Validation_VinTorque_Total);
    end
end

rmse_Validation_VinCurrent_Avg_opt = 1; % Some large std

%% Current Recursive Validation
for i = 1:length(motorData)
    % Get SysID values for i .mat
    [~,current,velocity,Vin,~] = motorDataUnpacker(motorData{i});
    
    % Least Squares Method - Current
    [Ra, Kw_I] = motorCurrentLSME(velocity, Vin, current);
    
    % Initialise Array
    rmse_Validation_VinCurrent_Total = zeros(1,length(motorData));
    
    % Loop for all data sets
    for j = 1:length(motorData)
        [rmse_Validation_VinCurrent_Total(j)] = motorValidationCurrent(motorData{j}, Ra, Kw_I);
    end
    
    % If data set has better RMSE then store the SYSID values
    if (mean(rmse_Validation_VinCurrent_Total) < rmse_Validation_VinCurrent_Avg_opt)
        Ra_opt = Ra;
        Kw_I_opt = Kw_I;
        rmse_Validation_VinCurrent_Avg_opt = mean(rmse_Validation_VinCurrent_Total);
    end
end

%% Results
disp(['TORQUE RMSE OPT  :' num2str(rmse_Validation_VinTorque_Avg_opt)]);
disp(['WITH OPTIMUM SYSID VALUES - phi = ' num2str(phi_opt) ' and Kw_T = ' num2str(Kw_T_opt)]);
disp(['CURRENT RMSE OPT :' num2str(rmse_Validation_VinCurrent_Avg_opt)]);
disp(['WITH OPTIMUM SYSID VALUES - Ra = ' num2str(Ra_opt) ' and Kw_I = ' num2str(Kw_I_opt)]);

%% Plot using optimum SysID values
if rmse_Validation_VinCurrent_Avg_opt > rmse_Validation_VinTorque_Avg_opt
    disp('TORQUE RESULTS IN BEST SYSID')
    for i = 1:length(motorData)
        [time,~,velocity,Vin,torqueInterp] = motorDataUnpacker(motorData{i});

        N = 255;
        VinTorque = phi_opt*(torqueInterp/N) + Kw_T_opt*velocity;

        figure
        hold on
        grid on
        plot(time,Vin,'LineWidth', 2)
        plot(time,VinTorque,'*')
        legend('Vin', 'Torque Calculation')
    end
else
    disp('CURRENT RESULTS IN BEST SYSID')
    for i = 1:length(motorData)
        [time,current,velocity,Vin,~] = motorDataUnpacker(motorData{i});

        VinCurrent = Ra_opt*current + Kw_I_opt*velocity;

        figure
        hold on
        grid on
        plot(time,Vin,'LineWidth', 2)
        plot(time,VinCurrent,'*')
        legend('Vin', 'Current Calculation')
    end
end

