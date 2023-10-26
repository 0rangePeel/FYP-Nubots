function [rmse_Validation_VinTorque,rmse_Validation_VinCurrent] = motorValidation(motorData_Validation, phi, Kw_T, Ra, Kw_I)
    
    [~,current_Validation,velocity_Validation,Vin_Validation,torqueInterp_Validation] = motorDataUnpacker(motorData_Validation);
    
    % Torque Validation
    N = 255;
    VinTorque_Validation = phi*(torqueInterp_Validation/N) + Kw_T*velocity_Validation;
    
    % Current Validation
    VinCurrent_Validation = Ra*current_Validation + Kw_I*velocity_Validation;
    
    % Root Mean Squared Error
    rmse_Validation_VinTorque  = sqrt(mean((Vin_Validation - VinTorque_Validation).^2));
    rmse_Validation_VinCurrent = sqrt(mean((Vin_Validation - VinCurrent_Validation).^2));
end