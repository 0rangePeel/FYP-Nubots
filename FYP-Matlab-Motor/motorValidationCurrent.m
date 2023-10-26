function [rmse_Validation_VinCurrent] = motorValidationCurrent(motorData_Validation, Ra, Kw_I)
    
    [~,current_Validation,velocity_Validation,Vin_Validation,~] = motorDataUnpacker(motorData_Validation);
    
    % Current Validation
    VinCurrent_Validation = Ra*current_Validation + Kw_I*velocity_Validation;
    
    % Root Mean Squared Error
    rmse_Validation_VinCurrent = sqrt(mean((Vin_Validation - VinCurrent_Validation).^2));
end