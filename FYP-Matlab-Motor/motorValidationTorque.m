function [rmse_Validation_VinTorque] = motorValidationTorque(motorData_Validation, phi, Kw_T)
    
    [~,~,velocity_Validation,Vin_Validation,torqueInterp_Validation] = motorDataUnpacker(motorData_Validation);
    
    % Torque Validation
    N = 255;
    VinTorque_Validation = phi*(torqueInterp_Validation/N) + Kw_T*velocity_Validation;
    
    
    % Root Mean Squared Error
    rmse_Validation_VinTorque  = sqrt(mean((Vin_Validation - VinTorque_Validation).^2));
end