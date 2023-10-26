function q = initialSystem
    %% Inital Revolute Orientation
    % q = [           0; % θ₁ right_hip_yaw - opp
    %             pi/64; % θ₂ right_hip_roll [hip] - opp
    %             -pi/5; % θ₃ right_hip_pitch
    %            2*pi/5; % θ₄ right_knee_pitch
    %             -pi/5; % θ₅ right_ankle_pitch
    %            -pi/64; % θ₆ right_ankle_roll - opp
    %                 0; % θ₇ left_hip_yaw
    %             pi/64; % θ₈ left_hip_roll [hip]
    %              pi/5; % θ₉ left_hip_pitch - opp
    %           -2*pi/5; % θ₁₀ left_knee_pitch - opp
    %              pi/5; % θ₁₁ left_ankle_pitch - opp
    %           -pi/64]; % θ₁₂ left_ankle_roll
%         q = [           0; % θ₁ right_hip_yaw - opp
%                 pi/64; % θ₂ right_hip_roll [hip] - opp
%                 -pi/5; % θ₃ right_hip_pitch
%                2*pi/5; % θ₄ right_knee_pitch
%                 -pi/5; % θ₅ right_ankle_pitch
%                -0.063; % θ₆ right_ankle_roll - opp
%                     0; % θ₇ left_hip_yaw
%                 pi/64; % θ₈ left_hip_roll [hip]
%                  pi/5; % θ₉ left_hip_pitch - opp
%               -2*pi/5; % θ₁₀ left_knee_pitch - opp
%                  pi/5; % θ₁₁ left_ankle_pitch - opp
%               -0.063]; % θ₁₂ left_ankle_roll

%     q = [           0; % θ₁ right_hip_yaw - opp
%                 pi/32; % θ₂ right_hip_roll [hip] - opp
%                 -pi/5; % θ₃ right_hip_pitch
%                2*pi/5; % θ₄ right_knee_pitch
%                 -pi/5; % θ₅ right_ankle_pitch
%                -pi/32; % θ₆ right_ankle_roll - opp
%                     0; % θ₇ left_hip_yaw
%                 pi/32; % θ₈ left_hip_roll [hip]
%                  pi/5; % θ₉ left_hip_pitch - opp
%               -2*pi/5; % θ₁₀ left_knee_pitch - opp
%                  pi/5; % θ₁₁ left_ankle_pitch - opp
%               -pi/32]; % θ₁₂ left_ankle_roll

    q = [           0; % θ₁ right_hip_yaw - opp
                pi/24; % θ₂ right_hip_roll [hip] - opp
                -pi/5; % θ₃ right_hip_pitch
               2*pi/5; % θ₄ right_knee_pitch
                -pi/5; % θ₅ right_ankle_pitch
               -pi/24; % θ₆ right_ankle_roll - opp
                    0; % θ₇ left_hip_yaw
                pi/24; % θ₈ left_hip_roll [hip]
                 pi/5; % θ₉ left_hip_pitch - opp
              -2*pi/5; % θ₁₀ left_knee_pitch - opp
                 pi/5; % θ₁₁ left_ankle_pitch - opp
              -pi/24]; % θ₁₂ left_ankle_roll
end

