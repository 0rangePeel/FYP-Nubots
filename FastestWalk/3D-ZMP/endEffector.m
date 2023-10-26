function xe = endEffector(HT,model)
%ENDEFFECTOR Summary of this function goes here
%   Detailed explanation goes here

if model.base == 1
    % Left swing foot
    endEffector = HT.AbLB;
elseif model.base == -1
    % Right swing foot
    endEffector = HT.AbRB;
else
    disp('ERROR-Incorrect Base Parameter')
end

phi =   atan2( endEffector(3,2), endEffector(3,3) );  
theta = atan2(-endEffector(3,1), sqrt( endEffector(3,2)^2 + endEffector(3,3)^2 ) );
psi =   atan2( endEffector(2,1), endEffector(1,1) );

xe = [endEffector(1:3,4); % X Y Z
      phi;        % ϕ
      theta;      % θ
      psi];       % Ψ

end

