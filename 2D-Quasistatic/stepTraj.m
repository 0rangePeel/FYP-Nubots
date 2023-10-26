function [xe_n,Cp] = stepTraj(xe,dS,tf,step)

dP = 0.1;

x = [zeros(1,tf*0.3) linspace(0,dS,tf*0.6) dS*ones(1,tf*0.1)];

y = (2*dP*x)/dS - (2*dP*x.^2)/dS^2;

xe_n = [x + xe(1);y;zeros(3,tf);xe(6)*ones(1,tf)];

if step == 0
    Cp = dS*ones(1,tf);
else
    Cp = [0.5*dS*step + 0.5*dS*linspace(0.05,1,tf*0.3) 0.5*dS*(step+1)*ones(1,tf*0.7)];
end

end

