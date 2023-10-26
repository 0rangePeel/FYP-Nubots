clc
clear all
tic

%% Starting Parameters
q = initialConditions;

%% Parameters
param = parametersSystem;

%% Model
model = parametersModel;

model.rRBb = [param.A;0;param.P/2];
model.rLBb = [param.A;0;-param.P/2];

%% Animation
fig1 = figure;
step = 0;
% Animation Length
tf = 100;
ee = zeros(6,tf);
positionCoM=zeros(3,tf);
HT = kinematics(q,param,model);
xe = endEffector(HT,model);
[xe_n,Cp] = stepTraj(xe,0.05,tf,step);
CpHist = Cp;
[~,outputCoM] = CoM(HT,param,model,q);
custom_plot(HT,outputCoM,Cp(1))

set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% Create and open video writer object
v = VideoWriter('Quasistatic_2D.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i = 1:(tf)
    clf(fig1)

    q = invKinematics(q,param,model,xe_n(:,i),Cp(i));

    HT = kinematics(q,param,model);

    xe = endEffector(HT,model);

    ee(:,i) = xe(:,1);

    [~,outputCoM] = CoM(HT,param,model,q);
    
    positionCoM(:,i) = outputCoM.Output(1:3,4);

    custom_plot(HT,outputCoM,Cp(i))

    % pause(0.02);
    frame = getframe(gcf);
    writeVideo(v,frame);

end



step = 1;
model.base = 1;  % Right Base
% Position of Right and Left feet
model.rRBb = [param.A;0;param.P/2] + [0.05;0;0];

HT = kinematics(q,param,model);
xe = endEffector(HT,model);
[~,outputCoM] = CoM(HT,param,model,q);
custom_plot(HT,outputCoM,Cp(i))
[xe_n,Cp] = stepTraj(xe,0.1,tf,step);
CpHist = [CpHist Cp];

for i = 1:(tf)
    clf(fig1)

    q = invKinematics(q,param,model,xe_n(:,i),Cp(i));

    HT = kinematics(q,param,model);

    xe = endEffector(HT,model);

    ee(:,i+step*tf) = xe(:,1);

    [~,outputCoM] = CoM(HT,param,model,q);
    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);

    custom_plot(HT,outputCoM,Cp(i))

    % pause(0.02);
    frame = getframe(gcf);
    writeVideo(v,frame);

end

step = 2;
model.base = -1;  % Left Base
% Position of Right and Left feet
model.rLBb = [param.A;0;-param.P/2] + [0.1;0;0];

HT = kinematics(q,param,model);
xe = endEffector(HT,model);
[~,outputCoM] = CoM(HT,param,model,q);
custom_plot(HT,outputCoM,Cp(i))
[xe_n,Cp] = stepTraj(xe,0.1,tf,step);
CpHist = [CpHist Cp];

for i = 1:(tf)
    clf(fig1)

    q = invKinematics(q,param,model,xe_n(:,i),Cp(i));

    HT = kinematics(q,param,model);

    xe = endEffector(HT,model);

    ee(:,i+step*tf) = xe(:,1);

    [~,outputCoM] = CoM(HT,param,model,q);
    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);

    custom_plot(HT,outputCoM,Cp(i))

    % pause(0.02);
    frame = getframe(gcf);
    writeVideo(v,frame);

end

step = 3;
model.base = 1;  % Right Base
% Position of Right and Left feet
model.rRBb = [param.A;0;param.P/2] + [0.15;0;0];

HT = kinematics(q,param,model);
xe = endEffector(HT,model);
[~,outputCoM] = CoM(HT,param,model,q);
custom_plot(HT,outputCoM,Cp(i))
[xe_n,Cp] = stepTraj(xe,0.1,tf,step);
CpHist = [CpHist Cp];

for i = 1:(tf)
    clf(fig1)

    q = invKinematics(q,param,model,xe_n(:,i),Cp(i));

    HT = kinematics(q,param,model);

    xe = endEffector(HT,model);

    ee(:,i+step*tf) = xe(:,1);

    [~,outputCoM] = CoM(HT,param,model,q);
    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);

    custom_plot(HT,outputCoM,Cp(i))

    % pause(0.02);
    frame = getframe(gcf);
    writeVideo(v,frame);

end

step = 4;
model.base = -1;  % Left Base
% Position of Right and Left feet
model.rLBb = [param.A;0;-param.P/2] + [0.2;0;0];

HT = kinematics(q,param,model);
xe = endEffector(HT,model);
[~,outputCoM] = CoM(HT,param,model,q);
custom_plot(HT,outputCoM,Cp(i))
[xe_n,Cp] = stepTraj(xe,0.1,tf,step);
CpHist = [CpHist Cp];

for i = 1:(tf)
    clf(fig1)

    q = invKinematics(q,param,model,xe_n(:,i),Cp(i));

    HT = kinematics(q,param,model);

    xe = endEffector(HT,model);

    ee(:,i+step*tf) = xe(:,1);

    [~,outputCoM] = CoM(HT,param,model,q);
    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);

    custom_plot(HT,outputCoM,Cp(i))

    % pause(0.02);
    frame = getframe(gcf);
    writeVideo(v,frame);

end

close(v);

tf = tf*5;

%% End Effector Position
fig2 = figure;
subplot(3,1,1)
sgtitle('End Effector Position')
plot(linspace(0,tf,tf),ee(1,:))
grid on
title('X')
subplot(3,1,2)
plot(linspace(0,tf,tf),ee(2,:))
grid on
title('Y')
subplot(3,1,3)
plot(linspace(0,tf,tf),rad2deg(ee(6,:)))
grid on
title('PSI')
saveas(gcf,'QuasistaticEE_2D.png')
%% CoM Position
fig3 = figure;
subplot(2,1,1)
sgtitle('CoM Position')
plot(linspace(0,50,tf),positionCoM(1,:),'LineWidth',2)
grid on
title('X')
xlabel('Time(s)')
ylabel('X (m)')
hold on
subplot(2,1,1)
plot(linspace(0,50,tf),CpHist,'--','LineWidth',2)
grid on
hold on
subplot(2,1,1)
plot(linspace(0,50,tf),CpHist-0.020,'LineWidth',2)
grid on
hold on
subplot(2,1,1)
plot(linspace(0,50,tf),CpHist+0.02,'LineWidth',2)
grid on
title('Cp')
subplot(2,1,2)
plot(linspace(0,50,tf),positionCoM(2,:),'LineWidth',2)
grid on
title('Y')
saveas(gcf,'QuasistaticCoM_2D.png')

%% Error Print

max(abs(positionCoM(1,:)-CpHist))

%% 
fig4 = figure;
set(fig4,'Position', [100,100,800,400])

plot(linspace(0,50,tf),positionCoM(1,:),'LineWidth',4)
hold on
plot(linspace(0,50,tf),CpHist,'--','LineWidth',4)
hold on
plot(linspace(0,50,tf),CpHist-0.010,'--','color','black','LineWidth',2)
hold on
plot(linspace(0,50,tf),CpHist+0.01,'--','color','black','LineWidth',2)
grid on

    xlabel('Time (sec)');
    ylabel('X-Axis (m)');
    title('2D Quasistatic CoM Trajectory Tracking')
    legend('CoM','CoM Trajectory', 'Limit', 'Location', 'best')

saveas(fig4,'2DQuasi.png')
