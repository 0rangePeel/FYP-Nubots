clc
clear all
close all
% Helpful time keeping :)
tic
hbar = waitbar(0,'Simulation Progress');

%% Starting Parameters
% Inital Revolute Orientation
q = initialConditions;

%% Parameters
param = parametersSystem;

%% Control
control = parametersControl;

%% Model
% Simulation Time
t = 10;
model = parametersModel(t,param,control);

%% Simulation Setup
ZMPpc = zeros(1,length(model.tspan));
CoMpc = zeros(1,length(model.tspan));

HT = kinematics(q,param,model);
xe = endEffector(HT,model);
[~,outputCoM] = CoM(HT,param,model,q);
CoMr = outputCoM.Output(1:3,4);

[model.CoM_star,eeL,eeR,ee,base,eeB] = trajectory(model.tspan,control,param,model,xe);
xe_n = [ee ; zeros(3,control.NL*control.stepNum + 1);xe(6)*ones(1,control.NL*control.stepNum + 1)];

HT_Hist = HT;
CoM_Hist = outputCoM;

%% Error Values
xe_error = 0;
CoM_error = 0;
q_change = zeros(6,length(model.tspan));
CoM_pos = zeros(1,451);

%% Simulation
for i=1:length(model.tspan)
    model.base = base(i);
    model.rRBb = eeB(:,i);
    model.rLBb = eeB(:,i);
    [ZMPpc(i), CoMpc(i), model, control] = LIPM(param,control,model,i);
    q = invKinematics(q,param,model,xe_n(:,i),CoMpc(i));
    HT = kinematics(q,param,model);
    xe = endEffector(HT,model);
    [temp,outputCoM] = CoM(HT,param,model,q);
    CoM_pos(i) = temp(1);
    HT_Hist = [HT_Hist HT];
    CoM_Hist = [CoM_Hist outputCoM];
    xe_error = xe_error + norm(xe.*[1;1;0;1;1;1] - xe_n(:,i));
    CoM_error = CoM_error + norm(outputCoM.Output(1,4) - CoMpc(i));
    if i > 1
        q_change(:,i) = abs(rad2deg(q_temp) - rad2deg(q));
    end
    q_temp = q;
    waitbar(i/(length(model.tspan)),hbar)
end
close(hbar);
toc

%% Display Total Error
disp('Total End Effector Error (mm)')
disp(xe_error*1000);
disp('Total CoM Error (mm)')
disp(CoM_error*1000);
disp('Max Angle change in 0.02 seconds (deg)')
disp(max(max(q_change)));
disp('Therefore Max speed Achieved (deg/sec)')
disp(max(max(q_change))/0.02);

%% Animation
fig = 1;
figure(fig);

set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% Create and open video writer object
v = VideoWriter('ZMP_2D.mp4','MPEG-4');
v.Quality   = 100;
open(v);
for i = 1:(length(model.tspan))
    clf(fig);
    ZMPPlot = [ZMPpc(i); 0;0];
    ZMP_custom_plot(CoMpc(i),param,eeL(:,i),eeR(:,i),ee(:,i),CoM_Hist(:,i),HT_Hist(:,i),ZMPPlot)
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);

%% CoM Out
CoM_pos = zeros(1,length(CoM_Hist)-1);
for i=1:length(CoM_Hist)-1
    temp = CoM_Hist(i).Output;
    CoM_pos(i) = temp(1,4);
end

%% Plots
% CoM_pos = CoM_Hist(1:end-1);
fig2 = figure;
set(fig2,'Position', [100,100,800,400])
plot(model.tspan, model.CoM_star,'linewidth',3);
hold on 
plot(model.tspan,ZMPpc,'linewidth',3);
plot(model.tspan,CoM_pos,'linewidth',3);
plot(model.tspan,CoMpc,'--','linewidth',3);
xlabel('Time (sec)');
ylabel('X-Axis (m)');
legend("ZMP Reference","ZMP","CoM-Ref","CoM",'location','best');
title('2D ZMP and CoM Trajectory Tracking')
grid on
saveas(gcf,'ZMP_2D.png')