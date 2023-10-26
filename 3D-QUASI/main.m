clc
clear all
tic

%% Starting Parameters
q = initialSystem;

%% Parameters
param = parametersSystem;

%% Model
model = parametersModel;

%% Run Simulation
TF = 700;
hbar = waitbar(0,'Simulation Progress');
step = 0;
% Animation Length
tf = 100;
ee = zeros(6,tf);
positionCoM=zeros(3,tf);

HT = kinematics_3D(q,param,model);
model.rRBb = HT.AbRB(1:3,4);
model.rLBb = HT.AbLB(1:3,4);

HT_Hist = HT;
xe = endEffector(HT,model);

[FinalCoM,outputCoM] = CoM(HT,param,model,q);
dS = 0.05;
[xe_star,CoM_star] = stepTraj(xe,dS,FinalCoM,tf,step,param,model);
xe_star_Hist = xe_star;
CoM_star_Hist = CoM_star;

kinematics_plot(HT,outputCoM,CoM_star(:,1),xe_star,CoM_star);

save_q = (q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)])';

for i = 1:(tf)
    q = invKinematics(q,param,model,xe_star(:,i),CoM_star(:,i));
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee(:,i) = xe(:,1);
    [~,outputCoM] = CoM(HT,param,model,q);    
    positionCoM(:,i) = outputCoM.Output(1:3,4);
    waitbar((i + step*tf)/(TF) ,hbar);
end

%%
step = 1;
model.base = 1;  % Right Base
% Position of Right and Left feet
model.rRBb = HT.AbRB(1:3,4);

HT = kinematics_3D(q,param,model);
xe = endEffector(HT,model);
[FinalCoM,~] = CoM(HT,param,model,q);
dS = 0.1;
[xe_star,CoM_star] = stepTraj(xe,dS,FinalCoM,tf,step,param,model);
xe_star_Hist = [xe_star_Hist xe_star];
CoM_star_Hist = [CoM_star_Hist CoM_star];

% kinematics_plot(HT,outputCoM,CoM_star(:,1),xe_star,CoM_star);

for i = 1:(tf)
    q = invKinematics(q,param,model,xe_star(:,i),CoM_star(:,i));
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee(:,i+step*tf) = xe(:,1);
    [~,outputCoM] = CoM(HT,param,model,q);    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);
    waitbar((i + step*tf)/(TF) ,hbar);
end

step = 2;
model.base = -1;  % Left Base
% Position of Right and Left feet
model.rLBb = HT.AbLB(1:3,4);

HT = kinematics_3D(q,param,model);
xe = endEffector(HT,model);
[FinalCoM,~] = CoM(HT,param,model,q);
dS = 0.1;
[xe_star,CoM_star] = stepTraj(xe,dS,FinalCoM,tf,step,param,model);
xe_star_Hist = [xe_star_Hist xe_star];
CoM_star_Hist = [CoM_star_Hist CoM_star];

kinematics_plot(HT,outputCoM,CoM_star(:,1),xe_star,CoM_star);

for i = 1:(tf)
    q = invKinematics(q,param,model,xe_star(:,i),CoM_star(:,i));
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee(:,i+step*tf) = xe(:,1);
    [~,outputCoM] = CoM(HT,param,model,q);    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);
    waitbar((i + step*tf)/(TF) ,hbar);
end

step = 3;
model.base = 1;  % Right Base
% Position of Right and Left feet
model.rRBb = HT.AbRB(1:3,4);

HT = kinematics_3D(q,param,model);
xe = endEffector(HT,model);
[FinalCoM,~] = CoM(HT,param,model,q);
dS = 0.1;
[xe_star,CoM_star] = stepTraj(xe,dS,FinalCoM,tf,step,param,model);
xe_star_Hist = [xe_star_Hist xe_star];
CoM_star_Hist = [CoM_star_Hist CoM_star];

kinematics_plot(HT,outputCoM,CoM_star(:,1),xe_star,CoM_star);

for i = 1:(tf)
    q = invKinematics(q,param,model,xe_star(:,i),CoM_star(:,i));
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee(:,i+step*tf) = xe(:,1);
    [~,outputCoM] = CoM(HT,param,model,q);    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);
    waitbar((i + step*tf)/(TF) ,hbar);
end

step = 4;
model.base = -1;  % Left Base
% Position of Right and Left feet
model.rLBb = HT.AbLB(1:3,4);

HT = kinematics_3D(q,param,model);
xe = endEffector(HT,model);
[FinalCoM,~] = CoM(HT,param,model,q);
dS = 0.1;
[xe_star,CoM_star] = stepTraj(xe,dS,FinalCoM,tf,step,param,model);
xe_star_Hist = [xe_star_Hist xe_star];
CoM_star_Hist = [CoM_star_Hist CoM_star];

kinematics_plot(HT,outputCoM,CoM_star(:,1),xe_star,CoM_star);

for i = 1:(tf)
    q = invKinematics(q,param,model,xe_star(:,i),CoM_star(:,i));
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee(:,i+step*tf) = xe(:,1);
    [~,outputCoM] = CoM(HT,param,model,q);    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);
    waitbar((i + step*tf)/(TF) ,hbar);
end

step = 5;
model.base = 1;  % Right Base
% Position of Right and Left feet
model.rRBb = HT.AbRB(1:3,4);

HT = kinematics_3D(q,param,model);
xe = endEffector(HT,model);
[FinalCoM,~] = CoM(HT,param,model,q);
dS = 0.1;
[xe_star,CoM_star] = stepTraj(xe,dS,FinalCoM,tf,step,param,model);
xe_star_Hist = [xe_star_Hist xe_star];
CoM_star_Hist = [CoM_star_Hist CoM_star];

kinematics_plot(HT,outputCoM,CoM_star(:,1),xe_star,CoM_star);

for i = 1:(tf)
    q = invKinematics(q,param,model,xe_star(:,i),CoM_star(:,i));
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee(:,i+step*tf) = xe(:,1);
    [~,outputCoM] = CoM(HT,param,model,q);    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);
    waitbar((i + step*tf)/(TF) ,hbar);
end

step = 6;
model.base = -1;  % Left Base
% Position of Right and Left feet
model.rLBb = HT.AbLB(1:3,4);

HT = kinematics_3D(q,param,model);
xe = endEffector(HT,model);
[FinalCoM,~] = CoM(HT,param,model,q);
dS = 0.1;
[xe_star,CoM_star] = stepTraj(xe,dS,FinalCoM,tf,step,param,model);
xe_star_Hist = [xe_star_Hist xe_star];
CoM_star_Hist = [CoM_star_Hist CoM_star];

kinematics_plot(HT,outputCoM,CoM_star(:,1),xe_star,CoM_star);

for i = 1:(tf)
    q = invKinematics(q,param,model,xe_star(:,i),CoM_star(:,i));
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee(:,i+step*tf) = xe(:,1);
    [~,outputCoM] = CoM(HT,param,model,q);    
    positionCoM(:,i+step*tf) = outputCoM.Output(1:3,4);
    waitbar((i + step*tf)/(TF) ,hbar);
end

close(hbar);
toc

%% Export motor angles
right_shoulder_pitch        = 1.1;
right_shoulder_roll         = -0.3;
right_elbow_pitch           = -1.5;
left_shoulder_pitch         = 1.1;
left_shoulder_roll          = 0.3;
left_elbow_pitch            = -1.5;



data_matrix = [right_shoulder_pitch*ones(length(save_q),1) ...
               right_shoulder_roll*ones(length(save_q),1) ...
               right_elbow_pitch*ones(length(save_q),1) ...
               left_shoulder_pitch*ones(length(save_q),1) ...
               left_shoulder_roll*ones(length(save_q),1) ...
               left_elbow_pitch*ones(length(save_q),1) ...
               save_q];

% Export the data to a TXT file
output_file = 'q.txt';
dlmwrite(output_file, data_matrix, 'delimiter', '\t');
output_file = 'q0.txt';
dlmwrite(output_file, data_matrix(1,:)', 'delimiter', '\t');


%% Animation and Analysis
fig = 1;
figure(fig);

set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% Create and open video writer object
v = VideoWriter('Quasistatic_3D.mp4','MPEG-4');
v.Quality   = 100;
open(v);

fprintf("Animation Start\n");

qCounter = 0;
xeCounter = 0;

q_previous = save_q(1,:);
for i = 1:(TF)
    clf(fig);

    % Check absolute differences and print a message
    q = save_q(i,:);
    for j = 1:length(q)
        absolute_difference = abs(q(j) - q_previous(j));
        if absolute_difference > deg2rad(2)
            fprintf('Absolute difference on iteration (%d) between q(%d) and q_star(%d) is greater than 2deg: %d\n', i , j, j, rad2deg(absolute_difference));
            qCounter = qCounter + 1;
        end
    end
    rms = rmse(ee(:,i),xe_star_Hist(:,i));
    if rms > 0.01
        fprintf("EX/EE on Iter: %d RMS: %d\n", i, rms);
        xeCounter = xeCounter + 1;
    end

    q_previous = save_q(i,:);
    animation_plot(HT_Hist(:,i),positionCoM(:,i),CoM_star_Hist(:,i),xe_star,CoM_star)
    pause(0.01)
    drawnow
    % frame = getframe(gcf);
    % % resizedFrame = imresize(frame.cdata, [1280, 720]);
    % writeVideo(v, frame);

    frame = getframe(gcf);
    resizedFrame = imresize(frame.cdata, [720, NaN]); % Preserve aspect ratio, set height to 720 pixels
    writeVideo(v, resizedFrame);
end
fprintf("Animation Stop with qErrors: (%d) xeErrors: (%d)\n", qCounter, xeCounter);
close(v);

%%
fig4 = figure;
set(fig4,'Position', [100,100,800,400])

plot(linspace(0,20,tf*7),positionCoM(1,:),'LineWidth',4)
hold on
plot(linspace(0,20,tf*7),CoM_star_Hist(1,:),'--','LineWidth',4)
hold on
plot(linspace(0,20,tf*7),CoM_star_Hist(1,:)-0.010,'--','color','black','LineWidth',2)
hold on
plot(linspace(0,20,tf*7),CoM_star_Hist(1,:)+0.01,'--','color','black','LineWidth',2)
grid on

    xlabel('Time (sec)');
    ylabel('X-Axis (m)');
    title('3D Quasistatic X-axis CoM Trajectory Tracking')
    legend('CoM','CoM Trajectory', 'Limit', 'Location', 'best')

saveas(fig4,'3DQuasi_x.png')
%%
fig5 = figure;
set(fig5,'Position', [100,100,800,400])

plot(linspace(0,20,tf*7),positionCoM(3,:),'LineWidth',4)
hold on
plot(linspace(0,20,tf*7),CoM_star_Hist(3,:),'--','LineWidth',4)
hold on
plot(linspace(0,20,tf*7),CoM_star_Hist(3,:)-0.010,'--','color','black','LineWidth',2)
hold on
plot(linspace(0,20,tf*7),CoM_star_Hist(3,:)+0.01,'--','color','black','LineWidth',2)
grid on

    xlabel('Time (sec)');
    ylabel('X-Axis (m)');
    title('3D Quasistatic Y-axis CoM Trajectory Tracking')
    legend('CoM','CoM Trajectory', 'Limit', 'Location', 'best')

saveas(fig5,'3DQuasi_y.png')

%%
fig6 = figure;
set(fig6,'Position', [100,100,800,400])

plot(positionCoM(1,:),positionCoM(3,:),'LineWidth',4)
hold on
plot(CoM_star_Hist(1,:),CoM_star_Hist(3,:),'--','LineWidth',4)
% hold on
% plot(linspace(0,20,tf*7),CoM_star_Hist(3,:)-0.010,'--','color','black','LineWidth',2)
% hold on
% plot(linspace(0,20,tf*7),CoM_star_Hist(3,:)+0.01,'--','color','black','LineWidth',2)
grid on

    xlabel('Time (sec)');
    ylabel('X-Axis (m)');
    title('3D Quasistatic Y-axis CoM Trajectory Tracking')
    legend('CoM','CoM Trajectory', 'Limit', 'Location', 'best')

saveas(fig6,'3DQuasi_xy.png')

