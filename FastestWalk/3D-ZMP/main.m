clc
clear all
% Helpful time keeping :)
tic
hbar = waitbar(0,'Simulation Progress');

%% Starting Parameters
q = initialSystem;

%% Parameters
param = parametersSystem;

%% Control
control = parametersControl;

%% Model
% t = control.stepNum*0.575;
t = control.stepNum*0.50;
model = parametersModel(t,param,control);

%% Run Kinematics to Find CoM
HT = kinematics_3D(q,param,model);
model.rRBb = HT.AbRB(1:3,4); % Right foot location
model.rLBb = HT.AbLB(1:3,4); % Left  foot location
xe = endEffector(HT,model);
[FinalCoM,CoMparam] = CoM(HT,param,model,q);

%% Initial Conditions
model.x0 = [FinalCoM(1);    % CoM X position
            0;              % CoM X Velocity
            0;              % CoM X Acceleration
            FinalCoM(3);    % CoM Z Position
            0;              % CoM Z Velocity
            0];             % CoM Z Acceleration

model.x             = [FinalCoM(1)*ones(1,length(model.tspan)); zeros(2,length(model.tspan));FinalCoM(3)*ones(1,length(model.tspan)); zeros(2,length(model.tspan))];
model.y             = [FinalCoM(1)*ones(1,length(model.tspan)); zeros(1,length(model.tspan))];

%% Calculate Trajectories for ZMP Ref, End Effector, Base Foot Switch
CoM_star = trajCoM(model.tspan,control,param,model,FinalCoM);
model.CoM_star = [CoM_star(1,:); CoM_star(3,:)]; 
% figure(1)
% plot(model.tspan,CoM_star(1,:));
% figure(2)
% plot(model.tspan,-CoM_star(3,:));
% figure(3)
% plot(CoM_star(1,:),-CoM_star(3,:));
[eeL,eeR,ee] = trajEE(model.tspan,control,param,model);
% figure(4)
% plot(model.tspan,eeL);
% figure(5)
% plot(model.tspan,eeR);
% figure(6)
% plot(model.tspan,ee);
[base,~] = trajBase(model.tspan,control,param,model);

if base(1) == 1
    eeB = model.rRBb;
else
    eeB = model.rLBb;
end

xe_n = [ee ; zeros(3,(length(model.tspan)))];
[~,outputCoM] = CoM(HT,param,model,q);
HT_Hist = HT;
ee_Hist = zeros(6,length(model.tspan));
save_q = (q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)])';

kinematics_plot(HT,CoMparam);

%% Simulation
for i = 1:(length(model.tspan))
    model.base = base(i);
    % model.rRBb = eeB(:,i);
    % model.rLBb = eeB(:,i);
    [ZMPpc(:,i), CoMpc(:,i), model, control] = LIPM_3D(param,control,model,i);
    CoMinvK = [CoMpc(1,i); param.zc;CoMpc(4,i)];
    q = invKinematics(q,param,model,xe_n(:,i),CoMinvK);
    q_temp = q.*[-1*ones(2,1);ones(3,1);-1*ones(1,1);ones(2,1);-1*ones(3,1);ones(1,1)];
    save_q = [save_q;q_temp'];
    HT = kinematics_3D(q,param,model);
    model.rRBb = HT.AbRB(1:3,4); % Right foot location
    model.rLBb = HT.AbLB(1:3,4); % Left  foot location
    if model.base == 1 
        eeB = [eeB model.rRBb];
    else
        eeB = [eeB model.rLBb];
    end
    HT_Hist = [HT_Hist HT];
    xe = endEffector(HT,model);
    ee_Hist(:,i) = xe(:,1);
    [positionCoM(:,i),temp] = CoM(HT,param,model,q);
    outputCoM = [outputCoM temp];
    waitbar(i/(length(model.tspan)),hbar)
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


%% Animation
fig = 1;
figure(fig);

set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% Create and open video writer object
v = VideoWriter('ZMP_3D.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

qCounter = 0;
xeCounter = 0;
q_previous = save_q(1,:);

for i = 1:(length(model.tspan))
    clf(fig);
    CoMPlot = [CoMpc(1,i); param.zc;CoMpc(4,i)];
    ZMPPlot = [ZMPpc(1,i); 0;ZMPpc(2,i)];
    animation_plot(HT_Hist(:,i),outputCoM(:,i),CoMPlot,ZMPPlot,eeL(:,i),eeR(:,i),eeB(:,i));

    % Analysis
    % Check absolute differences and print a message
    q = save_q(i,:);
    for j = 1:length(q)
        absolute_difference = abs(q(j) - q_previous(j));
        if absolute_difference > deg2rad(5)
            fprintf('Absolute difference on iteration (%d) between q(%d) and q_star(%d) is greater than 2deg: %d\n', i , j, j, rad2deg(absolute_difference));
            qCounter = qCounter + 1;
        end
    end
%     rms = rmse(ee_Hist(:,i),xe_n(:,i));
%     if rms > 0.01
%         fprintf("EX/EE on Iter: %d RMS: %d\n", i, rms);
%         xeCounter = xeCounter + 1;
%     end
    rms = sqrt(mean(ee_Hist(:,i)-xe_n(:,i)).^2);
    if rms > 0.01
        fprintf("EX/EE on Iter: %d RMS: %d\n", i, rms);
        xeCounter = xeCounter + 1;
    end

    q_previous = save_q(i,:);

    pause(0.001)
    drawnow
    % frame = getframe(gcf);
    % writeVideo(v,frame);
       frame = getframe(gcf);
    resizedFrame = imresize(frame.cdata, [720, NaN]); % Preserve aspect ratio, set height to 720 pixels
    writeVideo(v, resizedFrame);
end
close(v);
fprintf("Animation Stop with qErrors: (%d) xeErrors: (%d)\n", qCounter, xeCounter);

%% Plot Steps
fig = 2;
figure(fig);
plot(model.tspan, model.CoM_star(1,:));
grid on
hold on 
plot(model.tspan,ZMPpc(1,:));
plot(model.tspan,CoMpc(1,:));
legend("Reference","ZMP","CoM");
saveas(gcf,'ZMP_3D_x.png')

%%
fig = 3;
figure(fig);
plot(model.tspan, model.CoM_star(2,:));
grid on
hold on 
plot(model.tspan,ZMPpc(2,:));
plot(model.tspan,CoMpc(4,:));
legend("Reference","ZMP","CoM");
saveas(gcf,'ZMP_3D_y.png')

%% CoM Position
fig = 4;
figure(fig);
% X Plot
subplot(3,1,1)
sgtitle('CoM Position')
plot(model.tspan,positionCoM(1,:),'LineWidth',2)
grid on
title('X')
hold on
subplot(3,1,1)
plot(model.tspan,CoMpc(1,:),'--','LineWidth',2)
hold on
subplot(3,1,1)
plot(model.tspan,CoMpc(1,:)-0.005,'LineWidth',2)
hold on
subplot(3,1,1)
plot(model.tspan,CoMpc(1,:)+0.005,'LineWidth',2)
% Y Plot
subplot(3,1,2)
plot(model.tspan,positionCoM(2,:),'LineWidth',2)
grid on
title('Y')
hold on
subplot(3,1,2)
plot(model.tspan,param.zc*ones(1,length(model.tspan)),'--','LineWidth',2)
hold on
subplot(3,1,2)
plot(model.tspan,0.4385*ones(1,length(model.tspan)),'LineWidth',2)
hold on
subplot(3,1,2)
plot(model.tspan,0.475*ones(1,length(model.tspan)),'LineWidth',2)
% Z Plot
subplot(3,1,3)
plot(model.tspan,positionCoM(3,:),'LineWidth',2)
grid on
title('Z')
hold on
subplot(3,1,3)
plot(model.tspan,CoMpc(4,:),'--','LineWidth',2)
hold on
subplot(3,1,3)
plot(model.tspan,CoMpc(4,:)-0.005,'LineWidth',2)
hold on
subplot(3,1,3)
plot(model.tspan,CoMpc(4,:)+0.005,'LineWidth',2)
saveas(gcf,'ZMP_3D_EE.png')