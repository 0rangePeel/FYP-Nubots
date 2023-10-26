function [] = ZMP_custom_plot(CoMpc,param,eeL,eeR,ee,CoM,HT,ZMPPlot)
%% Plot Forward Kinemtaics
    rBRB = HT.AbRB(1:3,4);
    rBLB = HT.AbLB(1:3,4);
    rB1 = HT.Ab1(1:3,4);
    rB2 = HT.Ab2(1:3,4);
    rB3 = HT.Ab3(1:3,4);
    rB4 = HT.Ab4(1:3,4);
    rB5 = HT.Ab5(1:3,4);
    rB6 = HT.Ab6(1:3,4);

    hold on
    plot3([rBRB(3) rB3(3)],[rBRB(1) rB3(1)],[rBRB(2) rB3(2)],'Black','LineWidth',3)
    hold on
    plot3([rB3(3) rB2(3)],[rB3(1) rB2(1)],[rB3(2) rB2(2)],'Black','LineWidth',3)
    hold on
    plot3([rB2(3) rB1(3)],[rB2(1) rB1(1)],[rB2(2) rB1(2)],'Black','LineWidth',3)
    hold on
    plot3([rB1(3) rB4(3)],[rB1(1) rB4(1)],[rB1(2) rB4(2)],'Black','LineWidth',3)
    hold on
    plot3([rB4(3) rB5(3)],[rB4(1) rB5(1)],[rB4(2) rB5(2)],'Black','LineWidth',3)
    hold on
    plot3([rB5(3) rB6(3)],[rB5(1) rB6(1)],[rB5(2) rB6(2)],'Black','LineWidth',3)
    hold on
    plot3([rB6(3) rBLB(3)],[rB6(1) rBLB(1)],[rB6(2) rBLB(2)],'Black','LineWidth',3)

%% Plot CoM 
    % rBP = CoM.AbP(1:3,4);
    % rRA = CoM.AbRA(1:3,4);
    % rRT = CoM.AbRT(1:3,4);
    % rRF = CoM.AbRF(1:3,4);
    % rLF = CoM.AbLF(1:3,4);
    % rLT = CoM.AbLT(1:3,4);
    % rLA = CoM.AbLA(1:3,4);
    % 
    OutputCoM = CoM.Output(1:3,4);
    % 
    % plot3(rBP(3),rBP(1),rBP(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    % plot3(rRA(3),rRA(1),rRA(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    % plot3(rRT(3),rRT(1),rRT(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    % plot3(rRF(3),rRF(1),rRF(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    % plot3(rLF(3),rLF(1),rLF(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    % plot3(rLT(3),rLT(1),rLT(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    % plot3(rLA(3),rLA(1),rLA(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    h1 =plot3(OutputCoM(3),OutputCoM(1),OutputCoM(2),'o','Markersize',10,'LineWidth',3, 'Color', 'r');

%% Plot ZMP
    h2 = plot3(ZMPPlot(3),ZMPPlot(1),ZMPPlot(2),'*','Markersize',10,'color','blue',LineWidth=3); 

%% Reference Points
    h3 = plot3(0,CoMpc,param.zc,'*','Markersize',10,'color','m',LineWidth=3); 
    hold on  
    % plot3(0,eeL(1),eeL(2),'*','color','cyan',LineWidth=2); 
    % plot3(0,eeR(1),eeR(2),'*','color','green',LineWidth=2); 
    % plot3(0,ee(1),ee(2),'.','color','black','LineWidth',2); 

%% Graphing Setup   
    % view(135, 15);
    view(90, 0);
    grid on;
    set(gca, 'Layer', 'bottom'); % Set the grid lines to appear behind the data points
    axis([     -0.4,        0.4, ... % Z
               -0.2,       +1.0, ... % X
              -0.02,        0.8]);   % Y
    axis equal
    xlabel('Y-Axis');
    ylabel('X-Axis');
    zlabel('Z-Axis')
    title('Sagittal (Right side) View of 2D ZMP Walking');
    legend([h1,h2,h3],'CoM','ZMP','CoM Trajectory')
end

