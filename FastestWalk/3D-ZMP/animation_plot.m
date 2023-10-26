function [] = animation_plot(HT,CoM,CoMPlot,ZMPPlot,eeL,eeR,eeB)
%% Plot Forward Kinemtaics
    rBRB = HT.AbRB(1:3,4);
    rBLB = HT.AbLB(1:3,4);
    rB0 = HT.Ab0(1:3,4);
    rB1 = HT.Ab1(1:3,4);
    rB2 = HT.Ab2(1:3,4);
    rB3 = HT.Ab3(1:3,4);
    rB4 = HT.Ab4(1:3,4);
    rB5 = HT.Ab5(1:3,4);
    rB6 = HT.Ab6(1:3,4);
    rB7 = HT.Ab7(1:3,4);
    rB8 = HT.Ab8(1:3,4);
    rB9 = HT.Ab9(1:3,4);
    rB10 = HT.Ab10(1:3,4);
    rB11 = HT.Ab11(1:3,4);
    rB12 = HT.Ab12(1:3,4);


    hold on
    plot3([rBRB(3) rB6(3)],[rBRB(1) rB6(1)],[rBRB(2) rB6(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB6(3) rB5(3)],[rB6(1) rB5(1)],[rB6(2) rB5(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB5(3) rB4(3)],[rB5(1) rB4(1)],[rB5(2) rB4(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB4(3) rB3(3)],[rB4(1) rB3(1)],[rB4(2) rB3(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB3(3) rB2(3)],[rB3(1) rB2(1)],[rB3(2) rB2(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB2(3) rB1(3)],[rB2(1) rB1(1)],[rB2(2) rB1(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB1(3) rB0(3)],[rB1(1) rB0(1)],[rB1(2) rB0(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB0(3) rB7(3)],[rB0(1) rB7(1)],[rB0(2) rB7(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB7(3) rB8(3)],[rB7(1) rB8(1)],[rB7(2) rB8(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB8(3) rB9(3)],[rB8(1) rB9(1)],[rB8(2) rB9(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB9(3) rB10(3)],[rB9(1) rB10(1)],[rB9(2) rB10(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB10(3) rB11(3)],[rB10(1) rB11(1)],[rB10(2) rB11(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB11(3) rB12(3)],[rB11(1) rB12(1)],[rB11(2) rB12(2)],'color','black','LineWidth',3)
    hold on
    plot3([rB12(3) rBLB(3)],[rB12(1) rBLB(1)],[rB12(2) rBLB(2)],'color','black','LineWidth',3)

    %% Plot CoM
    OutputCoM = CoM.Output(1:3,4);
    plot3(OutputCoM(3),OutputCoM(1),OutputCoM(2),'*','Color','r','Markersize',10);

    %% Plot CoM*
    plot3(CoMPlot(3),CoMPlot(1),CoMPlot(2),'*','color','m','Markersize',10,'LineWidth',2); 
    hold on
    plot3([CoMPlot(3) CoMPlot(3)],[CoMPlot(1)-0.02 CoMPlot(1)+0.02],[CoMPlot(2) CoMPlot(2)],'color','m','LineWidth',2)
    plot3([CoMPlot(3)-0.02 CoMPlot(3)+0.02],[CoMPlot(1) CoMPlot(1)],[CoMPlot(2) CoMPlot(2)],'color','m','LineWidth',2)

    %% Plot ZMP
    plot3(ZMPPlot(3),ZMPPlot(1),ZMPPlot(2),'*','Markersize',10,'color','blue',LineWidth=3); 

    %% Plot eeL
    % plot3(eeL(3),eeL(1),eeL(2),'*','color','green');

    %% Plot eeR
    % plot3(eeR(3),eeR(1),eeR(2),'*','color','blue');

    %% Plot eeB
    % plot3(eeB(3),eeB(1),eeB(2)-0.01,'*','color','cyan');

    %% Plot Feet
    plotFoot(HT.AbRB,rBRB,'r');
    plotFoot(HT.AbLB,rBLB,'b');


    %% Graphing Setup  
    % view(90, 90);
    % view(90, 0);
    view(135, 25);
    grid on;
    set(gca, 'Layer', 'bottom'); % Set the grid lines to appear behind the data points
    axis equal
    % axis([     -0.4,        0.4, ... % Z
    %            -0.2,       +0.8, ... % X
    %           -0.02,        1.0]);   % Y
        axis([     -0.1,        0.3, ... % Z
               -0.2,       +1.5, ... % X
              -0.02,        0.6]);   % Y
        % view(135, 45);
    % grid on;
    % set(gca, 'Layer', 'bottom'); % Set the grid lines to appear behind the data points
    % axis([     -0.4,        0.4, ... % Z
    %            -0.2,       +0.8, ... % X
    %           -0.02,        1.0]);   % Y

    title('3D ZMP Animation')
    xlabel('Y-Axis');
    ylabel('X-Axis');
    zlabel('Z-Axis');
end

