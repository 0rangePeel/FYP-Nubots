function [] = kinematics_plot(HT,CoM)
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

    %% Plot Feet
    plotFoot(HT.AbRB,rBRB,'r');
    plotFoot(HT.AbLB,rBLB,'b');

    %% Plot CoM Positions
    r01 = CoM.Ab01(1:3,4);
    r12 = CoM.Ab12(1:3,4);
    r23 = CoM.Ab23(1:3,4);
    r34 = CoM.Ab34(1:3,4);
    r45 = CoM.Ab45(1:3,4);
    r56 = CoM.Ab56(1:3,4);
    r6RB = CoM.Ab6RB(1:3,4);

    r07 = CoM.Ab07(1:3,4);
    r78 = CoM.Ab78(1:3,4);
    r89 = CoM.Ab89(1:3,4);
    r910 = CoM.Ab910(1:3,4);
    r1011 = CoM.Ab1011(1:3,4);
    r1112 = CoM.Ab1112(1:3,4);
    r12LB = CoM.Ab12LB(1:3,4);

    plot3(r01(3),r01(1),r01(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r12(3),r12(1),r12(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r23(3),r23(1),r23(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r34(3),r34(1),r34(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r45(3),r45(1),r45(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r56(3),r56(1),r56(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r6RB(3),r6RB(1),r6RB(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');

    plot3(r07(3),r07(1),r07(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r78(3),r78(1),r78(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r89(3),r89(1),r89(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r910(3),r910(1),r910(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r1011(3),r1011(1),r1011(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r1112(3),r1112(1),r1112(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');
    plot3(r12LB(3),r12LB(1),r12LB(2),'*','Markersize',10,'LineWidth',1.2, 'Color', 'm');

    OutputCoM = CoM.Output(1:3,4);
    plot3(OutputCoM(3),OutputCoM(1),OutputCoM(2),'o','Markersize',10,'LineWidth',3, 'Color', 'r');


    % 
    % 
    % %% Plot CoM_star
    % plot3(CoM_star(3),CoM_star(1),CoM_star(2),'*','color','red'); 
    % 
    % plot3([CoM_star(3) CoM_star(3)],[CoM_star(1)-0.02 CoM_star(1)+0.02],[CoM_star(2) CoM_star(2)],'color','red')
    % plot3([CoM_star(3)-0.02 CoM_star(3)+0.02],[CoM_star(1) CoM_star(1)],[CoM_star(2) CoM_star(2)],'color','red')
    % 
    % %% Plot CoM Traj
    % plot3(CoM_star_traj(3,:),CoM_star_traj(1,:),CoM_star_traj(2,:),'color','black')
    % 
    % %% Plot Foot Trajectory
    % plot3(xe_star(3,:),xe_star(1,:),xe_star(2,:),'color','black')


    %% Graphing Setup   
    view(135, 25);
    % view(90, 0);
    % grid on;
    % set(gca, 'Layer', 'bottom'); % Set the grid lines to appear behind the data points
    % axis([     -0.4,        0.4, ... % Z
    %            -0.2,       +0.8, ... % X
    %           -0.02,        1.0]);   % Y
    xlabel('Y-Axis');
    ylabel('X-Axis');
    zlabel('Z-Axis');

    % view(90, 0);
    grid on;
    set(gca, 'Layer', 'bottom'); % Set the grid lines to appear behind the data points
    % axis([     -0.4,        0.4, ... % Z
    %            -0.2,       +0.8, ... % X
    %           -0.02,        1.0]);   % Y

        axis equal
        axis([     -0.10,        0.3, ... % Z
               -0.15,       +0.25, ... % X
              -0.00,        0.6]);   % Y

        title('Orthogonal View of 3D FKM with CoM')
        % title('Sagittal (Right side) View of 3D FKM with CoM')
    
end

