%Primeiro, a função Link é usada para criar cada elo. 
% Os parâmetros da função Link são exatamente os parâmetros
% de Denavit-Hartemberg do elo em questão, nesta ordem: 
% theta = ângulo da junta (rad)
% d = deslocamento do elo (m)
% a = comprimento do elo (m)
% alpha = torção do elo (rad)
% sigma = tipo de junta (0: rotativa ou 1: prismática)
clear all
close all
clc
startup_rvc;
% rosshutdown;

%% Conexao com ROS

%setenv('ROS_MASTER_URI','http://192.168.0.17:11311')
%setenv('ROS_IP','192.168.0.17')

%rosinit;

%% Criação dos Links para o braço de 6-juntas
L(1) = Revolute('a', 0, 'alpha', pi/2, 'd', 0.125, 'qlim', [-2.79, 2.79]);
L(2) = Revolute('a', 0.21, 'alpha', 0, 'd', 0, 'offset', pi/2, 'qlim', [-2.09, 2.09]);
L(3) = Revolute('a', -0.075, 'alpha', -pi/2, 'd', 0, 'offset', -pi/2, 'qlim', [0.33, 2.79]);
L(4) = Revolute('a', 0, 'alpha', pi/2, 'd', 0.21, 'qlim', [-2.79, 2.79]);
L(5) = Revolute('a', 0, 'alpha', -pi/2, 'd', 0, 'qlim', [-2.09, 2.09]);
L(6) = Revolute('a', 0, 'alpha', 0, 'd', 0.07, 'qlim', [-6.28, 6.28]);


robot = SerialLink(L, 'name', 'Denso');
% Config inicial para o Denso
q = [0 0 -pi/2 0 0 0];

% Matriz de transformação por cinemática direta(Config Inicial)
Kd = robot.fkine(q);
% q()s gerados por cinemática inversa(Config Inicial)
Ki = robot.ikunc(Kd);

%%

X = 0.22;
Y = -0.15;
Z = 0.1;

limY = 0.1;
limZ = 0.02;
vy = 0.03;
vz = 0.03;
dir = 1;

%%
%Ponto acima do cubo
x1 = X + 0.1;
y1 = Y + 0.02;
z1 = Z - 0.025 + 0.25 - 0.115;
t1 = transl(x1, y1, z1);
t1(1:3,1:3) = [-1 0 0;0 1 0;0 0 -1];

%Cinematica inversa
Kinit = robot.ikunc(t1);

% Essas trajetórias serão enviadas para o denso como angulos por cinemática
% inversa
t = 30;
traj = jtraj(q, Kinit, t);

for i = 1:1:t
    
    stheta1 = num2str(traj(i,1));
    stheta2 = num2str(traj(i,2));
    stheta3 = num2str(traj(i,3));
    stheta4 = num2str(traj(i,4));
    stheta5 = num2str(traj(i,5));
    stheta6 = num2str(traj(i,6));
    pause(0.1)
    set_param('projetoRobotica/theta1','Value',stheta1);
    set_param('projetoRobotica/theta2','Value',stheta2);
    set_param('projetoRobotica/theta3','Value',stheta3);
    set_param('projetoRobotica/theta4','Value',stheta4);
    set_param('projetoRobotica/theta5','Value',stheta5);
    set_param('projetoRobotica/theta6','Value',stheta6);
    
end

%%

for k = 1:3
    Ki1 = Kinit;
    x2 = x1;
    y2 = y1;
    z2 = z1;
    t = 5;
    while z2 > limZ
        if y2 <= -1 * limY && dir == -1
            dir = 1;
        elseif y2 >= limY && dir == 1
            dir = -1;
        end

        y2 = y2 + dir * vy
        z2 = z2 - vz;

        t2 = transl(x2, y2, z2);
        t2(1:3,1:3) = [-1 0 0;0 1 0;0 0 -1];
        Ki2 = robot.ikunc(t2);
        traj2 = jtraj(Ki1, Ki2, t);

        for i = 1:1:t

            stheta1 = num2str(traj2(i,1));
            stheta2 = num2str(traj2(i,2));
            stheta3 = num2str(traj2(i,3));
            stheta4 = num2str(traj2(i,4));
            stheta5 = num2str(traj2(i,5));
            stheta6 = num2str(traj2(i,6));
            pause(0.05)
            set_param('projetoRobotica/theta1','Value',stheta1);
            set_param('projetoRobotica/theta2','Value',stheta2);
            set_param('projetoRobotica/theta3','Value',stheta3);
            set_param('projetoRobotica/theta4','Value',stheta4);
            set_param('projetoRobotica/theta5','Value',stheta5);
            set_param('projetoRobotica/theta6','Value',stheta6);

        end

        Ki1 = Ki2;

    end
    set_param('projetoRobotica/gripper','Value','0.43');
    pause(3)

    z3 = z2 + 0.1;
    t3 = transl(x2, y2, z3);
    t3(1:3,1:3) = [-1 0 0;0 1 0;0 0 -1];
    Ki3 = robot.ikunc(t3);
    t = 10;
    traj3 = jtraj(Ki2, Ki3, t);

    xgoal = 0.2;
    ygoal = 0.15;
    zgoal = z2;

    t4 = transl(xgoal, ygoal, z3);
    t4(1:3,1:3) = [-1 0 0;0 1 0;0 0 -1];
    Ki4 = robot.ikunc(t4);
    traj4 = jtraj(Ki3, Ki4, t);

    t5 = transl(xgoal, ygoal, zgoal);
    t5(1:3,1:3) = [-1 0 0;0 1 0;0 0 -1];
    Ki5 = robot.ikunc(t5);
    traj5 = jtraj(Ki4, Ki5, t);

    traj6 = jtraj(Ki5, Ki4, t);
    t2 = 20;
    traj7 = jtraj(Ki4, Kinit, t2);

    trajF = [traj3;traj4;traj5;traj6;traj7];

    for i = 1:1:t*6
        stheta1 = num2str(trajF(i,1));
        stheta2 = num2str(trajF(i,2));
        stheta3 = num2str(trajF(i,3));
        stheta4 = num2str(trajF(i,4));
        stheta5 = num2str(trajF(i,5));
        stheta6 = num2str(trajF(i,6));
        pause(0.1)
        set_param('projetoRobotica/theta1','Value',stheta1);
        set_param('projetoRobotica/theta2','Value',stheta2);
        set_param('projetoRobotica/theta3','Value',stheta3);
        set_param('projetoRobotica/theta4','Value',stheta4);
        set_param('projetoRobotica/theta5','Value',stheta5);
        set_param('projetoRobotica/theta6','Value',stheta6);

        if i == t*3
            pause(1)
            set_param('projetoRobotica/gripper','Value','0');
            pause(3)
        end
    end
end

t =10;
trajfinal = jtraj(Kinit,q,t);
for i = 1:1:t

    stheta1 = num2str(trajfinal(i,1));
    stheta2 = num2str(trajfinal(i,2));
    stheta3 = num2str(trajfinal(i,3));
    stheta4 = num2str(trajfinal(i,4));
    stheta5 = num2str(trajfinal(i,5));
    stheta6 = num2str(trajfinal(i,6));
    pause(0.05)
    set_param('projetoRobotica/theta1','Value',stheta1);
    set_param('projetoRobotica/theta2','Value',stheta2);
    set_param('projetoRobotica/theta3','Value',stheta3);
    set_param('projetoRobotica/theta4','Value',stheta4);
    set_param('projetoRobotica/theta5','Value',stheta5);
    set_param('projetoRobotica/theta6','Value',stheta6);

end
