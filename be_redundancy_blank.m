clear all
close all

% Param robot
Dx = 0.1;
a=0;
b=0;
c=0;
h=0.4;
r=0.4;
f=1;

param_robot = [Dx, a, b, c, h, r, f];

% Points de la cible

OP1=[10 2.2 1 1]';
OP2=[10 1.8 1 1]';
OP3=[10 1.8 0.6 1]';
OP4=[10 2.2 0.6 1]';

% Config initiale
Q1 = zeros(4,1);
Q2 = [0 2 0 0]';
Q3 = [0 0 0 pi/6]';
Q4 = [2 0 -pi/3 pi/2]';
Q5 = [2 0 -2*pi/3 5*pi/6]';
Q=[Q1 Q2 Q3 Q4 Q5];
selec = 1;

X(1)=Q(1,selec);
Y(1)=Q(2,selec);
Tr(1)=Q(3,selec);
qpl(1)=Q(4,selec);

q = Q(:, selec);

% Param obstacles
pos_obst_1 = [6 1.5]';
pos_obst_2 = [2 1]';
pos_obst_3 = [4 1]';

pos_obst = pos_obst_3;

r_obst = 0.2;
d0 = 0.5;
d_switch = 1;

[d, alpha] = distAndAlpha(q, pos_obst, param_robot);

% Gains de la loi de commande
lambda=0.5;
beta = 1e1;

% Periode d'echantillonnage
Te=1e-2;

% Indices visuels de reference
et = 0.2;
X1et = -et;
Y1et = et;
X2et = -et;
Y2et = -et;
X3et = et;
Y3et = -et;
X4et = et;
Y4et = et;

s_star = [X1et Y1et X2et Y2et X3et Y3et X4et Y4et]';

% Param complementaires

Eps = 1e-6;
k = 1;
Cond = 1;

while Cond==1

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%  Measure sim  %%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [s, L] = visu(q, param_robot, OP1, OP2, OP3, OP4);

    [d, alpha] = distAndAlpha(q, pos_obst, param_robot);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%  Errors/Jacs calcs %%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    e = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%% Plot measures %%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    X1(k) = s(1); Y1(k) = s(2);
    X2(k) = s(3); Y2(k) = s(4);
    X3(k) = s(5); Y3(k) = s(6);
    X4(k) = s(7); Y4(k) = s(8);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%% Command algorithms %%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    qdot = [0; 0; 0];

    % Extraction des commandes pour affichage
    v(k) = qdot(1);
    w(k) = qdot(2);
    wpl(k) = qdot(3);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%% Dynamics sim %%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    % Estimation de l'etat
  
    X(k+1) = X(k) + Te * v(k) * cos(Tr(k));
    Y(k+1) = Y(k) + Te * v(k) * sin(Tr(k));
    Tr(k+1) = Tr(k) + Te * w(k);
    qpl(k+1) = qpl(k) + Te * wpl(k);

    q = [X(k+1); Y(k+1); Tr(k+1); qpl(k+1)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%% Loop & Stopping cond %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    k = k+1;
  
    if norm(e) < Eps | k > 1000
        Cond=0;
    end
end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% Plots %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Robot traj
figure
for i=1:10:length(X)
    drawRobotPlatine(X(i),Y(i),Tr(i),qpl(i))
    drawTarget(OP1,OP2,OP3,OP4, pos_obst, r_obst)
    hold on
end

% Image traj
drawImage(X1,Y1,X2,Y2,X3,Y3,X4,Y4)

tps=0:Te:(length(v)-1)*Te;

% Measure evolution

% figure
% plot(tps,v)
% title('Vitesse linéaire de la base')

% figure 
% plot(tps,w)
% title('Vitesse angulaire de la base')

% figure
% plot(tps,wpl)
% title('Vitesse angulaire de la platine')

