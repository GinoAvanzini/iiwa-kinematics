%% Script para validación numérica de la cinemática inversa

clc
close all, clear variables

DH = [0 .360 0 -pi/2 0
    0 0 0 pi/2 0
    0 .420 0 -pi/2, 0
    0 0 0 pi/2 0
    0 .400 0 -pi/2 0
    0 0 0 pi/2 0
    0 .126 0 0 0];

R = SerialLink(DH, 'name', 'KUKA iiwa 14 R820');

% R.base = transl(.5, .25, 0);

q = [0, 0, 0, 0, 0, 0, 0];
workspace = [-2, 2, -2, 2, -2, 3];

% R.plot(q, 'workspace', workspace, 'scale', 0.5)

R.qlim = deg2rad([-170, 170
                -120, 120
                -170, 170
                -120, 120
                -170, 170
                -120, 120
                -175, 175]);

a = deg2rad(-120);
b = deg2rad(120);
       
n = 1000;
q = a + (b-a)*rand(7,1);

T = R.fkine(q);

T_list = zeros(n, 4, 4);

% q_error = zeros(n, 7, 1);
T_error = zeros(n, 1);
q_error = zeros(n, 1);

for i=1:n
    q = a + (b-a)*rand(7,1);
    T = R.fkine(q);
    q_inv = inverse_kinematics(q(3), T, R);

    T_temp = R.fkine(q);

    for j=1:8
       T_error(i) = T_error(i) + sum(sum(abs( T - R.fkine(q_inv(:, j) ) ) ) );    
    end
    
    q_error(i) = sum(abs(alikeness(q, q_inv) - q));
    
end


%%
plot(q_error)
% pause
% plot(T_error)




% q_inversa = inverse_kinematics(q(3), T, R);
% 
% q_sol = alikeness(q, q_inversa)
% 
% q
% q_sol




%
