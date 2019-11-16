%% Iiwa trajectory
clear;
clf;
close all;
clc;

% Matriz Denavit–Hartenberg
DH = [0 0.36  0 -pi/2;
      0 0     0 pi/2;
      0 0.42   0 -pi/2;
      0 0     0 pi/2;
      0 0.4   0 -pi/2;
      0 0     0 pi/2;
      0 0.126 0 0];


% Objeto serial link 
iiwa1 = SerialLink(DH,'name','iiwa1');
iiwa2 = SerialLink(DH,'name','iiwa2');
% Offset del robot 
iiwa1.offset = [0 0 0 0 0 0 0];
iiwa2.offset = [0 0 0 0 0 0 0];

% limites de las articulaciones
iiwa1.qlim = [deg2rad(-170), deg2rad(170);
            deg2rad(-120), deg2rad(120);
            deg2rad(-170), deg2rad(170);
            deg2rad(-120), deg2rad(120);
            deg2rad(-170), deg2rad(170);
            deg2rad(-120), deg2rad(120);
            deg2rad(-175), deg2rad(175)];
 
iiwa2.qlim = iiwa1.qlim;
iiwa2.base = [1 0 0 0.5;
              0 1 0 0.5;
              0 0 1 0;
              0 0 0 1];
        
% Planificacion de trayectoria robot 1
[q1, r1] = pplan1(iiwa1); 
% Planificacion de trayectoria robot 2
[q2, r2] = pplan2(iiwa2);

fig = figure(1);
    plot3(r1(:,1),r1(:,2),r1(:,3),'r');
    hold on;
    plot3(r2(:,1),r2(:,2),r2(:,3),'b');

    iiwa1.plot(q1(1,:));
    iiwa2.plot(q2(1,:));

iiwa1.animate(q1);
iiwa2.animate(q2);