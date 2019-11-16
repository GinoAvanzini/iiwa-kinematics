function [q,r] = pplan1(R)
%     T0 = R.fkine([0 0 0 0 0 0 0])
%     tr2rpy(T0)
%     R.base()
% qH = [-0.7540 0.8796 0.9425 0.6283 -0.1257 0.2513 0];
qH= [-0.7121 0.7540 0.2967 0.3770 0.1780 0.3770 -0.5];
TH = R.fkine(qH); % Posicion de home

T1 = transl(0.4, -0.7, 0.45) * troty(pi/2); % Punto de aproximacion al objetivo

T2 = T1 * transl(0.1, 0, 0);  % Punto objetivo

T3 = T1;                   % Punto de paso

T4 = T3 * transl(0,0.55,0);      % Punto arriba

pd1 = [TH(1:3,4)'
       T1(1:3,4)'
       T2(1:3,4)'];

fid1 = [tr2rpy(TH(1:3,1:3))
        tr2rpy(T1(1:3,1:3))
        tr2rpy(T2(1:3,1:3))];

[q1, qdot1, r1, rd1, rdd1, tmax1] = linctraj(pd1,fid1,R ,qH);

T5 = T4 * transl(0.1, 0, 0); % Bajo hasta posicion

pd2 = [T2(1:3,4)'
       T3(1:3,4)'
       T4(1:3,4)'
       T5(1:3,4)'];
fid2 = [tr2rpy(T2(1:3,1:3))
        tr2rpy(T3(1:3,1:3))
        tr2rpy(T4(1:3,1:3))
        tr2rpy(T5(1:3,1:3))];

[q2, qdot2, r2, rd2, rdd2, tmax2] = linctraj(pd2,fid2,R ,q1(end,:));

q2(1,:) = [];
qdot2(1,:) = [];
r2(1,:) = [];
rd2(1,:) = [];
rdd2(1,:) = [];

qt = [q2(end,1:6) q2(end,7)+pi-0.01];

qqd = [q2(end,:); qt ; q2(end,:)];


[q3,qdot3,r3,rd3,rdd3, tmax3] = jotraj(qqd,R);

q3(1,:) = [];
qdot3(1,:) = [];
r3(1,:) = [];
rd3(1,:) = [];
rdd3(1,:) = [];
   
T6 = T4;
T7 = transl(0.05,-0.1,0.1) * T4 ;
T8 = TH;

pd4 = [T5(1:3,4)'
    T6(1:3,4)'
    T7(1:3,4)'
    T8(1:3,4)'];
fid4 = [tr2rpy(T5(1:3,1:3))
    tr2rpy(T6(1:3,1:3))
    tr2rpy(T7(1:3,1:3))
    tr2rpy(T8(1:3,1:3))];

[q4, qdot4, r4, rd4, rdd4, tmax4] = linctraj(pd4,fid4,R ,q3(end,:));

q4(1,:) = [];
qdot4(1,:) = [];
r4(1,:) = [];
rd4(1,:) = [];
rdd4(1,:) = [];


qt5 = [q4(end,1)-pi/6 q4(end,2:7)];

qqd5 = [q4(end,:); qt5 ];


[q5,qdot5,r5,rd5,rdd5, tmax5] = jotraj(qqd5,R);

q5(1,:) = [];
qdot5(1,:) = [];
r5(1,:) = [];
rd5(1,:) = [];
rdd5(1,:) = [];



qqd6 = [q5(end,:); q4(end,:) ];


[q6,qdot6,r6,rd6,rdd6, tmax6] = jotraj(qqd6,R);

q6(1,:) = [];
qdot6(1,:) = [];
r6(1,:) = [];
rd6(1,:) = [];
rdd6(1,:) = [];


q = [q1; q2; q3; q4; q5; q6];
qdot = [qdot1; qdot2; qdot3; qdot4; qdot5; qdot6];
r = [r1; r2; r3; r4; r5; r6];
rd = [rd1; rd2; rd3; rd4; rd5; rd6];
rdd = [rdd1; rdd2; rdd3; rdd4; rdd5; rdd6];

p = zeros(length(q(:,1)),3);
my = zeros(1,length(q(:,1)));

for i=1:length(q(:,1))
    Taux = R.fkine(q(i,:));
    p(i,:) = Taux(1:3,4)';
    my(i) = R.maniplty(q(i,:), 'yoshikawa');
end


t = 0 : 0.05 : tmax1+tmax2+tmax3+tmax4+tmax5+tmax6;

% % 
% figure(1)
% subplot(3,1,1);
% qplot7(t,q);
% title('Posición Articular');
% ylabel('rad')
% subplot(3,1,2);
% qplot7(t,qdot);
% title('Velocidad Articular Jacobiano');
% ylabel('rad/s')
% subplot(3,1,3);
% qdotn = diff(q)/0.05;
% qdotn = [[0 0 0 0 0 0 0]; qdotn];
% qplot7(t,qdotn);
% title('Velocidad Articular Numerica orden 1');
% ylabel('rad/s')

%     xlabel('t')
% 
% figure(2)
% subplot(3,1,1);
% hold on;
% plot(t, r(:,1:3));
% title('Posición Cartesiana');
% legend('x', 'y', 'z');
% ylabel('m')
% subplot(3,1,2);
% hold on;
% plot(t,rd(:,1:3));
% title('Velocidad Cartesiana');
% legend('x', 'y', 'z');
% ylabel('m/s')
% subplot(3,1,3);
% hold on;
% plot(t,rdd(:,1:3));
% title('Aceleracion Cartesiana');
% legend('x', 'y', 'z');
% ylabel('m/s^2')
% xlabel('time (s)')
% figure(3)
% title('Manipulability');
% plot(t,my);
% legend('my');


% figure(4)
% title('Trayectoria')
% plot3(p(:,1),p(:,2),p(:,3), 'r*');
% hold on;
% R.plot(q(1,:));
% R.animate(q);


end