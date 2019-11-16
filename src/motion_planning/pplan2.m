function [q,r] = pplan2(R)

qH = [-0.7714 pi/4 0 pi/4 0 pi/2 0];
TH = R.fkine(qH);
             
T4 =   [-0.7169    0.6971         0      0.4;
         0.6971    0.7169         0     -0.15;
         0         0         -1       0.41;
         0         0         0         1];   
     

     
     
T5 = [  -0.7169    0.6971        0      0.4;
         0.6971    0.7169        0     -0.15;
         0         0         -1         0.35;
         0         0         0         1];
                  

T6 = T5 * transl(0.05,0,0);

pd1 = [TH(1:3,4)'
       T4(1:3,4)'
       T5(1:3,4)'
       T6(1:3,4)'];

fid1 = [tr2rpy(T4(1:3,1:3))
        tr2rpy(T4(1:3,1:3))        
        tr2rpy(T4(1:3,1:3))
        tr2rpy(T4(1:3,1:3))
        tr2rpy(T4(1:3,1:3))];

[q1, qdot1, r1, rd1, rdd1, tmax1] = linctraj(pd1,fid1,R ,qH);


[q2, qdot2, r2, rd2, rdd2, tmax2] = cirtraj(T6,T5,R ,q1(end,:));

q2(1,:) = [];
qdot2(1,:) = [];
r2(1,:) = [];
rd2(1,:) = [];
rdd2(1,:) = [];

pd3 = [T6(1:3,4)'
       T4(1:3,4)'
       TH(1:3,4)'];

fid3 = [tr2rpy(T6(1:3,1:3))
        tr2rpy(T4(1:3,1:3))
        tr2rpy(TH(1:3,1:3))];

[q3, qdot3, r3, rd3, rdd3, tmax3] = linctraj(pd3,fid3,R ,q2(end,:));


q3(1,:) = [];
qdot3(1,:) = [];
r3(1,:) = [];
rd3(1,:) = [];
rdd3(1,:) = [];

q = [q1; q2; q3];
qdot = [qdot1; qdot2; qdot3];
r = [r1; r2; r3];
rd = [rd1; rd2; rd3];
rdd = [rdd1; rdd2; rdd3];

p = zeros(length(q(:,1)),3);
my = zeros(1,length(q(:,1)));
for i=1:length(q(:,1))
    Taux = R.fkine(q(i,:));
    p(i,:) = Taux(1:3,4)';
    my(i) = R.maniplty(q(i,:), 'yoshikawa');
end


t = 0 : 0.05 : tmax1+tmax2+tmax3;


% figure(4)
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
% % 
% figure(5)
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
% 
% figure(6)
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