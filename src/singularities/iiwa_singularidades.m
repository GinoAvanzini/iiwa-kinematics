%% Aplicación del métodos propuestos para análisis de singularidades
clear; clc;
%% Matriz de transformación homogénea D-H genérica
syms theta_i d_i a_i alpha_i real;
disp(sym_transfDH([theta_i, d_i, a_i, alpha_i]));

%% Matriz de transformación homogénea de cada articulación respecto la anterior
q = sym('q', [1 7], 'real');

d1 = 0.360; d3 = 0.420; d5 = 0.400;
%d7 = 0.126;
d7 = 0;

DH = [
    q(1)   d1  0   -pi/2   0;
    q(2)   0   0   pi/2    0;
    q(3)   d3  0   -pi/2   0;
    q(4)   0   0   pi/2    0;
    q(5)   d5  0   -pi/2   0;
    q(6)   0   0   pi/2    0;
    q(7)   d7  0   0       0];

R = SerialLink(DH);

A0_1 = sym_transfDH(DH(1, :));
A1_2 = sym_transfDH(DH(2, :));
A2_3 = sym_transfDH(DH(3, :));
A3_4 = sym_transfDH(DH(4, :));
A4_5 = sym_transfDH(DH(5, :));
A5_6 = sym_transfDH(DH(6, :));
A6_7 = sym_transfDH(DH(7, :));

%J = R.jacob0(q);

%% Matrices homogéneas de cada eslabón
T0_1 = simplify(A0_1);
T0_2 = simplify(A0_1*A1_2);
T0_3 = simplify(A0_1*A1_2*A2_3);
T0_4 = simplify(A0_1*A1_2*A2_3*A3_4);
T0_5 = simplify(A0_1*A1_2*A2_3*A3_4*A4_5);
T0_6 = simplify(A0_1*A1_2*A2_3*A3_4*A4_5*A5_6);
T0_7 = simplify(A0_1*A1_2*A2_3*A3_4*A4_5*A5_6*A6_7);

%% JACOBIANO GEOMÉTRICO
% Traslación 
o = {T0_1(1:3, 4); T0_2(1:3, 4); T0_3(1:3, 4); T0_4(1:3, 4); T0_5(1:3, 4); 
    T0_6(1:3, 4); T0_7(1:3, 4)};
z = {T0_1(1:3, 3); T0_2(1:3, 3); T0_3(1:3, 3); T0_4(1:3, 3); T0_5(1:3, 3);
    T0_6(1:3, 3); T0_7(1:3, 3)};

Jg = [
    cross(z{1}, (o{7}-o{1})) cross(z{2}, (o{7}-o{2}))...
    cross(z{3}, (o{7}-o{3})) cross(z{4}, (o{7}-o{4}))...
    cross(z{5}, (o{7}-o{5})) cross(z{6}, (o{7}-o{6}))...
    cross(z{7}, (o{7}-o{7}));
    z{1} z{2} z{3} z{4} z{5} z{6} z{7}];

disp(Jg);

%% Matriz Jacobiana para los distintos sistemas de referencia

% Respecto al sistema de referencia {1}
T1_0 = inv(T0_1);
R1_0 = T1_0(1:3, 1:3);
B = [R1_0 zeros(3, 3); zeros(3, 3) R1_0];
J1_0 = B*Jg;

% Respecto al sistema de referencia {2}
T2_0 = inv(T0_2);
R2_0 = T2_0(1:3, 1:3);
B = [R2_0 zeros(3, 3); zeros(3, 3) R2_0];
J2_0 = B*Jg;

% Respecto al sistema de referencia {3}
T3_0 = inv(T0_3);
R3_0 = T3_0(1:3, 1:3);
B = [R3_0 zeros(3, 3); zeros(3, 3) R3_0];
J3_0 = B*Jg;

% Respecto al sistema de referencia {4}
T4_0 = inv(T0_4);
R4_0 = T4_0(1:3, 1:3);
B = [R4_0 zeros(3, 3); zeros(3, 3) R4_0];
J4_0 = B*Jg;

% Respecto al sistema de referencia {5}
T5_0 = inv(T0_5);
R5_0 = T5_0(1:3, 1:3);
B = [R5_0 zeros(3, 3); zeros(3, 3) R5_0];
J5_0 = B*Jg;

% Respecto al sistema de referencia {6}
T6_0 = inv(T0_6);
R6_0 = T6_0(1:3, 1:3);
B = [R6_0 zeros(3, 3); zeros(3, 3) R6_0];
J6_0 = B*Jg;

% Respecto al sistema de referencia {7}
T7_0 = inv(T0_7);
R7_0 = T7_0(1:3, 1:3);
B = [R7_0 zeros(3, 3); zeros(3, 3) R7_0];
J7_0 = B*Jg;

%% Singularidades de posición: Cálculo de menores de J11
J11 = J4_0(1:3, 1:4);

M1 = simplify(det(J11(1:3, 1:3)));
M2 = simplify(det(J11(1:3, 2:4)));
M3 = simplify(det([J11(1:3, 1) J11(1:3, 3:4)]));
M4 = simplify(det([J11(1:3, 1:2) J11(1:3, 4)]));

disp([M1; M2; M3; M4]);

%% Singularidades de orientación: det(J22) = 0
J22 = J4_0(4:6, 5:7);
detJ22 = simplify(det(J22));
disp(detJ22);

%% Singularidades acopladas
J21 = J4_0(4:6, 1:4);

M1 = simplify(det(J21(1:3, 1:3)));
M2 = simplify(det(J21(1:3, 2:4)));
M3 = simplify(det([J21(1:3, 1) J21(1:3, 3:4)]));
M4 = simplify(det([J21(1:3, 1:2) J21(1:3, 4)]));

disp("Menores J21:");
disp([M1; M2; M3; M4]);

Jw = J4_0(4:6, :);

M1 = simplify(det(Jw(1:3, 1:3)));
M2 = simplify(det(Jw(1:3, 2:4)));
M3 = simplify(det(Jw(1:3, 3:5)));
M4 = simplify(det(Jw(1:3, 4:6)));
M5 = simplify(det(Jw(1:3, 5:7)));
M6 = simplify(det([Jw(1:3, 1) Jw(1:3, 6:7)]));
M7 = simplify(det([Jw(1:3, 1:2) Jw(1:3, 7)]));

disp("Menores Jw:");
disp([M1; M2; M3; M4; M5; M6; M7]);

%% Evaluación de direcciones singulares
Ji_0 = {Jg; J1_0; J2_0; J3_0; J4_0; J5_0; J6_0; J7_0};

% Primera singularidad de posición
%auxQ = [q(1:3) 0 q(5:7)];
auxQ = [q(1) 0 -pi/2 q(4:7)];
%auxQ = [q(1) 0 0 0 0 q(6:7)];
for i=1:length(Ji_0)
    aux = simplify(subs(Ji_0{i}, q, auxQ));
    disp(["J" i-1]);
    disp(aux);
    disp("SVD");
    dGdl = 0;
    S = svd(aux);
    for j=1:length(S)
        if S(j)==0
            dGdl = dGdl + 1;
        end
    end
    disp(dGdl);
    pause();
end

%% Función matriz de transformación homogénea D-H

function T = sym_transfDH(paramDH)
    theta = paramDH(1);
    d = paramDH(2);
    a = paramDH(3);
    alpha = paramDH(4);
    T = sym_trotx(alpha) * transl(a, 0, 0) * transl(0, 0, d) ...
        * sym_trotz(theta);
end

%% Funciones de rotación simbólica

function TX = sym_trotx(alpha)
    TX = [
    1   0           0               0;
    0   cos(alpha)  -sin(alpha)     0;
    0   sin(alpha)  cos(alpha)      0
    0   0           0               1];
end

function TY = sym_troty(beta)
    TY = [
    cos(beta)   0   -sin(beta)      0;
    0           1   0               0;
    sin(beta)   0   cos(beta)       0;
    0           0   0               1];
end

function TZ = sym_trotz(gamma)
    TZ = [
    cos(gamma)  -sin(gamma)     0   0;
    sin(gamma)  cos(gamma)      0   0;
    0           0               1   0;
    0           0               0   1];
end