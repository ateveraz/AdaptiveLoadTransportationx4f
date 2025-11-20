clearvars; close all; clc;

%% Datos Dinámica

mL = 0.225;
e3 = [0;0;1];
mi = 0.4;
J = diag([2.1,1.87,3.97])*0.01;

%% Calcular las qis
L1=1;
L2=1;
L3=1;
L4=1;

m = 0.225;
g = 9.81*e3;

%% Initial conditions
xL0=[0;0;0];
q1_rot=rotation_matrix(-25, 'z')*rotation_matrix(-35,'y');
q2_rot=rotation_matrix(65, 'z')*rotation_matrix(-35,'y');
q3_rot=rotation_matrix(155, 'z')*rotation_matrix(-35,'y');
q4_rot=rotation_matrix(245, 'z')*rotation_matrix(-35,'y');

x1_0=xL0+q1_rot*[0;0;L1];
x2_0=xL0+q2_rot*[0;0;L2];
x3_0=xL0+q3_rot*[0;0;L3];
x4_0=xL0+q4_rot*[0;0;L4];

d1 = (xL0-x1_0)/L1;
d2 = (xL0-x2_0)/L2; 
d3 = (xL0-x3_0)/L3;
d4 = (xL0-x4_0)/L4;

n1 = norm(d1); 
n2 = norm(d2); 
n3 = norm(d3);
n4 = norm(d4);

q1_0 = d1/n1;
q2_0 = d2/n2;
q3_0 = d3/n3;
q4_0 = d4/n4;


Kd1 = 5; Alpha1 = 1; Gamma1 = 0;
Kd2 = 5; Alpha2 = 1; Gamma2 = 0;
Kd3 = 5; Alpha3 = 1; Gamma3 = 0;
Kd4 = 5; Alpha4 = 1; Gamma4 = 0;

%sim('simulator.slx');