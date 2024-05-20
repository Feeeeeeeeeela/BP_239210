%Zadávané úhly ve stupních
theta1=deg2rad(0);
theta2=deg2rad(0);
theta3=deg2rad(0);
theta4=deg2rad(0);
theta5=deg2rad(0);
theta6=deg2rad(0);

% J1
a1 = 0;         %délka spoje
alpha1=0.5*pi;  %naklonění spoje
d1 = 131.22;    %délka vybočení do strany
delta1 = 0;     %úhel natočení (offset)
% J2
a2 = -110.4;
alpha2 = 0;
d2 = 0;
delta2 = -0.5*pi;
% J3
a3 = -96;
alpha3 = 0;
d3 = 0;
delta3 = 0;
% J4
a4 = 0;
alpha4 = 0.5*pi;
d4 = 63.4;
delta4 = -0.5*pi;
% J5
a5 = 0;
alpha5 = -0.5*pi;
d5 = 75.05;
delta5 = 0.5*pi;
% J6
a6 = 0;
alpha6 = 0;
d6 = 45.6;
delta6 = 0;

% Transformační matice pro J1
A1=[cos(theta1 + delta1), -sin(theta1 + delta1)*cos(alpha1), sin(theta1 + delta1)*sin(alpha1), a1*cos(theta1+delta1);
    sin(theta1 + delta1), cos(theta1 + delta1)*cos(alpha1), -cos(theta1 + delta1)*sin(alpha1), a1*sin(theta1+delta1);
    0, sin(alpha1), cos(alpha1), d1;
    0, 0, 0, 1];

% Transformační matice pro J2
A2=[cos(theta2 + delta2), -sin(theta2 + delta2)*cos(alpha2), sin(theta2 + delta2)*sin(alpha2), a2*cos(theta2+delta2);
    sin(theta2 + delta2), cos(theta2 + delta2)*cos(alpha2), -cos(theta2 + delta2)*sin(alpha2), a2*sin(theta2+delta2);
      0, sin(alpha2), cos(alpha2), d2;
      0, 0, 0, 1];


% Transformační matice pro J3
A3=[cos(theta3 + delta3), -sin(theta3 + delta3)*cos(alpha3), sin(theta3 + delta3)*sin(alpha3), a3*cos(theta3+delta3);
    sin(theta3 + delta3), cos(theta3 + delta3)*cos(alpha3), -cos(theta3 + delta3)*sin(alpha3), a3*sin(theta3+delta3);
      0, sin(alpha3), cos(alpha3), d3;
      0, 0, 0, 1];

% Transformační matice pro J4
A4=[cos(theta4 + delta4), -sin(theta4 + delta4)*cos(alpha4), sin(theta4 + delta4)*sin(alpha4), a4*cos(theta4+delta4);
    sin(theta4 + delta4), cos(theta4 + delta4)*cos(alpha4), -cos(theta4 + delta4)*sin(alpha4), a4*sin(theta4+delta4);
    0, sin(alpha4), cos(alpha4), d4;
    0, 0, 0, 1];


% Transformační matice pro J5
A5=[cos(theta5 + delta5), -sin(theta5 + delta5)*cos(alpha5), sin(theta5 + delta5)*sin(alpha5), a5*cos(theta5+delta5);
    sin(theta5 + delta5), cos(theta5 + delta5)*cos(alpha5), -cos(theta5 + delta5)*sin(alpha5), a5*sin(theta5+delta5);
    0, sin(alpha5), cos(alpha5), d5;
    0, 0, 0, 1];


% Transformační matice pro J6
A6=[cos(theta6 + delta6), -sin(theta6 + delta6)*cos(alpha6), sin(theta6 + delta6)*sin(alpha6), a6*cos(theta6+delta6);
    sin(theta6 + delta6), cos(theta6 + delta6)*cos(alpha6), -cos(theta6 + delta6)*sin(alpha6), a6*sin(theta6+delta6);
    0, sin(alpha6), cos(alpha6), d6;
    0, 0, 0, 1];

T6 = A1 * A2 * A3 * A4 * A5 * A6;

% Získání x,y,z poslední pozice
position_end_effector = T6(1:3, 4);

% Získání a převedení rotačních hodnot
R6 = T6(1:3, 1:3);
rx6 = atan2(R6(3, 2), R6(3, 3));
ry6 = atan2(-R6(3, 1), sqrt(R6(3, 2)^2 + R6(3, 3)^2));
rz6 = atan2(R6(2, 1), R6(1, 1));
% Převod na deg
rx6 = rad2deg(rx6);
ry6 = rad2deg(ry6);
rz6 = rad2deg(rz6);

fprintf('Pozice konce (x, y, z): %.2f, %.2f, %.2f mm\n', position_end_effector);
fprintf('Rotace (roll, pitch, yaw): %.2f, %.2f, %.2f degrees\n', rx6, ry6, rz6);



