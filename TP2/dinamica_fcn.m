function [ddq1, ddq2] = dinamica_fcn(q1, q2, dq1, dq2, tau1, tau2)
% DINAMICA_FCN  Bloque MATLAB Function para Simulink.
% Calcula las aceleraciones articulares del doble péndulo invertido.
% Modelo: tau = M(q)*ddq + H(q,dq) + G(q)  =>  ddq = M^-1*(tau - H - G)
%
% Notación: c1=cos(q1), s1=sin(q1), c12=cos(q1+q2), s12=sin(q1+q2), c2=cos(q2)

% --- Parámetros (hardcoded para el bloque) ---
a1    = 0.2;
a2    = 0.2;
m1    = 1.5;
m2    = 1.0;
Ig1zz = 1e-3;
Ig2zz = 1e-4;
xg1   = -a1/2;   % = -0.1
yg1   = 0;
xg2   = -a2/2;   % = -0.1
yg2   = 0;
grav  = 9.81;

% --- Trigonometría ---
c1  = cos(q1);
s1  = sin(q1);
c2  = cos(q2);
s2  = sin(q2);
c12 = cos(q1 + q2);
s12 = sin(q1 + q2);

% --- Matriz de inercia M (ec. 27-30 de la memoria) ---
% I0izz = Igizz + (xgi^2 + ygi^2)*mi  [Steiner, para la terna del eslabón]
I01zz = Ig1zz + (xg1^2 + yg1^2)*m1;   % = 0.016 Kg·m²
I02zz = Ig2zz + (xg2^2 + yg2^2)*m2;   % = 0.0101 Kg·m²  (aprox, yg2=0)

m22 = I02zz + 2*a2*xg2*m2 + a2^2*m2;
m12 = m22 + a1*((a2 + xg2)*c2 - yg2*s2)*m2;
m11 = I01zz + 2*a1*xg1*m1 + a1^2*(m1 + m2) + 2*m12 - m22;

M = [m11, m12; m12, m22];

% --- Vector H — Coriolis y centrífugo (ec. 33) ---
dm12_dq2 = -a1*((a2 + xg2)*s2 + yg2*c2)*m2;

H1 = dm12_dq2 * (2*dq1*dq2 + dq2^2);
H2 = -dm12_dq2 * dq1^2;

H = [H1; H2];

% --- Vector G — gravedad (ec. 36, 40) ---
% g^0 = [0, -9.81, 0]^t  (montado en la pared, gravedad en -Y de frame 0)
g2 = m2*grav*((xg2 + a2)*c12 - yg2*s12);
g1 = m1*grav*((xg1 + a1)*c1 - yg1*s1) + m2*grav*a1*c1 + g2;

G = [g1; g2];

% --- Ecuación de movimiento ---
tau = [tau1; tau2];
ddq = M \ (tau - H - G);

ddq1 = ddq(1);
ddq2 = ddq(2);
end
