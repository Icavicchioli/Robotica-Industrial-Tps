% Parámetros dinámicos del doble péndulo invertido
% Enunciado TP5 Dinámica 2024

a1  = 0.2;      % [m] longitud eslabón 1
a2  = 0.2;      % [m] longitud eslabón 2
m1  = 1.5;      % [Kg]
m2  = 1.0;      % [Kg]
Ig1zz = 1e-3;   % [Kg·m²] inercia de eslabón 1 respecto a su CM
Ig2zz = 1e-4;   % [Kg·m²] inercia de eslabón 2 respecto a su CM
xg1 = -a1/2;    % [m] posición del CM en frame 1 (eje x)
yg1 = 0;
xg2 = -a2/2;    % [m] posición del CM en frame 2 (eje x)
yg2 = 0;
g   = 9.81;     % [m/s²]
