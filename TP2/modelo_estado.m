function dx = modelo_estado(x, tau)
% MODELO_ESTADO  Bloque MATLAB Function para Simulink — formulación SS no lineal.
% Devuelve la derivada del vector de estado completo.
%
% Entradas:
%   x   = [q1; q2; dq1; dq2]  (4x1)
%   tau = [tau1; tau2]         (2x1)
%
% Salida:
%   dx  = [dq1; dq2; ddq1; ddq2]  (4x1)
%
% Conectar: un bloque Integrator vectorial (IC = x0, 4 estados)
%           con retroalimentación completa de x.

q1  = x(1);
q2  = x(2);
dq1 = x(3);
dq2 = x(4);

[ddq1, ddq2] = dinamica_fcn(q1, q2, dq1, dq2, tau(1), tau(2));

dx = [dq1; dq2; ddq1; ddq2];
end
