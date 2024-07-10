% Parámetros del espacio de estado
Ac = [0, -0.00081454, -0.035336, 0;
      0, -0.086973, -1.0254, 0;
      9.1371, 9.1371, -10.278, 0;
      0, 0, 0, -7.3235];
Bc = [0.47619, 0, -4.8609, 0;
      13.819, -0.0016082, -141.06, 0;
      0, 0, -201.21, 0;
      0, 0, 0, 7.3235];
Cc = eye(4);
D = zeros(4);

% Obtener los polos del sistema /
polos = eig(Ac); % Valores propios

% Calcular las constantes de tiempo
constantes_de_tiempo = -1 ./ real(polos);

% Mostrar los resultados
disp('Polos del sistema:');
disp(polos);
disp('Constantes de tiempo:');
disp(constantes_de_tiempo);

% Seleccionar el tiempo de muestreo
Tiempo_de_muestreo = min(constantes_de_tiempo) / 10;

% Mostrar el tiempo de muestreo recomendado
disp('Tiempo de muestreo recomendado (Tiempo_de_muestreo):');
disp(Tiempo_de_muestreo);
disp(Ts);