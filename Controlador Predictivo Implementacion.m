close all

% Definir las matrices del sistema en estado estacionario
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
r = 4; % Número de salidas
m = 4; % Número de entradas
n = 4; % Dimensión del estado


v = 1;  % Horizonte de Control
f = 15;  % Horizonte de Prediccion (Horizonte Extendido)


% Discretizar y simular la respuesta al escalón del sistema
Ts = 0.1;  % Constante de discretización

% Discretización del modelo
I = eye(size(Ac));  % Matriz identidad
A = inv(I - Ts * Ac);
B = A * Ts * Bc;
C = Cc;

% Comprobar los valores propios
eigen_A = eig(Ac);
eigen_Aid = eig(A);

timeSampleTest = 1500; % Teimpo de Simulacion

%% Ingresar valores por Consola



% disp('Ingrese valor de Altura:');
% u1 = input('h: ');
% disp('Ingrese valor de Presion:');
% u2 = input('P: ');
% disp('Ingrese valor de Cambio Fibra hj:');
% u3 = input('hj: ');
% disp('Ingrese valor de Cs:');
% u4 = input('Cs: ');
% 




%% Entradas del Sistema
% inputTest = ones(m, timeSampleTest);
% inputTest(1,:) = 3.5* ones(1, timeSampleTest);   %q1: Flujo de sotck dentro de la headbox m3/s
% inputTest(2,:) = 483.5783 * ones(1, timeSampleTest);   %Cg: Coeficiente de dimensionamiento
% inputTest(3,:) = 0.0804* ones(1, timeSampleTest); %Sj: Area del Labio cortador m2
% inputTest(4,:) = 0.6 * ones(1, timeSampleTest);%Ci: Consistencia de fibra delgada (dentro del labio) %


%% Simulacion del Sitema en tiempo Discreto


x0test = zeros(n, 1);
%x0test = [1;2;0.5;0.1]; % Condiciones Iniciales

%%  Simular el sistema en tiempo discreto
% [Ytest, Xtest] = Smiulacion_espacio_estados(A, B, C, D, inputTest, x0test);
% 
% figure(1);
% for i = 1:4
%     subplot(2, 2, i); % Crear una cuadrícula de 2x2 para los gráficos
%     plot(Ytest(i, :), 'LineWidth', 3);
%     if i == 1        
%         xlabel('time steps');
%         ylabel('metros');
%         legend('Altura (h)');
%         title('Altura (h)');
%     end
%     if i == 2        
%         xlabel('time steps');
%         ylabel('mh20');
%         legend('Presion ');
%         title('Presion');
%     end
%     if i == 3       
%         xlabel('time steps');
%         ylabel('metros');
%         legend('Cambio de Fibra en SL');
%         title('Cambio de Fibra en SL');
%     end
%     if i == 4        
%         xlabel('time steps');
%         ylabel('%');
%         legend('Consistencia de SL'); 
%         title('Consistencia de SL'); 
%     end
% end
% 



%% Parametros del Controlador



% Matriz W1
W1 = zeros(v*m, v*m);
for i = 1:v
    if i == 1
        W1((i-1)*m+1:i*m, (i-1)*m+1:i*m) = eye(m);
    else
        W1((i-1)*m+1:i*m, (i-1)*m+1:i*m) = eye(m);
        W1((i-1)*m+1:i*m, (i-2)*m+1:(i-1)*m) = -eye(m);
    end
end

% Matriz W2
Q0 = 500;
Q_1 = 10;
W2 = zeros(v*m, v*m);
for i = 1:v
    if i == 1
        W2((i-1)*m+1:i*m, (i-1)*m+1:i*m) = Q0;
    else
        W2((i-1)*m+1:i*m, (i-1)*m+1:i*m) = Q_1;
    end
end

% Matriz W3
W3 = W1' * W2 * W1;

% Matriz W4
W4 = zeros(f*r, f*r);
peso_w4 = 100 ;
for i = 1:f
    W4((i-1)*r+1:i*r, (i-1)*r+1:i*r) = peso_w4;
end

%% Definicion de setpoint y Modelo MPC
%  timeSteps = 300;
% setpoint = [3, 12, 3.5, 0.6]; % Altura (h)- Presion - Cambio de Fibra en SL




%Trayectoria de referencia
N_sim = 30000; % Número de pasos de simulación

Setpoint_1 = repmat([1.8; 10; 15; 0.6]', N_sim, 1);

%Setpoint_1 = repmat([1.6-u1; u2-1; u3-13; u4]', N_sim, 1);

% Simular el algoritmo MPC y plotear los resultados
CI = x0test; % Condiciones Iniciales

% Crear el objeto MPC
mpc1 = ControlPredictivoModel(A, B, C, f, v, W3, W4, CI, Setpoint_1);



% Simulación del control MPC
for k = 1:N_sim-f
    mpc1 = mpc1.ControlInputs();
end


%% Visualización de resultados
t = ((0:N_sim-1-f) * Ts);
X = cell2mat(mpc1.estados(1:end-1));
U = cell2mat(mpc1.inputs);


figure(2);
for i = 1:4
    subplot(2, 2, i); % Crear una cuadrícula de 2x2 para los gráficos
    plot(t, X(i, :),'LineWidth', 3);
    if i == 1        
        xlabel('time steps');
        ylabel('metros');
        title('Altura (h)');
    end
    if i == 2        
        xlabel('time steps');
        ylabel('mh20');
        title('Presion (P)');
    end
    if i == 3
        plot(t, X(i, :),'LineWidth', 3);
        xlabel('time steps');
        ylabel('metros');
        title('Cambio de Fibra en SL (hj)');
    end
    if i == 4
        plot(t, X(i, :),'LineWidth', 3);
        xlabel('time steps');
        ylabel('%');
        title('Consistencia de SL (Cs)');
        %plot(Ytest(i, :), 'LineWidth', 3)
    end
end

figure(3);
for i = 1:4
    subplot(2, 2, i); % Crear una cuadrícula de 2x2 para los gráficos
    plot(t, U(i, :),'LineWidth', 3);
    % if U(i, :)<0
    %     U(i, :) = 0;
    % end 
    if i == 1        
        xlabel('time steps');
        ylabel('m^3/s');
        title('Flujo de Stock dentro de la Headbox (q1)');
    end
    if i == 2        
        xlabel('time steps');
        ylabel('Coeficiente');
        title('Coeficiente de Dimensionamiento (Cg)');
    end
    if i == 3    
        plot(t, -1*U(i, :),'LineWidth', 3);
        xlabel('time steps');
        ylabel('m^2');
        title('Area del Labio Cortador (Sj)');
    end
    if i == 4        
        xlabel('time steps');
        ylabel('%');
        title('Consistencia de fibra LC (Ci)'); % Dentro del labio cortador

    end
end






