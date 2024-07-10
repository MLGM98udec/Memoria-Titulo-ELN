syms h(t) P(t) hj(t) Cs(t) t x1 x2 x3 x4 u1 u2 u3 u4  
% Sistema no lineal puro

%syms  h(t) P(t) hj(t) Cs(t) t u1 u2 u3 u4  % Obtener primero los valores del sistema en estado estacionario del otro .m

% x1 =  0.365;
% x2 = 6.1588;
% x3 = 5.53;
% x4 = 0.6;


sys = zeros(1,4);

flag =1;


% if flag ==1 % x(1) nivel
% 
%     A=4.35*(0.610-(2*(0.475-x1)/sqrt(3)));
%     V=(4.35*pi*0.355*0.355*0.5) + (4.35*asin((0.655-0.475)/0.355)*0.355*0.355)+ 4.35*((0.655-0.475)*0.610/2) + (0.475-x1)*((0.610 + (A/4.35))/2)*4.35;
% else
%     A=4.35*pi*0.71*cos(asin((0.655-x1)/0.355));
%     V=4.35*pi*0.355*0.355*0.2 + 4.35*asin((0.655-x1)/0.355)*0.355*0.355 + 4.35*(A*(0.655-x1)/(2*4.35));
% end
A = 2.1;
V= 1.7;

Vt = 1.8705;
Vs = 0.115;
T = 27;
Aa = 0.2670;
At = 0.3155;
c1= 30;
ae = (0.7854/39.375)/(39.375/2.9);
a1 = 1.31e-4;
k = 1.4;
g = 9.81;
rhow = 999.1; % Densidad del agua
rhoa = 1.293; % Densidad del aire
qi = 0.1;
atm = 10.34;
P0=atm;
friction = 0.13;
Nr = 4;
R = 287; % constante universal de los gases
Cds = 0.98;
L = 1.14;



angle = (3417/c1)*sqrt(x2/(x2+atm));
% if angle >= 90  % Comentar si los x son simbolicos
%     angle = 90;
% end 

GT = 32 + T*(9/5)+460;
qe = (((x2+atm)/atm)*(14.7)*sqrt(520/GT)*u2*sin(pi*angle/180))/(13.1*2.2*3600);
q1 = u1;
qs = 0.98*u3*sqrt(2*9.81*x3); % hj = x(3)
qo = 0.98*0.002*sqrt(2*9.81*x2); % P = x(2)

% if qo > 0.019
%     qo = 0.019;
% end

vb = qs/Aa;
vt = qs/At;
vj = qs/u3;

%%
Dh = diff(h(t),t);
sys11 = (1/A)*(q1-qs-qo);
sys1= Dh == (1/A)*(q1-qs-qo);
sys1 = vpa(sys1,3)
DP = diff(P(t),t);
sys22= (k*P0/rhoa)*(((x2+atm)/P0)^((k-1)/k))*(1/V)*((qi-qe) +rhoa*A*(((x2+atm)/P0)^(1/k))*sys11);
sys2= DP == (k*P0/rhoa)*(((x2+atm)/P0)^((k-1)/k))*(1/V)*((qi-qe) +rhoa*A*(((x2+atm)/P0)^(1/k))*((1/A)*(q1-qs-qo))); % dP/dt
sys2 = vpa(sys2,3)

h0 = ((vb^2)/(2*g))*((1-(16^2/25^2))^2);

hi = x1+x2-h0;

ht = x3 - ((vt^2)/(2*g));

hf = (4*friction*Nr*(vt^2))/(2*g);

%% 
Dhj = diff(hj(t),t);
sys33 = (1/1.14)*sqrt(2*9.81*x3)*(hi-x3-hf);
sys3= Dhj == (1/1.14)*sqrt(2*9.81*x3)*(hi-x3-hf);
sys3 = vpa(sys3,3)
DCs = diff(Cs(t),t);
sys4 = DCs == q1*(u4-x4)/(Vs);
sys44 =  q1*(u4-x4)/(Vs);
sys4 = vpa(sys4,3)
Ecuaciones = [sys1 sys2 sys3 sys4]';
Ecuaciones = vpa(Ecuaciones,3)

Obtencion de los Jacobianos del Sistema

% Puntos del sistema en estado estacionario


% Linealizacion del sistema
A = jacobian([sys11;sys22;sys33;sys44],[x1;x2;x3;x4]);
B = jacobian([sys11;sys22;sys33;sys44],[u1,u2,u3,u4]);
C = eye(4);
D = zeros(4);

x1 = 0.365;
x2 = 6.44;
x3 = 5.53;
x4 = 0.6;
u1 = 0.8422;
u2 = 483.5783;
u3 = 0.0804;
u4 = 0.6;

A = eval(A)
B =eval(B)
C
D


Funciones de Transferencia - Variables de estado

syms s U X Y x_s u1_s u1(s) u2(s) u3(s) u4(s)


Fun_trans = C*inv(s*eye(4)-A)*B+D;
%Fun_trans = C*(s*eye(4)-A)\B+D; % Analizar la sugerencia de matlab
Fun_trans = simplify(Fun_trans);
Fun_trans = vpa(Fun_trans,3)
h_s = Fun_trans(1,1)*u1(s)+Fun_trans(1,2)*u2(s)+Fun_trans(1,3)*u3(s)+Fun_trans(1,4)*u4(s);
h_s = simplify(h_s);
h_s = vpa(h_s,3)

P_s = Fun_trans(2,1)*u1(s)+Fun_trans(2,2)*u2(s)+Fun_trans(2,3)*u3(s)+Fun_trans(2,4)*u4(s);
P_s = vpa(P_s,3)
hj_s = Fun_trans(3,1)*u1(s)+Fun_trans(3,2)*u2(s)+Fun_trans(3,3)*u3(s)+Fun_trans(3,4)*u4(s);
hj_s = simplify(hj_s);
disp(vpa(hj_s, 3));


Cs_s = Fun_trans(4,1)*u1(s)+Fun_trans(4,2)*u2(s)+Fun_trans(4,3)*u3(s)+Fun_trans(4,4)*u4(s);
Cs_s = vpa(Cs_s,3)

Funcion de Transferencia

H_s = [h_s P_s hj_s Cs_s]';
%H_s = vpa(H_s,3)
H_s = simplify(H_s);
disp(vpa(H_s, 3));


Constante de tiempo del sistema
%% Crear el sistema de espacio de estados
Espacio_Estados = ss(A, B, C, D);


%% Simular la respuesta al escalón
t = 0:0.01:10; % Define el tiempo de simulación
[Y, T] = step(Espacio_Estados, t);


%% Obtener la información de la respuesta al escalón
Resp_info = arrayfun(@(i) stepinfo(Y(:,i,:), T), 1:4, 'UniformOutput', false);

%% Mostrar la información de la respuesta al escalón
for i = 1:4
    fprintf('Salida %d:\n', i);
    fprintf('  Tiempo de subida: %f segundos\n', Resp_info{i}.RiseTime);
    fprintf('  Tiempo de asentamiento: %f segundos\n', Resp_info{i}.SettlingTime);
    fprintf('  Tiempo pico: %f segundos\n', Resp_info{i}.PeakTime);
    fprintf('  Máximo sobrepaso: %f %%\n\n', Resp_info{i}.Overshoot);
end

%% Graficar la respuesta al escalón
figure;
for i = 1:4
    subplot(2, 2, i);
    plot(T, Y(:, i));
    title(['Respuesta al escalón de la salida ' num2str(i)]);
    xlabel('Tiempo (s)');
    ylabel(['Salida ' num2str(i)]);
    grid on;
end