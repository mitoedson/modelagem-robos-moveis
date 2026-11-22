close all
clear all
clc

% ===== Posição e parâmetros iniciais do robô =====
x0 = 0;
y0 = 0;
theta0 = 0;

% Ponto de chegada (usado só no desenho; o bloco tem G e thG escritos dentro dele)
xg = 5;
yg = 5;
thg = pi/4;
v0 = 0;
w0 = 0;

% ===== Obstáculos: um elemento por obstáculo em xo, yo (centro) e Ro (raio) =====
% O bloco lê xo e yo do workspace; Ro é usado só no desenho.
% Só há o ponto de chegada. O robô parte da pose (x0, y0, theta0) e, a cada instante, usa a
% própria pose, o ponto de chegada e os obstáculos ao alcance do sensor (3 m, definido no bloco).
xo = [1.67 2.3 4.23];
yo = [1.25  2.6 4.25];
Ro = [0.1 0.1 0.1];

l  = 0.2;     % distância do ponto de controle à frente do eixo do robô
rs = 0.3;     % raio da região de segurança (igual para todos os obstáculos)
psc = 100;    % peso da folga da CLF no QP

% Velocidade de retomada da trajetória (CLF)
c = 0.3;

% Aproximação da barreira: maior = chega mais perto antes de a CBF frear
lambda = 10;

% Ganhos (kp, ka, kb), G, thG, alcance do sensor e parâmetros do desvio (m, pw, Ld) ficam dentro do bloco.

% ===== Simulação =====
tspan = [0 60];
sim('SIMU_DIF_CBF_ponto_sensor', tspan);

figure(1)
subplot(321),plot(t,x,'r',t,xd,'b')
xlabel('t(s)')
ylabel('x(m)')
grid
legend('Atual','Referência');
subplot(322),plot(t,y,'r',t,yd,'b')
xlabel('t(s)')
ylabel('y(m)')
grid
legend('Atual','Referência');
subplot(323),plot(t,wrapToPi(theta),'r',t,thetad,'b')
xlabel('t(s)')
ylabel('\theta(rad)')
grid
legend('Atual','Referência');
subplot(324),plot(t,v)
xlabel('t(s)')
ylabel('v (m/s)')
grid
subplot(325),plot(t,w)
xlabel('t(s)')
ylabel('w (rad/s)')
grid
subplot(326),plot(t,h)
hold on
yline(0, '--r')
ylim([-5 20])
xlabel('t(s)')
ylabel('h')
grid

figure(2)
ang = linspace(0,2*pi,200);
for k = 1:length(xo)
    if k == 1, vis = 'on'; else, vis = 'off'; end   % legenda só do primeiro obstáculo
    plot(xo(k) + Ro(k)*cos(ang), yo(k) + Ro(k)*sin(ang),'r','HandleVisibility',vis);
    hold on
    plot(xo(k) + rs*cos(ang), yo(k) + rs*sin(ang),'r:','HandleVisibility',vis);
    hold on
end
plot(x,y,'b','LineWidth',1.5)
hold on
plot(x(1),y(1),'yo','MarkerSize',8,'LineWidth',2);
hold on
quiver(x(1),y(1),0.6*cos(theta0),0.6*sin(theta0),0,'y','LineWidth',2,'MaxHeadSize',2);
hold on
plot(xg,yg,'ys','MarkerSize',10,'LineWidth',2);      % Final = ponto de chegada
xlabel('x(m)')
ylabel('y(m)')
grid
axis equal
legend('Obstáculo','Região de segurança','Robô','Inicial','Sentido inicial','Final');

% Erro final do robô em relação ao ponto de chegada
fprintf('Erro final: posição = %.3f m, orientação = %.3f rad\n', hypot(x(end)-xg, y(end)-yg), abs(wrapToPi(theta(end)-thg)));
