close all
clear all
clc

% ===== Posição e parâmetros iniciais do robô =====
x0 = 1;
y0 = 0;
theta0 = 0;
v0 = 0;
w0 = 0;

% ===== Obstáculos: um elemento por obstáculo em xo, yo (centro) e Ro (raio) =====
% O bloco lê xo e yo do workspace; Ro é usado só no desenho.
% O controle segue internamente uma referência deformada em torno de cada obstáculo,
% mas a referência exibida (verde) é o círculo original (R = 2, centro na origem).
xo = [-1   0    1.75 -2.0];
yo = [-1.5 2.2 -1.0   0.8];
Ro = [0.1  0.1  0.1   0.1];

l  = 0.2;     % distância do ponto de controle à frente do eixo do robô
rs = 0.3;     % raio da região de segurança (igual para todos os obstáculos)
psc = 1000;   % peso da folga da CLF no QP

% Velocidade de retomada da trajetória (CLF)
c = 0.3;

% Aproximação da barreira: maior = chega mais perto antes de a CBF frear
lambda = 10;

% Ganhos do controle nominal (kx, ky, kth) e parâmetros da deformação da
% referência (m, pw, tolo) ficam dentro do bloco MATLAB Function.

% ===== Simulação =====
tspan = [0 60];
sim('SIMU_DIF_CBF_v2_refcirc', tspan);

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
plot(xd,yd,'g--','LineWidth',2)
hold on
plot(x(1),y(1),'yo','MarkerSize',8,'LineWidth',2);
hold on
plot(x(length(x)),y(length(y)),'ys','MarkerSize',8,'LineWidth',2);
xlabel('x(m)')
ylabel('y(m)')
grid
axis equal
legend('Obstáculo','Região de segurança','Robô','Referência (círculo original)','Inicial','Final');