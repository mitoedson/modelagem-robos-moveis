% Cópia LEGÍVEL do código do bloco MATLAB Function de SIMU_DIF_CBF_ponto_sensor.slx
% Ponto de chegada guiado por "sensor" (sem trajetória definida).
%
% Este arquivo é só para leitura e estudo: o código executado é o que está dentro do .slx.
% Entradas: u = [x; y; theta; t]. Saída: out = [x'; y'; theta'; v; w; h; xd0; yd0; thetad0].
% (comentários de versões antigas foram removidos)

function out = fcn(u, xo, yo, l, rs, psc, c, lambda)
x = u(1);
y = u(2);
theta = u(3);
t = u(4);

f_model = [0;0;0];
g_model = [cos(theta) 0;sin(theta) 0;0 1];

obs  = [xo(:), yo(:)];
nobs = size(obs,1);

% ===== PONTO DE CHEGADA GUIADO POR "SENSOR" (sem trajetoria definida) =====
% A cada instante o robo usa sua propria pose, o ponto de chegada G e os obstaculos ao alcance.
% O alvo fica a Ld a frente, na direcao de G, deslocado lateralmente quando um obstaculo
% bloqueia o caminho direto (lado de MENOR desvio; exatamente sobre a linha: pela direita).
G    = [5; 5];  thG = pi/4;     % ponto e orientacao de chegada (iguais ao INIT do grafico)
m    = 0.05;    % folga alem de rs
pw   = 1.2;     % meia-largura da deformacao [m]
Ld   = 0.6;     % distancia do alvo a frente do robo [m]
Lb   = 1.5;     % distancia de G a partir da qual a orientacao final passa a contar [m]
Rsen = 3.0;     % alcance do sensor de obstaculos [m]
Lrot = 0.005;   % distancia de G a partir da qual o robo gira no lugar ate thG [m] (5 mm)
cm   = rs + m;
gx = G(1) - x;  gy = G(2) - y;
Lg = sqrt(gx^2 + gy^2);
if Lg > 1e-6
    ur = [gx; gy]/Lg;
else
    ur = [cos(theta); sin(theta)];
end
nr = [-ur(2); ur(1)];
angG = atan2(ur(2), ur(1));
st = min(Lg, Ld);
l0 = 0;  l1 = 0;
for i = 1:nobs
    so   = (obs(i,1)-x)*ur(1) + (obs(i,2)-y)*ur(2);
    dlat = (obs(i,1)-x)*nr(1) + (obs(i,2)-y)*nr(2);
    if so > -0.3 && so < Rsen && abs(dlat) < cm
        if abs(dlat) < 1e-3
            ell = -cm;
        else
            ell = dlat - sign(dlat)*cm;
        end
        sg = (st - so)/pw;
        if abs(sg) < 1
            l0 = l0 + ell*0.5*(1 + cos(pi*sg));
            l1 = l1 - ell*0.5*sin(pi*sg)*pi/pw;
        end
    end
end
xd = x + st*ur(1) + l0*nr(1);           % alvo
yd = y + st*ur(2) + l0*nr(2);
tht  = angG + atan(l1);                 % orientacao tangente ao desvio
lamb = min(max(1 - Lg/Lb, 0), 1);       % 0 longe de G, 1 ao chegar
thetad = tht + lamb*atan2(sin(thG - tht), cos(thG - tht));
xd_dot = 0;  yd_dot = 0;  thetad_dot = 0;

ex = cos(theta)*(xd-x) + sin(theta)*(yd-y);
ey = -sin(theta)*(xd-x) + cos(theta)*(yd-y);
etheta = thetad-theta;
ctheta = 1;
V = ((ex)^2 + (ey)^2)/2 + ctheta*(1-cos(etheta));
LfV = 0;
LgV = [x*cos(theta) - xd*cos(theta) + y*sin(theta) - yd*sin(theta), ctheta*sin(theta - thetad)];
Vt = xd*xd_dot - x*xd_dot - y*yd_dot + yd*yd_dot - ctheta*thetad_dot*sin(theta - thetad);

side = 1;      % +1 desvia virando a esquerda, -1 a direita
hact = 1.0;    % h abaixo do qual o vies de lado comeca
wb   = 0.0;    % intensidade maxima do vies em w [rad/s]

px = x + l*cos(theta);
py = y + l*sin(theta);

hv    = zeros(nobs,1);
Lghm  = zeros(nobs,2);
wbias = 0;
for i = 1:nobs
    dx = px - obs(i,1);
    dy = py - obs(i,2);
    hv(i)     = dx^2 + dy^2 - rs^2;
    Lghm(i,:) = [2*dx*cos(theta) + 2*dy*sin(theta), ...
                 2*l*(dy*cos(theta) - dx*sin(theta))];
    dist  = max(sqrt(dx^2 + dy^2), 1e-6);
    a     = max(0, min(1, 1 - hv(i)/hact));
    ahead = max(0, (-dx*cos(theta) - dy*sin(theta))/dist);
    wbias = wbias + side*a*ahead;
end
wbias = wb*max(-1, min(1, wbias));

h   = min(hv);   % apenas para o grafico
Lfh = 0;

% ===== CONTROLE NOMINAL DE POSE (polar) =====   (comentário corrigido nesta cópia)
kp = 0.8;  ka = 2.0;  kb = -0.5;  krot = 2.0;   % controlador polar (kp>0, kb<0, ka>kp)
rho = sqrt((xd-x)^2 + (yd-y)^2);
al  = atan2(yd-y, xd-x) - theta;
al  = atan2(sin(al), cos(al));
be  = thetad - theta - al;
be  = atan2(sin(be), cos(be));
v_nom = kp*rho*max(cos(al), 0);         % nao anda de re: vira antes de avancar
w_nom = ka*al + kb*lamb*be + wbias;
if Lg < Lrot                            % chegou: gira no lugar ate a orientacao final
    v_nom = 0;
    w_nom = krot*atan2(sin(thG - theta), cos(thG - theta));
end
u_nom = [v_nom; w_nom];

% QP: minimiza ||u - u_nom||^2 + psc*delta^2
Hqp = [1 0 0; 0 1 0; 0 0 psc];
Fqp = [-u_nom; 0];

% ===== RESTRICOES: 1 CLF + nobs CBFs + limites de v e w =====
Aqp = zeros(nobs+5,3);
bqp = zeros(nobs+5,1);
Aqp(1,:) = [LgV, -1];
bqp(1)   = -LfV - c*V - Vt;
for i = 1:nobs
    Aqp(1+i,:) = [-Lghm(i,:), 0];
    bqp(1+i)   = Lfh + lambda*hv(i);
end
Aqp(nobs+2:nobs+5,:) = [1 0 0; -1 0 0; 0 1 0; 0 -1 0];
bqp(nobs+2:nobs+5)   = 2;

ucont = [0;0;0];
coder.extrinsic('QPhild');
ucont = QPhild(Hqp,Fqp,Aqp,bqp);
uc = [ucont(1,1);ucont(2,1)];
xmponto = f_model + g_model*uc;

% Saidas de referencia para o grafico: ponto de chegada.
xd0 = G(1);
yd0 = G(2);
thetad0 = thG;
out = [xmponto(1,1);xmponto(2,1);xmponto(3,1);uc(1,1);uc(2,1);h;xd0;yd0;thetad0];
