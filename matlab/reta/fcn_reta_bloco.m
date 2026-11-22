% Cópia LEGÍVEL do código do bloco MATLAB Function de SIMU_DIF_CBF_reta_refdef.slx
% Reta de S a G com referência interna deformada em torno dos obstáculos.
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

% ===== REFERENCIA: RETA DE S ATE G, DEFORMADA EM TORNO DOS OBSTACULOS =====
% Reta percorrida a velocidade constante em Tf segundos. Cada obstaculo a menos de
% (rs+m) da reta desloca a referencia lateralmente para o lado de MENOR desvio
% (obstaculo exatamente sobre a reta: passa pela direita).
S  = [0; 0];  G = [6; 6];  Tf = 60;      % inicio, fim e duracao (iguais ao INIT do grafico)
Lseg = sqrt((G(1)-S(1))^2 + (G(2)-S(2))^2);
ur = [(G(1)-S(1)); (G(2)-S(2))]/Lseg;    % direcao da reta
nr = [-ur(2); ur(1)];                    % normal (esquerda)
vd0 = Lseg/Tf;
sc = min(vd0*t, Lseg);                   % abscissa ao longo da reta
m    = 0.05;    % folga alem de rs
pw   = 0.8;     % meia-largura da deformacao ao longo da reta [m]
cm   = rs + m;
l0 = 0;  l1 = 0;  l2 = 0;                % deslocamento lateral e derivadas em relacao a sc
for i = 1:nobs
    so   = (obs(i,1)-S(1))*ur(1) + (obs(i,2)-S(2))*ur(2);
    dlat = (obs(i,1)-S(1))*nr(1) + (obs(i,2)-S(2))*nr(2);
    if abs(dlat) < cm
        if abs(dlat) < 1e-3
            ell = -cm;                   % sobre a reta: passa pela direita
        else
            ell = dlat - sign(dlat)*cm;  % lado de menor desvio
        end
        sg = (sc - so)/pw;
        if abs(sg) < 1
            l0 = l0 + ell*0.5*(1 + cos(pi*sg));
            l1 = l1 - ell*0.5*sin(pi*sg)*pi/pw;
            l2 = l2 - ell*0.5*cos(pi*sg)*(pi/pw)^2;
        end
    end
end
xd = S(1) + sc*ur(1) + l0*nr(1);
yd = S(2) + sc*ur(2) + l0*nr(2);
xd_dot = vd0*(ur(1) + l1*nr(1));
yd_dot = vd0*(ur(2) + l1*nr(2));
thetad = atan2(yd_dot, xd_dot);
thetad_dot = vd0*l2/(1 + l1^2);

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

% ===== CONTROLE NOMINAL DE RASTREAMENTO (Kanayama) =====
kx = 2.0;  ky = 10.0;  kth = 3.0;
vd    = sqrt(xd_dot^2 + yd_dot^2);
v_nom = vd*cos(etheta) + kx*ex;
w_nom = thetad_dot + vd*(ky*ey + kth*sin(etheta)) + wbias;
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

% Saidas de referencia para o grafico: CIRCULO ORIGINAL (sem deformacao).
% O controle usa internamente a referencia deformada (xd, yd, thetad).
% Saidas de referencia para o grafico: RETA ORIGINAL (sem deformacao).
xd0 = S(1) + sc*ur(1);
yd0 = S(2) + sc*ur(2);
thetad0 = atan2(ur(2), ur(1));
out = [xmponto(1,1);xmponto(2,1);xmponto(3,1);uc(1,1);uc(2,1);h;xd0;yd0;thetad0];
