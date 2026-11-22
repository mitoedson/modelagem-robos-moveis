% Cópia LEGÍVEL do código do bloco MATLAB Function de SIMU_DIF_CBF_v2_refcirc.slx
% Trajetória circular (R = 2 m) com referência interna deformada em torno dos obstáculos.
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

% ===== REFERENCIA CIRCULAR DEFORMADA EM TORNO DOS OBSTACULOS =====
% Escolhe o lado de menor desvio: obstaculo com ro <= R+tolo (na linha ou dentro do circulo)
% e contornado por fora; com ro > R+tolo, por dentro. Com tolo = 0, e sempre o lado mais curto.
R = 2;
w_ref_const = 0.25;
phi = w_ref_const*t;
Rr = R;  d1 = 0;  d2 = 0;          % raio efetivo e suas derivadas em relacao a phi
m    = 0.05;    % folga alem de rs
pw   = 0.7;     % meia-largura angular da deformacao [rad]
tolo = 0.0;     % 0: escolhe o lado de menor desvio (ro<=R: por fora; ro>R: por dentro)
for i = 1:nobs
    ro = sqrt(obs(i,1)^2 + obs(i,2)^2);
    ph = atan2(obs(i,2), obs(i,1));
    if ro <= R + tolo
        delta = ro + rs + m - R;
    else
        delta = ro - rs - m - R;
    end
    dphi = atan2(sin(phi - ph), cos(phi - ph));
    if abs(dphi) < pw
        Rr = Rr + delta*0.5*(1 + cos(pi*dphi/pw));
        d1 = d1 - delta*0.5*sin(pi*dphi/pw)*pi/pw;
        d2 = d2 - delta*0.5*cos(pi*dphi/pw)*(pi/pw)^2;
    end
end
xd = Rr*cos(phi);
yd = Rr*sin(phi);
xd_dot = w_ref_const*(d1*cos(phi) - Rr*sin(phi));
yd_dot = w_ref_const*(d1*sin(phi) + Rr*cos(phi));
thetad = atan2(yd_dot, xd_dot);
thetad_dot = w_ref_const*(1 + (d1^2 - Rr*d2)/(Rr^2 + d1^2));

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
kx = 4.0;  ky = 4.0;  kth = 6.0;
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
xd0 = R*cos(phi);
yd0 = R*sin(phi);
thetad0 = atan2(R*w_ref_const*cos(phi), -R*w_ref_const*sin(phi));
out = [xmponto(1,1);xmponto(2,1);xmponto(3,1);uc(1,1);uc(2,1);h;xd0;yd0;thetad0];
