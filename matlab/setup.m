% setup.m -- adiciona a pasta common/ (QPhild.m, LIE_DIF_CBF.m) ao caminho do MATLAB.
% Rode UMA vez por sessão, a partir da pasta matlab/:
%     >> setup
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'common'));
fprintf('Caminho configurado: %s\n', fullfile(here, 'common'));
