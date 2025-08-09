%% ==============================================================
%  アームモデル PI-D 制御器設計スクリプト（エラー対策済み）
%  対象: arm_linear_sim_pi_d_cont モデル
%  内容: パラメータ読込 → ゲイン算出 → Simulink実行 → 応答解析
% ==============================================================

%% --- アーム物理パラメータ読込
arm_para    % l, M, J, c, g 定義

%% --- プラントパラメータ算出
d0 = M*g*l;   % 重力による復元トルク係数
d1 = c;       % 粘性摩擦係数
d2 = J;       % 慣性モーメント

%% --- 目標モデル（2次系）パラメータ設定
wm = 10;      % 固有角周波数 [rad/s]
a1 = 1.4;     % 減衰係数

gamma_m1 = a1/wm;
gamma_m2 = 1/wm^2;
gamma_m3 = 0; % γm3=0（標準2次系）

%% --- PI-D制御器ゲイン計算（目標モデル追従用）
kI = d0/gamma_m1;
kP = (d2*gamma_m1 - d0*gamma_m3)/(gamma_m1*gamma_m2);
kD = (d2*gamma_m1^2 - d1*gamma_m1*gamma_m2 ...
    + d0*(gamma_m2^2 - gamma_m1*gamma_m3))/(gamma_m1*gamma_m2);

%% --- ステップ応答シミュレーション条件
rc_deg = 30;                  % 指令値 [deg]
rc = rc_deg*(pi/180);         % [rad]へ変換
dc = 0;                     % 外乱トルク設定（一定値）

% Simulinkモデル実行
sim('arm_linear_sim_pi_d_cont');

%% --- Simulink 出力整形（t を step() 用に）
t_sim = double(t(:));          % 列ベクトル化 + double化
y_sim = double(y(:));          % 出力も列ベクトル化

% 等間隔ベクトル生成（目標値）
Ts = mean(diff(t_sim));                     
t_uniform = (0:Ts:t_sim(end))';             

%% --- 目標モデル応答計算
sysGm2 = tf([wm^2],[1 a1*wm wm^2]); 
ym = step(sysGm2, t_uniform) * rc;  

%% --- 実応答 vs 目標応答 プロット
figure(1)
plot(t,y*(180/pi),t_uniform,ym*(180/pi),'--')
%plot(t_uniform, y_sim*(180/pi), ...
%t_uniform, ym*(180/pi), '--')

xlabel('t [s]')
ylabel('y(t), y_m(t) [deg]')
legend({'y(t)','y_m(t) (d(t)=0)'}, 'Location', 'SouthEast')
ylim([0 (4/3)*rc_deg])
grid on

%% --- 閉ループ伝達関数計算
s = tf('s');
sysP  = 1/(J*s^2 + c*s + M*g*l); % プラントモデル
sysC1 = kP + kI/s + kD*s;        % 外乱応答用 PI-D制御器
sysC2 = kP + kI/s;               % 目標値応答用 PI制御器

sysGyr = zpk(minreal(sysP*sysC2/(1 + sysP*sysC1))); % 目標値→出力
sysGyd = zpk(minreal(         1/(1 + sysP*sysC1))); % 外乱→出力
