% 馬達參數
R = 10;         % 歐姆
L = 0.0005;     % 亨利
Kt = 0.011;     % 轉矩常數 (Nm/A)
Ke = 0.011;     % 背電動勢常數 (V/rad/s)
J = 1e-5;       % 慣量
B = 1e-4;       % 阻尼
N = 30;
% 建立傳遞函數 G(s) = ω(s)/V(s)
num = [Kt];
den = [L*J, (L*B + R*J), (R*B + Kt*Ke)];
G_motor = tf(num, den)/30;

% 顯示傳遞函數
disp('DC Motor Transfer Function (ω(s)/V(s))');
G_motor

% 畫根軌跡
figure;
rlocus(G_motor);
title('Root Locus of Motor');

% 畫 Bode 圖
figure;
margin(G_motor);
title('Bode Plot of Motor');

% 畫步階響應（速度響應）
figure;
step(G_motor);
title('Speed Step Response');
ylabel('Speed (rad/s)');
xlabel('Time (s)');
