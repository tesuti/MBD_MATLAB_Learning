disp('アーム角度の平衡点を入力してください');
ye = input('ye =');  % y(t)の平衡点 ye を入力
ue = M*l*g*sin(ye)  % u(t)の平衡点 ue

numP = 1;  % 伝達関数 P(s) の分子多項式
denP = [J c M*l*g*cos(ye)];  %　伝達関数　P(s)　の分子多項式
sysP = zpk(tf(numP,denP))  % 伝達関数 P(s) の定義
pole(sysP)  % 伝達関数 P(s) の定義