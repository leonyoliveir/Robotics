T1 = se2(1, 2, 30*pi/180)
figure()
trplot2(T1, 'frame', '1', 'color', 'b', 'axis', [0 5 0 5])

T2 = se2(2, 1, 0)
hold on
trplot2(T2, 'frame', '2', 'color', 'r');

T3 = T1*T2
trplot2(T3, 'frame', '3', 'color', 'g');

T4 = T2*T1;
trplot2(T4, 'frame', '4', 'color', 'c');

P = [3 ; 2 ];
plot_point(P, '*');
P1 = inv(T1) * [P; 1]
h2e( inv(T1) * e2h(P) )
homtrans( inv(T1), P)
P2 = homtrans( inv(T2), P)