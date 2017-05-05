load('qp_system.mat');
x = 1;
s = size(Aeq{x}, 1);
A = [H(:,:,x) Aeq{x}';
     Aeq{x} zeros(s,s)];
s = size(H(:,:,x), 2);
B = [zeros(s, 1); beq{x}];
sol = linsolve(A, B);
reshape(sol(1:s), 7, s/7)