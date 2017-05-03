% Reproduction of equation 16.9 page 452 of Numerical Optimization by
% Nocedal and Wright, 2nd edition

Q = [   6 2 1;
        2 5 2;
        1 2 4];
c = [ -8; -3; -3];
Aeq = [ 1 0 1;
        0 1 1];
beq = [3; 0];

x = quadprog(Q, c, [], [], Aeq, beq)
% Should give x* = (2, -1, 1)
