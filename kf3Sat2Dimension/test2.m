A = [1 0 1; 0 2 0; 1 0 3]

R = chol(A, 'lower')
R*R'