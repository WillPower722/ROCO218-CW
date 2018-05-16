function xDot = SSSimulateV1(X, A, B, k)

U = -k*X;

xDot = (A*X) + (B*U);