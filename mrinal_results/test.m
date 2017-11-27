A = randn(4);
[q, r] = NSI(A'* A, 10000);

estimated_sigs  = diag(r);

actual_sigs = svds(A);



function [Q,R]=NSI(A,numsteps)
    [Q,R] = qr(A);
    for k = 1:numsteps
        [Q, R] = qr(R * Q);
    end
end
