function [np, npp] = GenerateOrthonormalBasis(n)
    n = n(1:3) / norm(n(1:3));
    [~, i] = min(abs(n));
    n2 = n;
    
    if n(i) >= 0
        sgn = 1;
    else
        sgn = -1;
    end
    
    n2(i) = n2(i) + sgn;
    np = skewsymm(n2) * n;
    np = np / norm(np);
    npp = skewsymm(n) * np;
    npp = npp / norm(npp);
end
