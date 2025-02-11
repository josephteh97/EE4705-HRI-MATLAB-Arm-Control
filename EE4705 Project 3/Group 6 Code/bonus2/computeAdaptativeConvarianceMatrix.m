function [Rinv_, factor_] = computeAdaptativeConvarianceMatrix(Rinv, Qtheta, QthetaOld, factor)

if Qtheta < QthetaOld 
    if factor < 0.1
        Rinv_ = Rinv;
        factor_ = factor;
    else
        Rinv_ = 0.9 * Rinv;
        factor_ = 0.9 * factor;
    end
else
    Rinv_ = Rinv;
    factor_ = factor;
end