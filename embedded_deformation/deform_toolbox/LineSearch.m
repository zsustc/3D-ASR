function [ alpha ] = LineSearch( x,d,F,P,ED_Parameter )
%LINESEARCH Summary of this function goes here
%   Using line search to determine the exact step for 
%   descending. alpha * d is the actual descending step
%   x:current point
%   d:descending direction
%   F:f(x+0*h)
%   P:covariance matrix

alpha = 0.5;   %initial alpha
step = 0.25;   %increase step
%0 < gamma_1 < gamma_2 < 1
gamma_1 = 0.1;  
gamma_2 = 0.9;

F = CalculateF(x,ED_Parameter);
J = JacobianF(x,ED_Parameter);
phi_0 = F' * P * F;
phi_0_derivative = d' * J' * P * F;

F_alpha = CalculateF(x + alpha * d,ED_Parameter);
J_alpha = JacobianF(x + alpha * d,ED_Parameter);
phi_alpha = F_alpha' * P * F_alpha;
phi_alpha_derivative = d' * J_alpha' * P * F;

k = 0;
if((phi_alpha>phi_0+gamma_1*phi_0_derivative*alpha)&&(phi_alpha_derivative<gamma_2*phi_0_derivative))
    alpha = 0;
    return;
end
while(~((phi_alpha<=phi_0+gamma_1*phi_0_derivative*alpha)&&(phi_alpha_derivative>=gamma_2*phi_0_derivative))&&k<10)
    if(phi_alpha>phi_0+gamma_1*phi_0_derivative*alpha)
        alpha = alpha - step;
        step = step / 2;
        F_alpha = CalculateF(x + alpha * d,ED_Parameter);
        phi_alpha = F_alpha' * P * F_alpha;
        phi_alpha_derivative = d' * J_alpha' * P * F;
    elseif(phi_alpha_derivative<gamma_2*phi_0_derivative)
        alpha = alpha + step;
        step = step / 2;
        F_alpha = CalculateF(x + alpha * d,ED_Parameter);
        phi_alpha = F_alpha' * P * F_alpha;
        phi_alpha_derivative = d' * J_alpha' * P * F;
    end
    k = k + 1;
end

end

