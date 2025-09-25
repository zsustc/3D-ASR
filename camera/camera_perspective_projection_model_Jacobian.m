function [Jacobian] = camera_perspective_projection_model_Jacobian(dv, K, R, t, v)

Pc = R*v+t;

Puv2Pc = [K(1,1)/Pc(3),0,(-K(1,1)*Pc(1))/(Pc(3)^2);0,K(2,2)/Pc(3),(-K(2,2)*Pc(2))/(Pc(3)^2)];
Pc2Pw = R*dv; %jacobian:Pc/Pw

Jacobian = Puv2Pc*Pc2Pw;
end
