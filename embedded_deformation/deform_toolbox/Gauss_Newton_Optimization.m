function [ x_optimum ] = Gauss_Newton_Optimization(input_variable,ED_Parameter)
%GAUSS_NEWTON_OPTIMIZATION Summary of this function goes here
%   Optimize the object function using Gauss-newton algorithm. Jingwei Song
control_vertices = ED_Parameter.control_vertices;
num_nearestpts   = ED_Parameter.num_nearestpts;
num_nodes        = ED_Parameter.num_nodes;

[row,col] = size(input_variable);
x0 = zeros(row*col,1);
i = 1;
for i = 1 : num_nodes
    x0((i-1)*12+1) = input_variable(3*(i-1)+1,1);
    x0((i-1)*12+2) = input_variable(3*(i-1)+2,1);
    x0((i-1)*12+3) = input_variable(3*(i-1)+3,1);
    x0((i-1)*12+4) = input_variable(3*(i-1)+1,2);
    x0((i-1)*12+5) = input_variable(3*(i-1)+2,2);
    x0((i-1)*12+6) = input_variable(3*(i-1)+3,2);
    x0((i-1)*12+7) = input_variable(3*(i-1)+1,3);
    x0((i-1)*12+8) = input_variable(3*(i-1)+2,3);
    x0((i-1)*12+9) = input_variable(3*(i-1)+3,3);
    x0((i-1)*12+10) = input_variable(end - num_nodes + i,1);
    x0((i-1)*12+11) = input_variable(end - num_nodes + i,2);
    x0((i-1)*12+12) = input_variable(end - num_nodes + i,3);
end

F = CalculateF(x0,ED_Parameter);
J = JacobianF(x0,ED_Parameter);
m = size(F,1);
v_diag = ones(m,1);

num_transformed_points = size(control_vertices.after,1);
num_rownode = 6 + 3 * (num_nearestpts - 1);
for i = 1 : num_nodes
    v_diag((i-1)*num_rownode+1:(i-1)*num_rownode+6,1) = 1;
    v_diag((i-1)*num_rownode+6+1:(i-1)*num_rownode+6+3*num_nearestpts,1) = 0.1;
end
for i = 1 : num_transformed_points
    v_diag(num_nodes*num_rownode+(i-1)*3+1:num_nodes*num_rownode+i*3,1) = 0.01;
end

P = inv(diag(v_diag));
M=0.000000001;
tic  %%%%timing
%while (sum((J'*F).^2))^(1/2)>M
min_FX_old = 10000000000000;
k=0;
x = x0;    % initialize x
while ((F'*P*F)>M&&abs(F'*P*F-min_FX_old)>M&&k<50)
    F = sparse(F);
    P = sparse(P);
    J = sparse(J);
    min_FX_old = F'*P*F
    d = -(J'*P*J)\(J'*P*F);
   
    x_new = x + d;
    F = CalculateF(x_new,ED_Parameter);
    min_FX = F'*P*F;
    if(min_FX > min_FX_old)
        alpha = LineSearch(x,d,F,P,ED_Parameter);
        x = x + alpha * d;
    else
        x = x + d;
    end
    
    k=k+1;
    F = CalculateF(x,ED_Parameter);
    J = JacobianF(x,ED_Parameter);
    
end
disp('Gauss-Newton algorithm');
% 
% disp('Real result is:');

disp('GN result is:');
min_FX = F'*P*F;
iteration=k

x_optimum = zeros(row,col);
for i = 1 : num_nodes
    x_optimum(3*(i-1)+1,1) = x((i-1)*12+1);
    x_optimum(3*(i-1)+2,1) = x((i-1)*12+2);
    x_optimum(3*(i-1)+3,1) = x((i-1)*12+3);
    x_optimum(3*(i-1)+1,2) = x((i-1)*12+4);
    x_optimum(3*(i-1)+2,2) = x((i-1)*12+5);
    x_optimum(3*(i-1)+3,2) = x((i-1)*12+6);
    x_optimum(3*(i-1)+1,3) = x((i-1)*12+7);
    x_optimum(3*(i-1)+2,3) = x((i-1)*12+8);
    x_optimum(3*(i-1)+3,3) = x((i-1)*12+9);
    x_optimum(end - num_nodes + i,1) = x((i-1)*12+10);
    x_optimum(end - num_nodes + i,2) = x((i-1)*12+11);
    x_optimum(end - num_nodes + i,3) = x((i-1)*12+12);
end


end

