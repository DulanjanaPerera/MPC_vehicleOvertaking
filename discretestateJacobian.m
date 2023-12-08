function [A, B] = discretestateJacobian(x, u, Ts, obs)

L = 2.5;

A = zeros(4,4);

A(1,1) = 1;
A(1,3) = -Ts*x(4)*sin(x(3));
A(1,4) = Ts*cos(x(3));

A(2,2) = 1;
A(2,3) = Ts*x(4)*cos(x(3));
A(2,4) = Ts*sin(x(3));

A(3,3) = 1;
A(3,4) = Ts*tan(u(2))/L;

A(4,4) = 1 - Ts*1/25;


B = zeros(4,2);

B(3,2) = (x(4)/L)*( 1 / ( cos(u(2))*cos(u(2)) ) );
B(4,1) = Ts;

end