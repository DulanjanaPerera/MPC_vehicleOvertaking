function z = discreteStateEq(x, u, Ts, obs)
% x - [X; Y; psi; V]
% u - [a; delta]


L = 2.5; % lenght of the vehicle

z = zeros(4, 1);
z(1) = x(1) + Ts*x(4)*cos(x(3));
z(2) = x(2) + Ts*x(4)*sin(x(3));
z(3) = x(3) + (x(4)/L)*tan(u(2));
z(4) = x(4) + Ts*u(1) - x(4)/25;

end