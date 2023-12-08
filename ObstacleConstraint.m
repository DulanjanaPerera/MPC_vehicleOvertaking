function cineq = ObstacleConstraint(X,U,e,data,Ts, obs)


p = data.PredictionHorizon;

X1 = X(2:p+1,1);
X2 = X(2:p+1,2);

[r, c] = size(obs);
R = 4;
cineq = zeros(p*r,1);

for i=1:r

    cineq((i-1)*p + 1:i*p, 1) = R - sqrt( ( X1-obs(i,1) ).^2 + ( X2-obs(i,2) ).^2 );

end

% cineq = [R - sqrt( ( X1-obs(1,1) ).^2 + ( X2-obs(1,2) ).^2 );
%          R - sqrt( ( X1-obs(2,1) ).^2 + ( X2-obs(2,2) ).^2 )];


end