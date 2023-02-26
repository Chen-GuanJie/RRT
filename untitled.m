a = 5; % 长轴

b = 3; %短轴

fac = 5; % 长轴焦点

fbc = 3; % 短轴焦点

numSamples = 100; % 采样数

% 生成随机数
us = rand(1, numSamples);

% 椭圆方程
% t = linspace(0, 2 * pi, numSamples);
% x = fac + a * cos(t);
% y = fbc + b * sin(t);
% 
% % 在椭圆范围内随机采样点
% samples = [x(us>0.5) ; y(us>0.5)];
ra=rand(1, 2);
for i=1:1000
% ra=rand(1, 3);
% x = fac + ra(1)*a * cos(ra(3)*2 * pi);
% y = fbc + ra(2)*b * sin(ra(3)*2 * pi);
% scatter(x,y);hold on
ra=rand(1, 2); 
xx=[ra(1)*2*a ra(2)*2*b]-[a b];


scatter(xx(1),xx(2));hold on

end
