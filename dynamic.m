robot_x=0:0.01:0.2;
robot_y=0:0.01:0.2;
L=0.2;

robot_y=robot_y*0;
vlabs=0.5; %  m/s
vrabs=0.5; %  m/s
lpoint=[-0.1;0];
rpoint=[0.1;0];
body=(rpoint-lpoint)/norm(rpoint-lpoint);
theta_body=atan2(body(2),body(1));
theta=theta_body+pi/2;
vl=vlabs*([cos(theta);sin(theta)])/norm([cos(theta);sin(theta)]);
vr=vrabs*([cos(theta);sin(theta)])/norm([cos(theta);sin(theta)]);
lpoint=lpoint+vl*0.01;
rpoint=rpoint+vr*0.01;
t=0;
k=0;
p=0;
while p<2
while k<2
while t<10
body=(rpoint-lpoint)/norm(rpoint-lpoint);
theta_body=atan2(body(2),body(1));
theta=theta_body+pi/2;
vl=vlabs*([cos(theta);sin(theta)])/norm([cos(theta);sin(theta)]);
vr=vrabs*([cos(theta);sin(theta)])/norm([cos(theta);sin(theta)]);
lpoint=lpoint+vl*0.01;
rpoint=rpoint+vr*0.01;
t=t+0.1;
body=(rpoint-lpoint)/norm(rpoint-lpoint);
robot=[lpoint,lpoint+body*0.5,rpoint];
robotx=robot(1,:);
roboty=robot(2,:);
plot(robotx,roboty,'r')
hold on
plot(robotx,roboty,'r')
axis equal
axis([-3 3 -2 6])
cor=-(0.2*vlabs)/(vrabs-vlabs);

ro=cor*(body)+lpoint;
com=rpoint+body*0.1;
radius=sqrt((ro(2)-com(2))^2+(ro(1)-com(1))^2);
theta=0:0.01:2*pi;
moving_x = radius*cos(theta)+ro(1);
moving_y = radius*sin(theta)+ro(2);
plot(ro(1),ro(2),'r*')

hold on
plot(moving_x,moving_y,'b')
hold on
pause(0.0001);
hold off
plot(com(1),com(2),'b*')
hold on
end
vlabs=0.5; %  m/s
vrabs=vrabs+0.1; %  m/s
k=k+1;
t=0;
end
vlabs=vrabs+0.1; %  m/s
p=p+1;
t=0;
k=0;

end 
