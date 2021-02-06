robot = rigidBodyTree;

a1 = 0;
a2 = 0.95;
a3 = 0.95;
a4 = 0;
a5 = 0;
a6 = 0;

d1 = 0.65;
d2 = 0;
d3 = 0;
d4 = 0;
d5 = 1.30;
d6 = 0.45; % varies between 0 and 0.45

alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = -pi/2;
alpha5 = 0;
alpha6 = 0;

q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;

dhparams = [
    a1   alpha1    d1   q1;
    a2   alpha2    d2   q2;
    a3   alpha3    d3   q3;
    a4   alpha4    d4   q4;
    a5   alpha5    d5   q5;
    a6   alpha6    d6   q6;
];

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','prismatic');

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot, body2, 'body1')
addBody(robot, body3, 'body2')
addBody(robot, body4, 'body3')
addBody(robot, body5, 'body4')
addBody(robot, body6, 'body5')

config = homeConfiguration(robot);
config(2).JointPosition = pi/2;
config(4).JointPosition = -pi/2;
config(6).JointPosition = 0.45;