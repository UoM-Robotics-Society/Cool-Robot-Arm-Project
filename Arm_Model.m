body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
tform = trvec2tform([0, 0, 0]);
setFixedTransform(jnt1,tform);
axis = [0,0,1];
jnt1.JointAxis = axis;
body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base');

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; % User defined
tform2 = trvec2tform([0, 0, 0.055]); % User defined
setFixedTransform(jnt2,tform2);
axis = [0,1,0];
jnt2.JointAxis = axis;
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.HomePosition = 0; % User defined
tform3 = trvec2tform([0, 0, 0.1]); % User defined
setFixedTransform(jnt3,tform3);
axis = [0,1,0];
jnt3.JointAxis = axis;
body3.Joint = jnt3;
addBody(robot,body3,'body2'); % Add body3 to body2

body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.HomePosition = 0; % User defined
tform4 = trvec2tform([0, 0, 0.1]); % User defined
setFixedTransform(jnt4,tform4);
axis = [0,1,0];
jnt4.JointAxis = axis;
body4.Joint = jnt4;
addBody(robot,body4,'body3'); % Add body3 to body2

body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
jnt5.HomePosition = 0; % User defined
tform5 = trvec2tform([0, 0, 0.06]); % User defined
setFixedTransform(jnt5,tform5);
axis = [0,0,1];
jnt5.JointAxis = axis;
body5.Joint = jnt5;
addBody(robot,body5,'body4'); % Add body3 to body2

body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
jnt6.HomePosition = 0; % User defined
tform6 = trvec2tform([0, 0, 0.115]); % User defined
setFixedTransform(jnt6,tform6);
body6.Joint = jnt6;
addBody(robot,body6,'body5'); % Add body3 to body2

config = homeConfiguration(robot);