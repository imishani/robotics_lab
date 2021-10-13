function [cart] = pose2cart(pose)
% Angles Tait Bryan ZYX extrinsic
cart=zeros(1,6);
cart(1,1:3)=pose(1:3,4)';
if (pose(3,1)==1)||(pose(3,1)==-1)
        cart(1,4:6)=[atan2d(pose(1,2),-pose(1,3)),90,0];
else
    cart(1,4:6)=[atan2d(pose(3,2),pose(3,3)),atan2d(-pose(3,1),sqrt(pose(2,1)^2+pose(1,1)^2)),atan2d(pose(2,1),pose(1,1))];

end
cart(1,4:6)=wrapTo180(cart(1,4:6));
end