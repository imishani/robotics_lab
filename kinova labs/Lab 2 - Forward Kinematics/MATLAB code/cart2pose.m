function [pose] = cart2pose(cart)
% Tait Bryan ZYX extrinsic
rot_z=[cosd(cart(6)) -sind(cart(6)) 0
    sind(cart(6)) cosd(cart(6)) 0
    0 0 1];
rot_y=[cosd(cart(5)) 0 sind(cart(5))
    0 1 0
    -sind(cart(5)) 0 cosd(cart(5))];
rot_x=[1 0 0
    0 cosd(cart(4)) -sind(cart(4))
    0 sind(cart(4)) cosd(cart(4))];
pose=[rot_z*rot_y*rot_x cart(1:3)'
    0 0 0 1];
end
