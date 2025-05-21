function reval=ObsCollision(p1,p2,obs)
%   输入障碍物中心点和半径、待检测点
    for i = 1: size(obs, 1)
        center = obs(i, 1:2);
        radio = obs(i, 3);
        if(radio == 0) continue; end
        % 点在圆内
        if norm(center - p1) < radio || norm(center - p2) < radio
            reval = false;
            return;
        end
        % 直线 Ax + By + C = 0;  (y1 - y2) x + (x2 - x1) y + x1y2 - y1x2 = 0;
        if p1(1) == p2(1)
            A = 1;
            B = 0;
            C = -p1(1);
        elseif p1(2) == p2(2)
            A = 0;
            B = 1;
            C = -p1(2);
        else
            A = p1(2) - p2(2);
            B = p2(1) - p1(1);
            C = p1(1) * p2(2) - p1(2) * p2(1);
        end
        dist1 = (A * center(1) + B * center(2) + C) ^ 2;
        dist2 = (A * A + B * B) * radio * radio;
        if dist1 > dist2
            continue;
        end
        angle1 = (center(1) - p1(1)) * (p2(1) - p1(1)) + (center(2) - p1(2)) * (p2(2) - p1(2));
        angle2 = (center(1) - p2(1)) * (p1(1) - p2(1)) + (center(2) - p2(2)) * (p1(2) - p2(2));
        if angle1 > 0 && angle2 > 0
            reval = false;
            return;
        end
    end
    reval = true;
end


