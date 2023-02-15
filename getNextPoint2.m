function son = getNextPoint2(father, sample)
    %% father 父节点(x,y,z,phi,gamma,pitch)  phi0 航迹角  Gamma 滚转角 pitch俯仰角 v速度
    pai = 3.1415926 * 2;
    v = 1; %速度
    deltaT = 0.1; %时间步长
    GammaStep = 5/360 * pai; %滚转角最大步长
    pitchstep = 10/360 * pai; %俯仰角最大步长

    % GammaMax=20/360*pai;
    % GammaMin=-20/360*pai;
    % pitchMax=40/360*pai;
    % pitchMin=-40/360*pai;
    %滚转角
    gamma = sample(1, 5);

    if gamma > father(1, 5) + GammaStep
        gamma = father(1, 5) + GammaStep;
    elseif gamma < father(1, 5) - GammaStep
        gamma = father(1, 5) - GammaStep;
    end

    % if gamma>GammaMax
    %     gamma=GammaMax;
    % elseif gamma<GammaMin
    %     gamma=GammaMin;
    % end
    %俯仰角
    pitch = sample(1, 6);

    if pitch > father(1, 6) + pitchstep
        pitch = father(1, 6) + pitchstep;
    elseif pitch < father(1, 6) - pitchstep
        pitch = father(1, 6) - pitchstep;
    end

    % if pitch>pitchMax
    %     pitch=pitchMax;
    % elseif pitch<pitchMin
    %     pitch=pitchMin;
    % end
    if gamma ~= 0
        Rou = 1 / (v * v) * tan(gamma); %计算水平转弯曲率
        deltaPhi = Rou * v * deltaT; %航迹角变化
        phi = father(4);
        rotation = [cos(phi) -sin(phi) 0;
                  sin(phi) cos(phi) 0;
                  0 0 1];
        a = [sin(deltaPhi) / Rou; ((1 - cos(deltaPhi)) / Rou); deltaPhi];
        temp = (rotation * a)' + [father(1), father(2), phi];
        z = father(3) + v * sin(pitch);
        son = [temp(1), temp(2), z, temp(3), gamma, pitch];
    else
        phi = father(4);
        z = father(3) + v * sin(pitch);
        rotation = [cos(phi) -sin(phi);
                  sin(phi) cos(phi)];
        temp = (rotation * [v * deltaT; 0])' + [father(1), father(2)];
        son = [temp(1), temp(2), z, phi, gamma, pitch];
    end

end
