function fuel=Wfuel(a,b)
%计算两点之间生成航迹段的距离，作为后续启发信息的一部分
    fuel=sqrt(sum((a-b).^2));
end