function map_out = occupyObsMap( map, obs )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    s_map = size(map);
    n = size(obs,1);
    cnt = 0;
    map_out = map;
    for i = 1:s_map(1)
       for j = 1:s_map(2)
           p = [i,j];
           if(isInside(obs, n,p))
              cnt = cnt + 1;
              map_out(i,j) = 1000;
           end
       end
    end
    
end

