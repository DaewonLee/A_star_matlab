%   Created on: June 6, 2017
%   Author: Daewon Lee
%   email: lee.daewon@gmail.com
function [costmap_update, search_list_new] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node)
    costmap_update = costmap;
    isSearchedMap_update = isSearchedMap;
    new_node = [];
    arrayFrontier_new = [];
    search_list_new = [];
    p = [node(3),node(4)];
    for i=-1:1
        for j=-1:1
           if ((p(1) + i > 0) && (p(2) + j > 0) && (p(1) + i <= size(costmap,1)) && (p(2) + j  <= size(costmap,2))) % check if the node is inside of the map
                if (( isSearchedMap(p(1)+i, p(2)+j) == 0) && costmap(p(1)+i, p(2)+j) ~= 1000) % 1000 : obstacle
                    cost_prev = node(6) + findDistance([p(1)+i, p(2)+j], [p(1), p(2)]); % node(6) : cost from start position to this position
                    value = cost_prev + mapHeuristic(p(1)+i, p(2)+j); % total cost
                    if(costmap(p(1)+i, p(2)+j) > value) % update the cost map & searchList only if the new cost is lower than previous cost
                        costmap_update(p(1)+i, p(2)+j) = value;
                        search_list_new = [search_list_new; value, p(1)+i, p(2)+j, node(1), cost_prev];     
                    end
                    
                end
           end
        end
    end


end



