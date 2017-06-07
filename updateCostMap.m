function [costmap_update, search_list_new] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    costmap_update = costmap;
    isSearchedMap_update = isSearchedMap;
    new_node = [];
    arrayFrontier_new = [];
    search_list_new = [];
    p = [node(3),node(4)];
    for i=-1:1
        for j=-1:1
           if ((p(1) + i > 0) && (p(2) + j > 0) && (p(1) + i <= size(costmap,1)) && (p(2) + j  <= size(costmap,2)))
                if (( isSearchedMap(p(1)+i, p(2)+j) == 0) && costmap(p(1)+i, p(2)+j) ~= 1000)
                    cost_prev = node(6) + findDistance([p(1)+i, p(2)+j], [p(1), p(2)]);
                    value = cost_prev + mapHeuristic(p(1)+i, p(2)+j);
                    if(costmap(p(1)+i, p(2)+j) > value)
                        costmap_update(p(1)+i, p(2)+j) = value;
                        search_list_new = [search_list_new; value, p(1)+i, p(2)+j, node(1), cost_prev];     
                    end
                    
                end
           end
        end
    end


end



