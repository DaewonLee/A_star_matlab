%   Created on: June 6, 2017
%   Author: Daewon Lee
%   email: lee.daewon@gmail.com

%% Setting
clear all; close all 

obs1 = [5,3 ; 8,3; 8,12; 3,12; 3,7; 5,7]; % obstacle polygon
obs2 = [10,9; 10,12; 11,12; 11,9];

start = [1,11]; % start position
goal = [12,12]; % goal 
initialCost = 999;
costmap_init = ones(12)*initialCost;  % creat map (12x12) , initialCost should be bigger than possible cost in the map
mapSize = size(costmap_init);
isSearchedMap = zeros(size(costmap_init)); % 0: not visited node, 1: visited node
mapHeuristic = findHeuristicMap(size(costmap_init),goal); % heuristic map with euclidian distance
searchList = [];

costmap=occupyObsMap(costmap_init,obs1); % occupy the map with obstacle
costmap=occupyObsMap(costmap,obs2);

%% Initialization
initVal = findDistance(start,goal); 
isSearchedMap(start)=1;
costmap(start) = initVal;
node=[1,initVal,start(1),start(2),0,0]; % [ID, cost, x, y, ID_prev,  cost from start position to this position]
[costmap_update, searchListNew] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node);
costmap = costmap_update;
searchList=[searchList;searchListNew]; % [cost, x, y, node ID, cost from start position to this position]

%% Main Loop
while(1)

    [r,c]=find(searchList==min(searchList(:,1))); % find an index (r) of a not-visited node that has lowest cost 
    node = [node; size(node,1)+1, searchList(r(1),1), searchList(r(1),2), searchList(r(1),3), searchList(r(1),4),  searchList(r(1),5)]; % update node
    isSearchedMap(searchList(r(1),2), searchList(r(1),3)) = 1; % update isSearchedMap
    searchList(r(1),:)=[];

    if((node(size(node,1),3) == mapSize(1)) && (node(size(node,1),4) == mapSize(2)))
        break;
    end
    [costmap_update, searchListNew] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node(size(node,1),:));
    costmap = costmap_update;
    searchList=[searchList;searchListNew]; % update searchList

end

%% Find Optimal Path
path = [node(size(node,1),3),node(size(node,1),4)]; 
index_next = node(size(node,1),5);
while(1)
    path=[node(index_next,3),node(index_next,4);path];
    index_next = node(index_next,5);
    if(index_next == 0)
        break;
    end
end

%% Plot
figure(1)
fill(obs1(:,1), obs1(:,2), 'r');
hold on
fill(obs2(:,1), obs2(:,2), 'r');
plot(start(1),start(2),'o');
plot(goal(1),goal(2),'*');
plot(path(:,1),path(:,2),'x')
axis([0 mapSize(1) 0 mapSize(2)])
