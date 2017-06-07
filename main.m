clear all
close all

obs1 = [5,3 ; 8,3; 8,12; 3,12; 3,7; 5,7];

start = [1,1];
goal = [12,12];
costmap_init = ones(12)*999;
isSearchedMap = zeros(size(costmap_init));
mapHeuristic = findHeuristicMap(size(costmap_init),goal);
searchList = [];


costmap=occupyObsMap(costmap_init,obs1);

initVal = findDistance(start,goal);
isSearchedMap(start)=1;
costmap(start) = initVal;
node=[1,initVal,start(1),start(2),0,0]; % [ID, Val, x, y, ID_prev,  cost_prev]
[costmap_update, searchListNew] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node);
costmap = costmap_update;
searchList=[searchList;searchListNew];

while(1)

    [r,c]=find(searchList==min(searchList(:,1)));
    node = [node; size(node,1)+1, searchList(r(1),1), searchList(r(1),2), searchList(r(1),3), searchList(r(1),4),  searchList(r(1),5)];
    isSearchedMap(searchList(r(1),2), searchList(r(1),3)) = 1;
    searchList(r(1),:)=[];

    if((node(size(node,1),3) == 12) && (node(size(node,1),4) == 12))
        break;
    end
    [costmap_update, searchListNew] = updateCostMap( costmap, mapHeuristic, isSearchedMap, node(size(node,1),:));
    costmap = costmap_update;
    searchList=[searchList;searchListNew];

end

path = [node(size(node,1),3),node(size(node,1),4)];
index_next = node(size(node,1),5);
while(1)
    path=[node(index_next,3),node(index_next,4);path];
    index_next = node(index_next,5);
    if(index_next == 0)
        break;
    end
end

figure(1)
fill(obs1(:,1), obs1(:,2), 'r');
hold on
plot(start(1),start(2),'o');
plot(goal(1),goal(2),'*');
plot(path(:,1),path(:,2),'x')

