map = load_map('map3.txt', 0.15, 1, .1);
start = [1  2 5];
stop  = [20  0 0];

[a1,b] =dijkstra(map,start,stop,true);

plot_path(map, a1)
