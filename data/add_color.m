ptCloud = pcread('~/map_ws/src/map_registration/data/reactor_phone_ascii.pcd')
ptCloud2 = pcread('~/map_ws/src/map_registration/data/reactor2_ascii.pcd')

colors = ones(ptCloud2.Count,3);
colors = colors.*[156,175,136];
ptCloud2.Color = cast(colors,"uint8");

cd ~/map_ws/src/map_registration/data/
pcwrite(ptCloud2,'reactor_colored.pcd','Encoding','ascii');