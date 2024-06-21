1. topomap_loop3 由image_bag5生成dt=1 -r 2.0 效果巨差
2. topomap_loop4 由image_bag5生成dt=1 -r 2.5 效果巨差
3. topomap_loop5 由image_bag5生成dt=1 -r 3.0 效果巨差
4. topomap_loop6 由image_bag5生成dt=1 -r 1.5 
5. topomap_loop7 由image_bag5生成dt=1 -r 1.5 ros2 图片格式bgr8，保存图片格式为 rgb8 不太行

目前测试：

./my_navigation.py --dir=topomap_loop7 -t=6 -w=4 # 图片格式应该一直保持为bgr8，不应该转换为rgb8保存，不太行

./my_navigation.py --dir=topomap_loop6 -t=6 -w=4


./my_navigation.py --dir=topomap_loop6 -t=6 -w=4 -g=145

# 能导航
./my_navigation.py --dir=pose_image_bag3 --waypoint=4

