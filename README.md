多约束路径规划算法对比与可视化评测平台

简介
- 项目目标：实现一个可加载网格地图的路径规划实验台，支持 Dijkstra、A* 算法，输出路径并对比耗时、扩展节点数与路径长度。当前项目统一使用 FLTK 做可视化界面。

最小可交付（MVP）
1. 支持读取简单网格地图（txt/csv）。
2. 实现 Map 类和 PathPlanner 抽象类。
3. 实现 Dijkstra 和 A* 两个算法并保证正确性。
4. 输出路径、耗时、扩展节点数到控制台与结果文件。
5. 提供 10 组测试用例（正常、边界、异常）。

构建与运行（示例：使用 g++）
```bash
mkdir build && cd build
g++ -std=c++17 -I../src ../src/*.cpp -o planner
./planner ../data/example_map.txt
```

后续扩展
- 加入 JPS/Theta*、多约束（转弯代价、区域代价）、批量评测脚本。

可视化说明（FLTK）

现在的可视化入口是 [src/fltk_visualizer.cpp](src/fltk_visualizer.cpp)。它可以直接加载地图、设置起终点、分别或同时运行 Dijkstra / A*，并在窗口里叠加显示路径和统计信息。

Windows 下建议直接用仓库自带的 FLTK 头文件和静态库编译：
```powershell
g++ -std=c++17 -Isrc -Ifltk\include src\main.cpp src\Map.cpp src\DijkstraPlanner.cpp src\AStarPlanner.cpp -o planner.exe
g++ -std=c++17 -Isrc -Ifltk\include src\fltk_visualizer.cpp src\Map.cpp src\DijkstraPlanner.cpp src\AStarPlanner.cpp -Lfltk\lib -lfltk -lfltk_jpeg -lfltk_png -lfltk_z -lgdiplus -lole32 -luuid -lcomctl32 -lws2_32 -lwinspool -mwindows -o fltk_visualizer.exe
```

运行时先启动 [fltk_visualizer.exe](fltk_visualizer.exe)，再在界面里点 Load Map 或直接使用默认地图。主程序 [planner.exe](planner.exe) 仍然保留，用于命令行评测和生成路径文件。