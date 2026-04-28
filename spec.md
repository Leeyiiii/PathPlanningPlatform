项目说明与技术规范

1. 输入数据格式
- 网格地图（TXT）：每行一个行数据，0 表示可通过单元，1 表示障碍。例如：
  0 0 1 0 0
  0 0 0 1 0
- 起终点：单独文件或命令行参数，格式为 "sx,sy tx,ty"（0-based 索引）。
- 代价图（可选）：同尺寸矩阵，存放每格的通过代价（float）。

2. 主要类（接口概览）
- `Map`：读取/保存地图、提供邻居查询、代价查询。
- `PathPlanner`（抽象）：接口 `bool plan(const Map&, const Point& s, const Point& t)`，`std::vector<Point> getPath()`，`Stats getStats()`。
- `DijkstraPlanner` / `AStarPlanner`：继承 `PathPlanner` 实现具体算法。
- `ConstraintManager`：可注册约束（转向惩罚、区域代价覆盖等）。
- `PerformanceEvaluator`：接受多个算法与多个地图，输出对比表（CSV）。
- `PlannerVisualizer`（可选）：FLTK 界面封装，负责渲染地图、动画、参数面板。

3. 输出格式
- 路径文件（TXT）：每行一个坐标 "x y"。
- 评测文件（CSV）：包含 map_id, algorithm, success, time_ms, path_length, nodes_expanded。

4. 开发里程碑（建议）
- Day 1-2: Map + IO + 简单示例
- Day 3-5: Dijkstra + A* 实现与单元测试
- Day 6: Stats & 批量评测脚本
- Day 7-9: 可视化（FLTK）或结果可视化脚本
- Day 10-12: 测试、文档、演示准备

5. 测试用例建议
- 空地图（无障碍）
- 单一障碍
- 复杂迷宫
- 无解地图
- 大尺寸随机地图（压力测试）

6. 参考文献（建议）
- E. W. Dijkstra, "A note on two problems in connexion with graphs", 1959.
- P. E. Hart, N. J. Nilsson and B. Raphael, "A formal basis for the heuristic determination of minimum cost paths", 1968.
