# constraint-update 分支版本说明

日期：2026-09-28  
分支：`constraint-update`

## 版本目标

将原来的排气管统一管径约束，调整为空调管的工艺约束基础。依据：

- `doc/SAAA工艺要求-20260706.xlsx`
- `doc/管路路径布置工艺约束纪要.docx`
- `model/ACpipe/Product_Assemble.CATProduct` 的 CATIA 模型概念

当前测试对象是一条单独管路。金属铝管作为连续芯管，胶管作为局部包覆层；分支拓扑和连接点位置暂不参与本轮优化。

## 主要修改

### 规格与工艺参数

`settings.py` 新增 4 种金属管和 3 种胶管规格，记录外径、内径、壁厚、刚性/柔性、弯曲半径和直线段要求。

- 金属管弯曲半径按 `max(Excel 明确值, 1.5D)` 形成硬下限。
- 胶管使用 R50、R60、R75 的规格硬下限。
- 半径碰撞失败时每次递减 10 mm，但不允许低于当前规格硬下限。
- 小于 5° 的几何变化合并为直线；5°～10°允许，但加入评分惩罚。
- 焊点按中心线累计距离检查，推荐 30 mm、极限 25 mm。
- 阀座两侧分别检查直线段，推荐 25 mm、极限 5 mm。
- 铝套背部按已知轴长配置；铝套中心轴周围使用 R80 推荐、R65 极限避让区域。
- 连接件型号只记录，不参与本轮路径验证。

### 规划空间与障碍物

- 规划空间使用当前测试路线的最大外部包络半径。
- 半透明观察对象设置为忽略，不作为硬障碍或软障碍。
- `ground` 只用于确定搜索空间的 Z 下限和离地净空。
- 规划空间 JSON 继续保留 `pipe_radius`，并追加主动规格和包络半径信息。

### 路线与建模

- `main2.py` 将主动规格和管段配置写入理论路线结果，并对 5°～10° 小弯增加评分惩罚。
- `main3.py` 增加按规格的半径搜索、5°以下转角合并和工艺约束报告。
- `main4.py` 默认从工程化路线读取金属芯管半径。
- `pipeline.py` 增加 CATIA 建模阶段。

完整流程为：

1. CATIA 点测量；
2. 切片空间展示；
3. 理论化路线展示；
4. 工程化路线展示；
5. 调用 CATIA 建模。

## 当前配置边界

`PIPE_SEGMENTS`、`WELD_POINTS`、`VALVE_SEATS` 和 `ALUMINUM_SLEEVES` 已建立配置入口，当前为空，等待工程师提供管段和工艺件位置数据。

因此当前 `main4.py` 生成连续金属芯管；胶管局部包覆和铝套实体需要管段位置数据后再建模。CATProduct 仍需先通过 CATIA 导出 IGES，规划程序不直接读取 CATProduct 内部规格和分支拓扑。

## 已完成验证

- `python -m py_compile settings.py common_space.py main2.py main3.py main4.py pipeline.py`
- `git diff --check`
- `pipeline.py --dry-run`
- 规格表、R28.575 硬下限和半径递减逻辑检查
- 5°以下转角合并检查
- 软障碍关闭检查
- 铝套 R80/R65 检查函数检查
- `upload.py --dry-run` 对 `Product_Assemble.CATProduct` 的识别检查

由于当前 `base/` 没有规划输入，且没有可用的 CATIA COM 实例，本分支尚未完成真实的 `main1 → main2 → main3 → main4` 全流程运行。
