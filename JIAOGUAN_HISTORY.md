# 胶管与双夹具迭代历史

本文件合并了 2026-10-16 胶管约束运行记录和 2026-10-28 双夹具方向及局部管间干涉修复记录。运行目录中的中间产物按 `PROJECT_CLEANUP_20261028.md` 清理，以下数值、决策和失败原因作为长期迭代记录保留。

## 2026-10-16 胶管约束运行


> 2026-10-28 更正：下述历史 `formal_valid=true` 不代表通过局部胶管/铝套管间干涉或双夹验收。已确认旧管 1/2 局部扫掠包络干涉，详细证据、约束和代码修改见 [双夹具修复记录](JIAOGUAN_HISTORY.md#2026-10-28-双夹具方向及局部管间干涉修复)。静态障碍净空与管间净空必须区分。

## 本次修复与配置

此前 `PipeInstance.outer_radius` 会把局部 15 mm 胶管的半径 7.5 mm 用作整条路线和共享规划空间的包络。这样 PIPE_02 的 GUADIAN 桥接也按 15 mm 外径搜索，导致旧运行报 `required guardian cannot be connected without collision`。这个失败与两个 GUADIAN 点之间约 19.861 mm 的点间距离不是同一个判定。

现在连续金属芯按 `metal_9` 规划，`planning_radius=4.5 mm`；胶管和铝套只在各自的局部弧长区间使用 7.5 mm 半径。局部净空不足时，会在路线中搜索可行位置并记录 `hose_position_auto_shifted`。R50 仍是警告，不阻断输出。

`outputs/multi_pipe_config_20261010.json` 的 `PIPE_01`～`PIPE_04` 使用相同的 `pipe_segments`：

```json
{
  "core_spec": "metal_9",
  "cover_spec": "hose_8DE3",
  "min_length_mm": 30.0,
  "length_options_mm": [30.0, 75.0],
  "hose_length_mm": 30.0,
  "sleeve_length_mm": 30.0,
  "minimum_bend_radius_mm": 50.0
}
```

胶管外径由 `hose_8DE3` 定义为 15 mm；两端铝套长度各 30 mm；路线不足总跨度时回退整根铝管。该配置位于被 `.gitignore` 忽略的 `outputs/` 目录，本文件用于保留本次配置变化的跟踪记录。

## 正式运行

使用 `multi_pipeline.py`、`outputs/measured_points_20261010.json` 和新的 `run_id`：

```powershell
D:\Miniforge3\envs\occ\python.exe multi_pipeline.py outputs\multi_pipe_config_20261010.json --points outputs\measured_points_20261010.json --run-id run_20261016_jiaoguan30_coreplan --reuse-space-json outputs\run_20261015\planning_space.json
```

`outputs/run_20261016_jiaoguan30_coreplan/manifest.json` 状态为 `completed`，PIPE_01～PIPE_04 均为 `formal_valid=true`，共享空间 `max_outer_radius=4.5`。四根管均生成铝管、30 mm 铝套、30 mm 胶管、30 mm 铝套、铝管五段；PIPE_01～PIPE_03 的 GUADIAN 全部满足，测点文件没有 `GUADIAN_04#1`。

该正式运行已生成：

- `catia/PIPE_01.CATPart` ～ `catia/PIPE_04.CATPart`
- `routes_guide.html`
- `multi_route_engineered.html`

## 局部净空复核

对同一批工程路线执行局部胶管净空复核：

- PIPE_01：最小局部余量约 42.93 mm；
- PIPE_02：最小局部余量约 28.76 mm，`GUADIAN_02#1` 在路线弧长约 349.10 mm，胶管在约 307.53～337.53 mm，胶管不承担该 GUADIAN 桥接；
- PIPE_03：最小局部余量约 32.23 mm；
- PIPE_04：中点不足，自动移动组件中心到约 265 mm，最小余量约 12.91 mm，并记录 `hose_position_auto_shifted`。

这些局部复核结果位于 `outputs/run_20261016_jiaoguan30_coreplan_shifted/` 的各 `engineering.json` 中；该验证运行已写完四根管的 JSON，但在重复生成大体积 HTML/CATIA 后处理前停止，不能把它的 `manifest.json` 视为完成运行。正式 HTML/CATIA 文件采用上面的 `coreplan` 运行。

## 验证

- `python -m unittest discover -s tests -p "test_hose_segments.py" -v`：7 项通过。
- `python -m py_compile multi_pipeline.py pipe_config.py hose_segments.py main3.py main4.py`：通过。
- `git diff --check`：通过。
- 当前 OCC 环境没有安装 `pytest` 模块，因此 `test_pipe_config.py` 未能在该解释器下执行。

## 2026-10-28 双夹式挂点约束复核与修改

### 原因定位

对 `outputs/run_20261016_jiaoguan30_finalaudit` 的 PIPE_01/PIPE_02 工程路线逐点复核后：

- `GUADIAN_01#1` 与 `GUADIAN_02#1` 的实测中心距为 `19.8613755 mm`；
- 两个挂点都被路线准确经过，挂点到中心线距离约为 `0 mm`；
- 两个挂点处的路线切向与挂点连线的夹角分别约为 `48.52°` 和 `47.41°`；
- 因而两条路线相对挂点连线的垂直偏差约为 `41.48°` 和 `42.59°`。

主要问题不是胶管半径，也不是 19.861 mm 测量值本身。旧算法只检查每根路线是否经过自己的 GUADIAN 点，管间检查还错误地统一使用 4.5 mm 半径；它没有表达双夹姿态约束。对于局部平行管线，实际轴距是挂点连线在垂直于管轴方向上的投影，斜穿时会小于挂点中心距。完整证据和最终修复结果见 [2026-10-28 双夹具记录](JIAOGUAN_HISTORY.md#2026-10-28-双夹具方向及局部管间干涉修复)。

19.861 mm 仍然是有效的中心距检查：在两个挂点处都使用 15 mm 局部外径时，理论中心距余量为：

```text
19.8613755 - 7.5 - 7.5 = 4.8613755 mm
```

但这个标量余量只有在两根管路在双夹处近似垂直于挂点连线时，才代表夹具截面内的有效间距。

### 新约束定义

本次运行配置加入：

```json
{
  "double_clamps": [{
    "name": "DOUBLE_CLAMP_PIPE_01_PIPE_02",
    "pipe_a": "PIPE_01",
    "guardian_a": "GUADIAN_01#1",
    "pipe_b": "PIPE_02",
    "guardian_b": "GUADIAN_02#1",
    "tangent_tolerance_deg": 5.0
  }]
}
```

未列入 `double_clamps` 的挂点继续按单夹处理。双夹两侧必须引用各自管路的 `required_hangers`；两条路线在对应挂点处的中心线切向必须与两挂点连线保持 90°，默认允许偏差 5°；同时报告两挂点中心距、两侧局部外半径和中心距净间隙。GUADIAN 和双夹方向都是正式路线的硬约束。

### 代码修改

- `pipe_config.py`：新增 `DoubleClamp` 配置模型，解析并校验顶层 `double_clamps`，为两根管生成局部双夹切向约束。
- `main3.py`：硬挂点桥接方向投影到垂直于双夹连线的平面，并在 `protected_waypoint_geometry` 中记录切向偏差。
- `route_clearance.py`：新增 `path_tangent_at_point()` 和 `double_clamp_report()`，计算中心距、局部外半径、净间隙和切向偏差。
- `multi_pipeline.py`：两根相关路线都生成后执行双夹报告；失败会记录 `blocking_reasons=double_clamp_constraint`，不允许正式 CATPart 导出。
- `README.md`：增加双夹配置格式和单夹/双夹边界说明。

### 当前旧路线的判定

对旧 `run_20261016_jiaoguan30_finalaudit` 路线运行新报告：中心距 `19.8613755 mm`，15 mm 局部外径下净间隙 `4.8613755 mm`；PIPE_01 垂直偏差约 `41.48°`，PIPE_02 垂直偏差约 `42.59°`，双夹报告为 `valid=false`。因此图中旧 PIPE_01/PIPE_02 布置不能作为双夹式夹具的正式布置。下一次正式多管运行必须使用带 `double_clamps` 的配置，并重新检查路线、双夹报告、HTML 和 CATIA 输出。




---

## 2026-10-28 双夹具方向及局部管间干涉修复


分支：`feat/JIAOGUAN`。本次依据用户提供的两张截图、新确认的单夹/双夹区别，以及保存的测点和工程路线复核。截图不包含 run_id；数值证据来自 `outputs/run_20261016_jiaoguan30_finalaudit`，不将截图像素作为测量依据。

## 原因及上次结论更正

测点文件 `outputs/measured_points_20261010.json`：

| 挂点 | X / mm | Y / mm | Z / mm |
| --- | ---: | ---: | ---: |
| GUADIAN_01#1 | 563.4221576112253 | 70.15800000000172 | 247.82735786450417 |
| GUADIAN_02#1 | 563.2917924454263 | 71.5290000000017 | 228.01378673450475 |

点间距离为 **19.861375506 mm**。原输出胶管和铝套半径均为 **7.5 mm**，铝管半径为 **4.5 mm**。没有发现这两个数值录入错误。

上次把 `19.861 - 7.5 - 7.5 = 4.861 mm` 直接解释为管壁净空是不充分的，现明确撤回该推论。挂点距离不是曲线的最近距离。若两条局部平行直线方向为单位向量 t、挂点位移为 d，则轴线最短距离为：

```text
axis_distance = ||d - dot(d,t) * t|| = ||d|| * sin(theta)
```

只有轴线与挂点连线垂直时，该距离才等于挂点距离。对于不平行或弯曲路线，需要对有限线段/弧的实际位置求最近距离，不能直接套平行直线公式。

旧工程路线复核：

| 检查项 | 数值 / 结果 |
| --- | --- |
| 两挂点中心距 | 19.861375506 mm |
| 管 1 挂点切向与连线的锐角 | 48.520093°，偏离垂直 41.479907° |
| 管 2 挂点切向与连线的锐角 | 47.408930°，偏离垂直 42.591070° |
| 挂点相邻直线区间的最短轴距 | 14.360600444 mm |
| 上述直线按两侧 7.5 mm 半径计算的净空 | −0.639399556 mm |
| 两管局部组件的最不利组合 | 铝套对铝套，轴距 10.755160115 mm |
| 上述组件净空 | **−4.244839885 mm，干涉** |
| 旧逻辑按两侧 4.5 mm 统一验收 | +1.755160115 mm，被错误放行 |

因此存在两个算法问题：没有双夹具姿态定义；上一轮把金属规划半径用于最终管间检查，漏掉局部 15 mm 胶管/铝套。之前 28.76 mm 等局部余量来自静态障碍截面，不能证明管 1 与管 2 不干涉。

数值报告：`outputs/double_clamp_audit_20261028.json`，包含测点文件 SHA256、各组件组合的最近点坐标、旧/新验收比较。这里的净空是基于工程路径采样折线的扫掠包络检查，并非 CATIA 精确实体布尔干涉检验。

## 已实现的约束

单夹仍只要求经过本管 GUADIAN，并沿连续直线穿过挂点；方向随本管路线确定。双夹必须显式配置两个管号及两个挂点，不按编号邻近或空间距离自动配对。

```json
{
  "double_clamps": [{
    "name": "DOUBLE_CLAMP_PIPE_01_PIPE_02",
    "pipe_a": "PIPE_01",
    "guardian_a": "GUADIAN_01#1",
    "pipe_b": "PIPE_02",
    "guardian_b": "GUADIAN_02#1",
    "tangent_tolerance_deg": 5.0
  }]
}
```

该定义已写入 `outputs/multi_pipe_config_20261010.json`；其余挂点没有被标为双夹。胶管 30 mm、两端铝套各 30 mm、铝管直径 9 mm 保留。

令 d 为两挂点连线，双夹约束为：两管分别经过自己的挂点；挂点处两侧保持连续直线；两侧切向 t1、t2 均与 d 垂直。实现以 90°为目标，5°仅为当前可配置的数值验收默认值，尚非工程师确认的制造公差。夹具附近以外的弯曲段仍必须通过完整管间净空验收。

两点本身不能确定夹具绕连线的安装转角，本次没有凭截图固定额外的三维孔轴方向；按本次要求约束垂直性。若以后提供夹具孔轴测量，可进一步指定共同切向。

## 修改位置与调用链

- `pipe_config.py`：解析 `DoubleClamp`、校验管号/挂点引用、拒绝重合挂点；合并新测点时重建连线约束。
- `main2.py`：规划插入挂点时，双夹挂点同时插入垂直于连线的局部直线桥；连线均检查障碍和已预留管路。预留管路保留各组件的实际外半径。
- `main3.py`：工程化及圆角半径搜索保留上述方向约束；`protected_waypoint_geometry.points[].tangent_constraint` 记录垂直偏差。局部胶管位置搜索同时检查静态障碍和预留管路，无可行位置时拒绝正式验收。R50 仍只警告。
- `route_clearance.py`：在组件弧长边界切分路线，按铝管/胶管/铝套实际半径逐对检查；提供挂点切向、局部半径和双夹报告。
- `multi_pipeline.py`：验收时保留完整工程路线的组件元数据；两根相关管路都存在后检查双夹；一侧失败则整对夹具不能正式导出。报告进入 JSON 和 manifest。
- `main4.py`：直接调用 CATIA 导出时也拒绝已知双夹或挂点几何失败。

## 验证边界

旧路线的新报告明确失败；静态测点未改。独立单管重建曾得到两侧均垂直的局部结果，但这不证明二者组合后整条路线无干涉。

首次 `run_20261028_doubleclamp_json` 为诊断调用，禁用了 HTML 导出并跳过 CATIA。发现管间检查遗漏后主动中止，manifest 已标记 `aborted`，不是成功输出。

最终代码通过 `run_20261028_doubleclamp_pair_validation` 对 PIPE_01/02 顺序规划、预留、工程化和联合验收进行了数值验证；HTML/CATIA 后处理明确禁用。旧 CATPart/HTML 均未覆盖，不能将旧输出当作双夹修复后的实体结果。

## 最终数值验证结果

临时验证脚本 `outputs/validate_doubleclamp_pair.py`（本次清理已删除，验证结果已合并到本记录） 读取同一份四管配置与测点，仅选取前两根管，调用真实 `multi_pipeline.run_multi_pipe()`，复用 `outputs/run_20261015/planning_space.json`。为了避免与本次几何验证无关的 STEP 网格加载，脚本临时禁用了两项 HTML 导出函数，使用 `skip_catia=True`；该目录的 `completed` 仅指这两根管的数值流程完成。

| 项目 | PIPE_01 | PIPE_02 |
| --- | --- | --- |
| 选定路线 | 01_bidir_balanced | 03_bidir_shortest |
| formal_valid | true | true |
| GUADIAN 到路线距离 | 0 mm | 0 mm |
| 挂点切向与连线夹角 | 90° | 90° |
| 挂点两侧保留直线 | 21.006 / 24.556 mm | 20.949 / 25.537 mm |
| 胶管中心弧长 | 324.362 mm | 520.764 mm |
| 胶管中心占全管比例 | 50% | 79.3574% |
| 胶管位置 | 默认中点 | 自动滑动避让 |
| 胶管 R50 | 仍有 R15 警告，按约定保留 | 无 R50 警告 |

双夹位置两侧实际半径为 7.5 / 4.5 mm（管 2 在该处为铝管），所以该处中心距减半径为 **7.861375506 mm**。完整两管所有组件之间的最小采样扫掠净空为 **+0.476271495 mm**，最不利组合是管 1 铝套对管 2 铝管；这比只看夹具位置更严格，已通过 `required_clearance=0` 的数值检查。

同时发现并修正了 `multi_pipeline.py` 保存验收报告时引用首个诊断候选的问题：`selected_reports` 现在保存真正选中候选的 guide/final 报告。已从本次最终路线 JSON 重新计算并保存 `PIPE_02/clearance.json`、`validation.json`；manifest 的 `post_run_clearance_recomputed=true` 标明这一点。选中 guide 净空为 0.623767158 mm，最终工程路线净空为 0.476271495 mm。

验证通过的范围是测点、双夹垂直性、局部组件位置、采样包络无重叠和几何/约束数值验收；**未运行新的四管联合验收、未导出新的 HTML/CATPart、未做 CATIA 精确实体干涉检验**。当前不到 0.5 mm 的最小余量也不能直接当作制造公差许可。

相关回归测试覆盖配置解析、双夹角度拒绝/通过、斜向挂点桥接校正、局部胶管重叠、滑动避让、手工指定冲突位置的拒绝、CATIA 导出阻断。单夹和既有胶管行为一并回归。

最终检查：Anaconda Python 执行 `test_pipe_config.py`、`test_route_clearance.py`、`test_main3_geometry.py`、`test_multi_pipeline.py`、`test_main2_guardian.py`、`test_hose_segments.py`，共 **60 项通过**；OCC Python 对修改模块执行 `py_compile` 通过；`git diff --check` 通过。代码保留在当前工作树，未提交或推送。
