# 项目清理记录（2026-10-28）

## 清理边界

本次只清理 `outputs/` 运行产物、临时验证脚本和 Python 测试缓存。`base/`、`model/`、源代码、测试、配置和 Git 历史不删除。胶管与双夹具的两份原始日志已合并到 `JIAOGUAN_HISTORY.md`。

## 保留的运行结果

- `outputs/run_20261001/`：旧单管基线。
- `outputs/run_20261012/`：双夹具改动前的四管多管基线。
- `outputs/run_20261015_pipe3_final/`：此前 `feat/pipe3` 的四管参考结果，保留以便对照用户之前确认过的输出。
- `outputs/run_20261028_jiaoguan_doubleclamp_4pipes/`：当前 `feat/JIAOGUAN` 的四管正式结果，四根管均 `completed/formal_valid=true`，包含最新路线 JSON、GUADIAN/双夹报告和两个 HTML。
- `outputs/measured_points_20261010.json`、`outputs/multi_pipe_config_20261010.json`：当前测点和运行配置。
- `outputs/double_clamp_audit_20261028.json`：双夹具问题定位的紧凑数值证据。

## 删除的产物

- `run_20261015/`：被 `run_20261015_pipe3_final/` 取代的重复/失败运行。
- `run_20261016_jiaoguan30/`、`run_20261016_jiaoguan30_coreplan/`、`run_20261016_jiaoguan30_coreplan_shifted/`、`run_20261016_jiaoguan30_finalaudit/`、`run_20261016_jiaoguan30_reuseold/`：胶管约束的中间迭代，结论已合并到 `JIAOGUAN_HISTORY.md`，当前正式结果由 2026-10-28 四管运行替代。
- `run_20261028_doubleclamp_json/`：主动中止的诊断运行。
- `run_20261028_doubleclamp_pair_validation/`：只验证 PIPE_01/02 的中间运行，已被四管联合结果取代。
- `outputs/validate_doubleclamp_pair.py`：一次性验证脚本，验证结论已写入历史记录和正式运行 JSON。
- `__pycache__/`、`.pytest_cache/`：工具缓存。

## 结果

清理后保留四个有明确用途的运行目录，避免把失败和重复 HTML 误认为当前正式结果。当前四管 HTML 位于：

- `outputs/run_20261028_jiaoguan_doubleclamp_4pipes/routes_guide.html`
- `outputs/run_20261028_jiaoguan_doubleclamp_4pipes/multi_route_engineered.html`

本次清理不改变路线算法和配置，只整理生成物并修正文档索引。
