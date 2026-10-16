# feat/JIAOGUAN 版本说明

日期：2026-10-16  
分支：`feat/JIAOGUAN`

## 目标

在连续铝管的路线工程化结果中支持局部“铝套 + 胶管 + 铝套”结构。历史配置
`PIPE_SEGMENTS=[]` 时仍生成整根铝管，便于和已有单管结果对照。

## 参数

- `metal_9` 是 9 mm 铝管；`hose_8DE3` 是对应的 15 mm 胶管，SAAA 最小弯曲半径为 R50。
- `min_length_mm` 当前取 30 或 75，`hose_length_mm` 不低于该值。
- `sleeve_length_mm` 是每一端铝套的长度，默认 30 mm（3 cm）；铝套外径默认与 15 mm 胶管相同。它是正值参数，不把“约 3 cm”误写成不可调整的工艺下限。
- 没有 `position_s_mm`/`position_ratio` 时，按整条中心线中部放置总跨度。
- 胶管和铝套沿已有规划/工程化路线截取，不根据胶管首尾重新拟合一段圆弧。
- R50 只做 warning；即使实际路线低于 R50，也继续生成并在 `hose_layout` 中记录提示。
- 当 `hose_length_mm + 2 * sleeve_length_mm` 超过整条路线长度时，局部结构回退为整根铝管，`hose_layout.fallback` 为 `all_aluminum`。

总占用弧长为 `hose_length_mm + 2 * sleeve_length_mm`。布局报告写入
`hose_layout`，局部网格写入 `component_meshes`，供工程化 HTML 使用；截切后的
line/arc 结构写入 `component_segments`，供 `main4.py` 分别扫掠成 CATIA 实体。

## 颜色与建模

工程化 HTML 使用浅铝色、红褐色胶管和土黄色铝套。CATIA 导出为每个局部区间
创建独立的 HybridBody、Join 和 circular sweep，并尝试写入对应显示颜色；不会
再用整根铝管覆盖胶管区间。

## 验证边界

已验证纯函数长度/位置/R50 规则、分段 line/arc 截切、配置解析、旧路线几何回归、
Python 编译和 `git diff --check`。本机没有执行真实 CATIA COM 建模，因此 CATPart
颜色和 sweep 成功仍需在装有 CATIA/pywin32 的环境中运行 `main4.py` 复核。
