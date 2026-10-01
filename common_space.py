# -*- coding: utf-8 -*-
"""规划空间计算公共模块。

本文件把 CAD 模型转换为“可用于布管的空间描述”：先读取并网格化
障碍物，再在 X/Y 两个方向进行截面求交，最后把截面中的自由区域保存为
JSON，并生成一个 Plotly HTML 预览。

坐标约定：所有 CAD 长度默认使用毫米；X 截面固定 X、在 Y/Z 平面工作，
Y 截面固定 Y、在 X/Z 平面工作。``hard`` 边界会被扣除，``soft`` 边界
只作为代价/显示信息保留；``none``（例如半透明观察对象）完全不参与约束。

阅读代码时可以抓住三条主线：
1. ``component_*`` 函数负责解释 settings 中的部件规则；
2. ``compute_axis_section`` 负责“一张截面”上的硬障碍扣除；
3. ``compute_planning_space`` 负责加载 CAD、并行计算所有截面和保存结果。
"""
from __future__ import annotations

import json
import math
import time
# ProcessPoolExecutor 用于并行计算互不依赖的 X/Y 截面。
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

# occ_mesh 负责 CAD 文件、网格和部件规则；occ_section 负责网格截面求交。
from occ_mesh import component_rule, infer_ground_z, load_components, read_cad_shape, scan_step_files, triangulate_shape_to_mesh
from occ_section import section_mesh_to_xz_lines, section_mesh_to_yz_lines


def save_json(data, path: Path) -> None:
    """把 Python 数据写成 UTF-8、带缩进的 JSON 文件。

    ``path.parent.mkdir`` 让调用方不必提前创建输出目录；
    ``ensure_ascii=False`` 保留中文文件内容，便于直接阅读。
    """
    # 统一转换为 Path，兼容字符串路径和 Path 对象。
    path = Path(path)
    # 输出目录不存在时递归创建；目录已经存在则不报错。
    path.parent.mkdir(parents=True, exist_ok=True)
    # indent=2 便于人工检查规划空间，encoding 保证中文不乱码。
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def load_json(path: Path):
    """读取 UTF-8 JSON 文件并还原为 Python 对象。"""
    # 先读取文本，再交给 json.loads 解析为 dict/list 等对象。
    return json.loads(Path(path).read_text(encoding="utf-8"))


def point_list(point) -> list[float]:
    """把点坐标中的数值统一转换为普通 Python ``float`` 列表。"""
    # 这样可以避免 numpy 数值或其他数值类型无法直接序列化的问题。
    return [float(v) for v in point]


def normalize_rule_key(value: str) -> str:
    """规范化规则键：转字符串、转小写并去掉首尾空白。"""
    return str(value).lower().strip()


def component_rule_dict(step_path: Path, settings) -> dict:
    """按文件名或不带扩展名的 stem 查找部件特殊规则。

    规则优先匹配完整文件名，例如 ``box.step``；找不到时再匹配
    ``box``。所有键先经过统一的小写/去空格处理，避免配置大小写不一致。
    """
    # 完整文件名和 stem 都做一次规范化，分别支持两种配置写法。
    key_file = normalize_rule_key(step_path.name)
    key_stem = normalize_rule_key(step_path.stem)
    # settings 可能没有 SPECIAL_COMPONENT_RULES，因此使用空字典兜底。
    rules = {
        normalize_rule_key(key): value
        for key, value in getattr(settings, "SPECIAL_COMPONENT_RULES", {}).items()
    }
    # ``or {}`` 表示没有命中规则时返回空字典。
    return rules.get(key_file) or rules.get(key_stem) or {}


def component_boundary_mode(step_path: Path, role: str, settings) -> str:
    """确定部件边界类型：``hard``、``soft`` 或 ``none``。

    显式配置优先；没有配置时，障碍物默认是硬边界，其他角色默认不参与
    自由空间裁剪。硬边界会扣除自由区域，软边界只记录给后续规划器使用。
    """
    # 读取当前 CAD 文件对应的特殊规则。
    rule = component_rule_dict(step_path, settings)
    if "boundary" in rule:
        # 配置中的值优先于角色默认值。
        boundary = normalize_rule_key(rule["boundary"])
    elif role == "obstacle":
        # 没有特殊配置的 obstacle 必须避让，因此默认采用硬边界。
        boundary = "hard"
    else:
        # ground、point 等非障碍角色不自动裁剪自由空间。
        boundary = "none"
    # 尽早检查拼写错误，避免错误值静默影响路径规划。
    if boundary not in {"hard", "soft", "none"}:
        raise ValueError(f"Invalid boundary mode for {step_path.name}: {boundary}")
    return boundary


def component_soft_penalty(step_path: Path, settings) -> float:
    """读取软边界的代价系数，缺省时使用全局默认值。"""
    # 软障碍暂不切掉空间，但后续代价函数可用该系数惩罚靠近它的路线。
    rule = component_rule_dict(step_path, settings)
    return float(rule.get("soft_penalty", getattr(settings, "DEFAULT_SOFT_PENALTY", 1.0)))


def import_shapely():
    """延迟导入 Shapely，并把依赖缺失转换为易懂的安装提示。

    Shapely 只在真正需要几何运算时加载，避免仅导入本模块时就强制依赖。
    """
    try:
        # 几何对象：折线、多边形、多多边形和矩形；几何操作：面生成、合并、三角化。
        from shapely.geometry import LineString, MultiPolygon, Polygon, box
        from shapely.ops import polygonize, unary_union, triangulate
        return LineString, MultiPolygon, Polygon, box, polygonize, unary_union, triangulate
    except Exception as exc:
        # 保留原始异常作为 cause，便于调试环境问题。
        raise ImportError("缺少 shapely，请在当前 Python 环境安装：python -m pip install shapely") from exc


# 延迟导入的全局占位符；ensure_shapely 首次使用时才填充它们。
LineString = MultiPolygon = Polygon = box = polygonize = unary_union = triangulate = None


def ensure_shapely() -> None:
    """确保 Shapely 符号已经加载；已加载时不重复导入。"""
    global LineString, MultiPolygon, Polygon, box, polygonize, unary_union, triangulate
    # Polygon 是导入完成的标志，避免每次几何运算都执行 import。
    if Polygon is None:
        LineString, MultiPolygon, Polygon, box, polygonize, unary_union, triangulate = import_shapely()


def get_x_range(settings) -> list[float]:
    """计算规划空间的 X 范围。

    若 settings 指定了固定 ``X_RANGE``，直接使用它；否则以 START/GOAL
    的 X 坐标为中心，并在两端增加配置的缓冲距离。
    """
    if settings.X_RANGE is not None:
        # 显式范围具有最高优先级，并统一转换为 float。
        return [float(settings.X_RANGE[0]), float(settings.X_RANGE[1])]
    # 未指定固定范围时，从起点和终点的 X 坐标自动推导。
    sx, gx = float(settings.START[0]), float(settings.GOAL[0])
    return [
        # 起点前的额外空间，允许路线在起点前调整方向。
        min(sx, gx) - float(settings.X_PADDING_BEFORE_START),
        # 终点后的额外空间，允许路线在终点后完成姿态过渡。
        max(sx, gx) + float(settings.X_PADDING_AFTER_GOAL),
    ]


def get_yz_range(settings, ground_z: float) -> tuple[list[float], list[float]]:
    """根据地面高度和 settings 计算截面上的 Y/Z 搜索范围。"""
    # Y 方向以 SECTION_CENTER_Y 为中心，向两侧展开一半宽度。
    half = float(settings.SECTION_WIDTH_Y) / 2.0
    y_range = [float(settings.SECTION_CENTER_Y) - half, float(settings.SECTION_CENTER_Y) + half]
    # Z 下限同时考虑地面净空和管道半径，避免管道实体穿过地面。
    z_range = [
        float(ground_z) + float(settings.DEFAULT_GROUND_CLEARANCE) + float(settings.PIPE_RADIUS),
        # Z 上限由允许的离地高度控制。
        float(ground_z) + float(settings.SECTION_HEIGHT_ABOVE_GROUND),
    ]
    return y_range, z_range


def frange(vmin: float, vmax: float, step: float) -> list[float]:
    """生成包含起点的等步长浮点采样序列。

    Python 内置 ``range`` 不支持浮点数，因此先计算采样点数量，再用
    ``vmin + i * step`` 生成坐标；至少返回一个点。
    """
    # floor 确保最后一个点不会超过 vmax，再加 1 保留起点。
    count = int(math.floor((float(vmax) - float(vmin)) / float(step))) + 1
    # max(count, 1) 防止范围反向或步长异常时返回空列表。
    return [float(vmin) + i * float(step) for i in range(max(count, 1))]


def polygon_to_record(poly) -> dict:
    """把 Shapely 多边形转换成 JSON 可序列化的记录。"""
    # buffer(0) 是常用的几何修复操作，可清理自交等轻微拓扑问题。
    poly = poly.buffer(0)
    # 零面积对象没有可用的自由区域，返回空记录。
    if float(poly.area) <= 0:
        return {}
    # exterior 是外轮廓，坐标按 [Y, Z] 保存。
    exterior = [[float(y), float(z)] for y, z in list(poly.exterior.coords)]
    # interiors 是孔洞；至少四个坐标点才可能形成闭合环。
    holes = [
        [[float(y), float(z)] for y, z in list(ring.coords)]
        for ring in poly.interiors
        if len(ring.coords) >= 4
    ]
    return {
        # 轮廓和孔洞供前端重建几何。
        "polygon": exterior,
        "holes": holes,
        # 面积用于过滤和统计，质心用于后续可视化/调试。
        "area": float(poly.area),
        "centroid": [float(poly.centroid.x), float(poly.centroid.y)],
    }


def explode_polygons(geom, min_area: float) -> list:
    """将任意 Shapely 几何拆成多边形列表并过滤小面积碎片。"""
    ensure_shapely()
    # 空几何无需继续处理。
    if geom.is_empty:
        return []
    # Polygon 只有一个部件，MultiPolygon 有多个部件，其他集合取其中的 Polygon。
    if isinstance(geom, Polygon):
        parts = [geom]
    elif isinstance(geom, MultiPolygon):
        parts = list(geom.geoms)
    else:
        parts = [g for g in getattr(geom, "geoms", []) if isinstance(g, Polygon)]
    # 过滤空对象和噪声小面，并再次修复每个多边形的拓扑。
    return [p.buffer(0) for p in parts if not p.is_empty and float(p.area) >= float(min_area)]



def line_to_linestring(line):
    """把 ``[[y, z], ...]`` 坐标转成有效的 Shapely LineString。

    数据不足、长度近似为零或坐标格式错误时返回 ``None``，调用方可以跳过
    该条异常截线而继续处理其他部件。
    """
    ensure_shapely()
    try:
        # 两个点才能构成折线。
        if len(line) < 2:
            return None
        # 强制转 float，确保后续 buffer/union 不受 numpy 类型影响。
        geom = LineString([(float(y), float(z)) for y, z in line])
        # 极短线段会制造不稳定的几何结果，因此视为无效。
        if geom.is_empty or geom.length <= 1e-6:
            return None
        return geom
    except Exception:
        # 单条坏数据不应中断整个规划空间计算。
        return None


def closed_line_polygons(lines, close_tol: float, min_area: float):
    """从首尾接近闭合的截线中直接构造填充多边形。"""
    ensure_shapely()
    polys = []
    for line in lines:
        # 少于 4 个点通常不足以形成可靠的闭合轮廓。
        if len(line) < 4:
            continue
        # 用首尾距离判断轮廓是否闭合，而不是要求浮点坐标完全相等。
        dy = float(line[0][0]) - float(line[-1][0])
        dz = float(line[0][1]) - float(line[-1][1])
        if math.hypot(dy, dz) > close_tol:
            continue
        # 复制为浮点坐标，并在必要时显式补回首点。
        coords = [(float(y), float(z)) for y, z in line]
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        try:
            # buffer(0) 修复轮廓，再按最小面积过滤噪声。
            poly = Polygon(coords).buffer(0)
            if not poly.is_empty and float(poly.area) >= float(min_area):
                polys.append(poly)
        except Exception:
            # 当前轮廓无效时跳过，不影响其他轮廓。
            continue
    return polys


def polygonize_from_lines(lines, min_area: float):
    """将多条相交截线合并后 polygonize，提取其中的封闭区域。"""
    ensure_shapely()
    line_geoms = []
    for line in lines:
        # 先过滤无效折线，避免 unary_union 接收到空对象。
        geom = line_to_linestring(line)
        if geom is not None:
            line_geoms.append(geom)
    if not line_geoms:
        return []
    try:
        # 合并相交/相邻线后再寻找闭环。
        merged = unary_union(line_geoms)
        polys = []
        for poly in polygonize(merged):
            # 每个闭环都修复并过滤面积过小的碎片。
            poly = poly.buffer(0)
            if not poly.is_empty and float(poly.area) >= float(min_area):
                polys.append(poly)
        return polys
    except Exception:
        # 几何拓扑异常时返回空列表，由上层使用线缓冲结果兜底。
        return []


def solid_fill_area_for_component(comp: dict, settings_values: dict):
    """为实体障碍物推断需要填充的截面面积。

    优先使用闭合轮廓和 polygonize；当 CAD 截线没有完全闭合时，可选地对
    截线做 buffer，再把缓冲区域作为实体填充。返回填充几何和调试统计。
    """
    ensure_shapely()
    # 组件没有截线时无法推断实体面积。
    lines = comp.get("lines", [])
    if not lines:
        return None, {"solid_fill_area": 0.0, "solid_loop_count": 0, "solid_mode": "none"}

    # 读取闭合判定、最小面积和管道净空对应的缓冲半径。
    close_tol = float(settings_values["SOLID_LOOP_CLOSE_TOLERANCE"])
    min_area = float(settings_values["SOLID_MIN_FILL_AREA"])
    buffer_radius = float(comp["buffer_radius"])

    # 两种闭环算法互相补充：显式闭环 + 多线 polygonize。
    fill_polys = []
    fill_polys.extend(closed_line_polygons(lines, close_tol, min_area))
    fill_polys.extend(polygonize_from_lines(lines, min_area))
    solid_mode = "closed_loop"

    # 截线不闭合时，用缓冲区封闭轮廓作为保守的实体近似。
    if settings_values.get("SOLID_FILL_FROM_BUFFER", True):
        line_geoms = [line_to_linestring(line) for line in lines]
        line_geoms = [g for g in line_geoms if g is not None]
        if line_geoms:
            try:
                # cap_style/join_style=1 表示圆形端帽/圆角连接，避免尖角低估占用。
                buffered = unary_union([g.buffer(buffer_radius, cap_style=1, join_style=1) for g in line_geoms]).buffer(0)
                for poly in explode_polygons(buffered, min_area):
                    fill_polys.append(poly)
                solid_mode = "closed_or_buffer_sealed"
            except Exception:
                pass

    # 没有任何可填充闭环时，返回 None 并保留统一的调试字段。
    if not fill_polys:
        return None, {"solid_fill_area": 0.0, "solid_loop_count": 0, "solid_mode": "none"}

    # 合并重叠填充面，得到组件最终实体占用区域。
    area = unary_union(fill_polys).buffer(0)
    return area, {
        "solid_fill_area": float(area.area) if area and not area.is_empty else 0.0,
        "solid_loop_count": len(fill_polys),
        "solid_mode": solid_mode,
    }


def obstacle_lines_to_area(obstacle_section_lines: list[dict], settings_values: dict) -> tuple[object | None, list[dict]]:
    """把障碍物截线转换为需要从自由空间扣除的面积。

    每条截线先按 ``PIPE_RADIUS + clearance`` 做缓冲，代表管道中心线必须
    保持的安全距离；实体部件还会额外合并推断出的填充区域。返回合并后的
    Shapely 几何和每个组件的面积调试信息。
    """
    ensure_shapely()
    # blockers 保存所有需要扣除的几何，debug 保存可写入 JSON 的统计信息。
    blockers = []
    debug = []

    for comp in obstacle_section_lines:
        # buffer_radius 已在截面构造阶段算好，包含管道半径和部件净空。
        radius = float(comp["buffer_radius"])
        line_geoms = []
        for line in comp.get("lines", []):
            # 一条非法截线只被忽略，不影响同一组件的其他截线。
            geom = line_to_linestring(line)
            if geom is not None:
                line_geoms.append(geom)

        # 组件没有有效截线时没有可扣除面积。
        if not line_geoms:
            continue

        buffered_lines = None
        try:
            # 对每条截线做圆形 buffer，再 union 成该组件的管道避让区。
            buffered_lines = unary_union([g.buffer(radius, cap_style=1, join_style=1) for g in line_geoms]).buffer(0)
            if not buffered_lines.is_empty:
                blockers.append(buffered_lines)
        except Exception:
            # 拓扑异常时保留实体填充尝试和调试信息。
            pass

        # 非实体部件只需要线缓冲；实体部件还要扣除本体填充区域。
        solid_info = {"solid_fill_area": 0.0, "solid_loop_count": 0, "solid_mode": "not_solid"}
        if comp.get("solid", True):
            solid_area, solid_info = solid_fill_area_for_component(comp, settings_values)
            if solid_area is not None and not solid_area.is_empty:
                blockers.append(solid_area)

        # 记录面积和线数量，便于检查净空是否被正确应用。
        debug.append({
            "name": comp.get("name", ""),
            "solid": bool(comp.get("solid", True)),
            "line_count": len(line_geoms),
            "buffer_radius": radius,
            "buffer_area": float(buffered_lines.area) if buffered_lines is not None and not buffered_lines.is_empty else 0.0,
            **solid_info,
        })

    # 没有任何障碍几何时返回 None，让上层直接使用完整矩形。
    if not blockers:
        return None, debug
    # 多个障碍物之间可能重叠，union 后再修复拓扑得到一个整体扣除区。
    return unary_union(blockers).buffer(0), debug



def compute_axis_section(
    axis: str,
    value: float,
    obstacle_components: list[dict],
    lateral_range: list[float],
    z_range: list[float],
    settings_values: dict,
) -> dict:
    """计算垂直于 X 或 Y 轴的一张自由空间截面。

    ``axis == "x"`` 时固定 X，截线坐标是 Y/Z；``axis == "y"`` 时固定 Y，
    截线坐标是 X/Z。两种截面共享同样的 buffer、实体填充和自由区域逻辑。
    硬边界会从矩形搜索域中扣除，软边界仅保留其截线记录。
    """
    # 子进程第一次执行时也必须确保 Shapely 已导入。
    ensure_shapely()
    # 统一轴名称并拒绝拼写错误，避免产生难以解释的空结果。
    axis = str(axis).lower()
    if axis not in {"x", "y"}:
        raise ValueError(f"Unsupported section axis: {axis}")
    # 两类边界分开保存：后面只用 hard 计算 free_regions。
    hard_obstacle_section_lines = []
    soft_obstacle_section_lines = []

    for comp in obstacle_components:
        try:
            # mesh 是主进程预先生成并传入的三角网格，避免子进程重复读取 CAD。
            mesh = comp.get("mesh")
            if mesh is None:
                print(f"[WARN] section skip {axis}={float(value):.1f}, {comp['name']}: mesh is None")
                continue

            # 根据固定轴选择对应的二维截面函数。
            if axis == "x":
                lines = section_mesh_to_yz_lines(mesh, float(value))
            else:
                lines = section_mesh_to_xz_lines(mesh, float(value))
            # 当前截面没有穿过组件时，跳过该组件。
            if not lines:
                continue

            # 统一截面记录格式，供扣除、checkpoint 和 HTML 复用。
            record = {
                "name": comp["name"],
                "file": comp["file"],
                "clearance": float(comp["clearance"]),
                "buffer_radius": float(settings_values["PIPE_RADIUS"]) + float(comp["clearance"]),
                "boundary": comp.get("boundary", "hard"),
                "solid": bool(comp.get("solid", True)),
                "soft_penalty": float(comp.get("soft_penalty", 1.0)),
                "lines": lines,
            }

            # soft 只记录；hard 会参与自由空间差集；none 理论上不会出现在 obstacles 中。
            if record["boundary"] == "soft":
                soft_obstacle_section_lines.append(record)
            elif record["boundary"] == "hard":
                hard_obstacle_section_lines.append(record)

        except Exception as exc:
            # 单个组件/截面的失败只报警并继续，便于得到尽可能完整的 checkpoint。
            print(f"[WARN] section failed {axis}={float(value):.1f}, {comp['name']}: {type(exc).__name__}: {exc}")

    # 用 lateral_range × z_range 建立没有障碍物时的完整搜索矩形。
    rect = box(float(lateral_range[0]), float(z_range[0]), float(lateral_range[1]), float(z_range[1]))

    # 核心：只扣除 hard 边界；soft 边界不裁剪 free_regions，只记录截面线。
    hard_obstacle_area, solid_debug = obstacle_lines_to_area(hard_obstacle_section_lines, settings_values)
    # 没有硬障碍就保留整个矩形，否则计算矩形与障碍扣除区的差集。
    free_geom = rect if hard_obstacle_area is None else rect.difference(hard_obstacle_area).buffer(0)

    # 把差集拆成独立多边形，并过滤面积过小的碎片。
    free_regions = []
    for poly in explode_polygons(free_geom, settings_values["MIN_FREE_REGION_AREA"]):
        # 简化轮廓可以缩小 JSON，但 preserve_topology 防止孔洞/自交被破坏。
        if settings_values["POLYGON_SIMPLIFY_TOLERANCE"] > 0:
            poly = poly.simplify(settings_values["POLYGON_SIMPLIFY_TOLERANCE"], preserve_topology=True)
        # 转成纯 Python 字典，确保可以跨进程和写入 JSON。
        rec = polygon_to_record(poly)
        if rec:
            free_regions.append(rec)

    # 组装一个截面的统一结果；同时保留旧字段以兼容旧版 main2/main3。
    result = {
        "axis": axis,
        "value": float(value),
        "free_regions": free_regions,

        # 兼容旧接口：obstacle_section_lines 仍代表硬障碍线
        "obstacle_section_lines": hard_obstacle_section_lines,

        # 新接口：软硬边界分开存储
        "hard_obstacle_section_lines": hard_obstacle_section_lines,
        "soft_obstacle_section_lines": soft_obstacle_section_lines,
        "all_obstacle_section_lines": hard_obstacle_section_lines + soft_obstacle_section_lines,
        "solid_debug": solid_debug,
    }
    # 旧调用方可能直接读取 sec["x"] 或 sec["y"]，因此动态补上轴坐标。
    result[axis] = float(value)
    return result


def compute_section(x: float, obstacle_components: list[dict], y_range: list[float], z_range: list[float], settings_values: dict) -> dict:
    """兼容旧接口的 X 截面包装器。"""
    # 旧代码只传 X 坐标；统一转发给支持双轴的新实现。
    return compute_axis_section("x", x, obstacle_components, y_range, z_range, settings_values)




def serialize_space(
    sections: list[dict],
    components: list,
    ignored: list,
    settings,
    ranges: dict,
    axes: dict,
    ground_z: float,
    is_checkpoint: bool,
    y_sections: list[dict] | None = None,
) -> dict:
    """把计算结果、配置摘要和部件清单组装成统一 JSON 结构。

    ``sections`` 是 X 截面；``y_sections`` 是新增的 Y 截面。输出同时保留
    ``sections`` 和 ``x_sections`` 两个键，让旧版消费者继续工作。
    """
    # None 表示当前尚未完成 Y 截面，统一成空列表方便后面的统计和序列化。
    y_sections = list(y_sections or [])
    # 统计所有 X 截面的自由面积，用于输出整体空间利用率指标。
    total_area = sum(float(r.get("area", 0.0)) for sec in sections for r in sec.get("free_regions", []))
    # 单个截面的设计面积是 Y 宽度乘 Z 高度。
    design_area = (ranges["y"][1] - ranges["y"][0]) * (ranges["z"][1] - ranges["z"][0])
    # X 轴采样总数用于 meta 进度字段；Y 轴数量单独记录。
    total_sections = len(axes["xs"])

    component_records = []
    hard_count = 0
    soft_count = 0

    # 把对象形式的组件转换成稳定、可 JSON 序列化的记录。
    for c in components:
        boundary = component_boundary_mode(Path(c.file), c.role, settings)
        if boundary == "hard":
            hard_count += 1
        elif boundary == "soft":
            soft_count += 1

        component_records.append({
            "name": c.name,
            "file": c.file,
            "role": c.role,
            "boundary": boundary,
            "clearance": float(c.clearance),
            "solid": bool(c.solid),
            "mesh_triangles": int(c.mesh.get("triangle_count", 0)) if c.mesh else 0,
        })

    # 返回的顶层字段按“元数据、范围、组件、截面、起终点”分组，便于其他阶段读取。
    return {
        # meta 记录算法版本、进度、几何参数和自由面积统计。
        "meta": {
            "method": "dual_axis_mesh_section_soft_hard_boundary_space",
            "is_checkpoint": bool(is_checkpoint),
            "completed_sections": len(sections),
            "total_sections": total_sections,
            "completed_x_sections": len(sections),
            "completed_y_sections": len(y_sections),
            "total_x_sections": len(axes.get("xs", [])),
            "total_y_sections": len(axes.get("ys", [])),
            "base_step_dir": str(settings.BASE_STEP_DIR),
            # 旧字段继续保留；它现在表示本次测试路线的最大外部包络半径。
            "pipe_radius": float(settings.PIPE_RADIUS),
            "route_max_envelope_radius": float(getattr(settings, "ROUTE_MAX_ENVELOPE_RADIUS", settings.PIPE_RADIUS)),
            "active_pipe_spec": str(getattr(settings, "ACTIVE_PIPE_SPEC", "")),
            "pipe_spec_ids": sorted(getattr(settings, "selected_pipe_spec_ids", lambda: set())()),
            "default_obstacle_clearance": float(settings.DEFAULT_OBSTACLE_CLEARANCE),
            "default_ground_clearance": float(settings.DEFAULT_GROUND_CLEARANCE),
            "ground_z": float(ground_z),
            "section_width_y": float(settings.SECTION_WIDTH_Y),
            "section_height_above_ground": float(settings.SECTION_HEIGHT_ABOVE_GROUND),
            "section_dx": float(settings.SECTION_DX),
            "section_dy": float(getattr(settings, "SECTION_DY", settings.SECTION_DX)),
            "hard_obstacle_count": int(hard_count),
            "soft_obstacle_count": int(soft_count),
            # 分母至少为 1，避免没有截面或异常范围时出现除零。
            "free_area_ratio": float(total_area / max(design_area * max(len(sections), 1), 1.0)),
        },
        # ranges 是连续搜索范围；axes 是真正参与计算的离散采样坐标。
        "ranges": ranges,
        "axes": axes,
        # all 保存有效组件，ignored 保存被规则明确忽略的文件。
        "components": {
            "all": component_records,
            "ignored": [
                {"name": c.name, "file": c.file, "role": c.role, "boundary": "none", "clearance": float(c.clearance), "solid": bool(c.solid)}
                for c in ignored
            ],
        },
        # ``sections`` remains an alias for X sections so older consumers keep
        # working.  New planners should read both explicit families.
        "sections": sections,
        "x_sections": sections,
        "y_sections": y_sections,
        # 起点和终点统一转为普通浮点列表，供 main2/main3 和 HTML 使用。
        "start": point_list(settings.START),
        "goal": point_list(settings.GOAL),
    }



def checkpoint(sections, components, ignored, settings, ranges, axes, ground_z, final: bool = False, y_sections=None) -> None:
    """按需写入中间或最终 checkpoint 文件。

    中间 checkpoint 受 ``ENABLE_SPACE_CHECKPOINT`` 控制；最终结果即使关闭
    该开关也会写出，保证主流程结束后始终有一份可恢复的记录。
    """
    # 非最终阶段且用户关闭 checkpoint 时直接返回，减少频繁磁盘写入。
    if not settings.ENABLE_SPACE_CHECKPOINT and not final:
        return
    # final=False 时 serialize_space 中的 is_checkpoint 应为 True。
    data = serialize_space(sections, components, ignored, settings, ranges, axes, ground_z, not final, y_sections=y_sections)
    # 中间和最终 checkpoint 使用同一个配置路径，后写入的结果覆盖前一个版本。
    save_json(data, settings.SPACE_CHECKPOINT_JSON)


def progress(label: str, current: int, total: int) -> None:
    """在同一行打印一个受限到 0~100% 的进度条。"""
    # total 至少取 1，避免除零；current 超界时由 ratio 截断。
    total = max(int(total), 1)
    ratio = min(max(current / total, 0.0), 1.0)
    # \r 覆盖当前行，flush=True 让长时间计算时立即显示。
    print(f"\r{label} {ratio * 100:6.2f}% {current}/{total}", end="", flush=True)



def compute_planning_space(settings) -> dict:
    """计算完整的双轴规划空间并保存 JSON。

    主流程可以按以下顺序理解：
    1. 读取 CAD、网格和 ground 高度；
    2. 根据 START/GOAL 和配置生成 X/Y 采样轴；
    3. 把可序列化的障碍物网格提交给并行 worker；
    4. 收集截面、周期性写 checkpoint；
    5. 按采样轴排序并写最终规划空间。
    """
    # 记录总耗时，最后用于诊断大模型或高采样密度下的性能。
    t0 = time.time()
    print("\n=== Main1 Debug: Soft/Hard Planning Space ===")
    print("[LOAD] Start loading CAD components...")
    # 一次性加载组件、原始 shape 和三角网格；ignored 单独保留用于审计。
    components, ignored = load_components(settings, with_shape=True, with_mesh=True)
    # 只有 obstacle 会裁剪自由空间；ground 只用于推断 Z 基准。
    obstacles = [c for c in components if c.role == "obstacle"]
    grounds = [c for c in components if c.role == "ground"]
    if not obstacles:
        raise RuntimeError("No obstacle CAD components found.")

    # 每个 ground 网格都推断一个高度，多块地面时取平均值。
    ground_values = [infer_ground_z(g.mesh) for g in grounds]
    # 没有 ground 时使用保守默认值，保持流程可运行并在日志中可见。
    ground_z = float(sum(ground_values) / len(ground_values)) if ground_values else -300.0
    # 计算三维搜索域和 X/Y 采样坐标。
    x_range = get_x_range(settings)
    y_range, z_range = get_yz_range(settings, ground_z)
    xs = frange(x_range[0], x_range[1], settings.SECTION_DX)
    section_dy = float(getattr(settings, "SECTION_DY", settings.SECTION_DX))
    ys = frange(y_range[0], y_range[1], section_dy)

    # ranges 是 JSON 对外描述，axes 是实际采样点列表。
    ranges = {"x": [float(x_range[0]), float(x_range[1])], "y": y_range, "z": z_range}
    axes = {
        "xs": [float(x) for x in (xs.tolist() if hasattr(xs, "tolist") else xs)],
        "ys": [float(y) for y in (ys.tolist() if hasattr(ys, "tolist") else ys)],
    }

    # 将组件对象压缩为 worker 需要的普通字典，避免把 settings 或 CAD shape 传入子进程。
    obstacle_components = []
    hard_count = 0
    soft_count = 0

    for c in obstacles:
        # 统计硬/软数量，并把规则解析结果固定下来，确保各 worker 行为一致。
        boundary = component_boundary_mode(Path(c.file), c.role, settings)
        if boundary == "hard":
            hard_count += 1
        elif boundary == "soft":
            soft_count += 1

        obstacle_components.append({
            "name": c.name,
            "file": c.file,
            "clearance": float(c.clearance),
            "boundary": boundary,
            "solid": bool(c.solid),
            "soft_penalty": component_soft_penalty(Path(c.file), settings),
            # 关键：把已经生成好的 mesh 传给子进程，worker 不再读取 CAD 文件。
            "mesh": c.mesh,
        })

    # 只传递可序列化的数值配置给 worker，避免 settings 模块对象跨进程问题。
    settings_values = {
        "MESH_LINEAR_DEFLECTION": float(settings.MESH_LINEAR_DEFLECTION),
        "MESH_ANGULAR_DEFLECTION": float(settings.MESH_ANGULAR_DEFLECTION),
        "PIPE_RADIUS": float(settings.PIPE_RADIUS),
        "MIN_FREE_REGION_AREA": float(settings.MIN_FREE_REGION_AREA),
        "POLYGON_SIMPLIFY_TOLERANCE": float(settings.POLYGON_SIMPLIFY_TOLERANCE),
        "SOLID_LOOP_CLOSE_TOLERANCE": float(getattr(settings, "SOLID_LOOP_CLOSE_TOLERANCE", 8.0)),
        "SOLID_MIN_FILL_AREA": float(getattr(settings, "SOLID_MIN_FILL_AREA", settings.MIN_FREE_REGION_AREA)),
        "SOLID_FILL_FROM_BUFFER": bool(getattr(settings, "SOLID_FILL_FROM_BUFFER", True)),
    }

    print(f"[SPACE] components={len(components)}, obstacles={len(obstacles)}, hard={hard_count}, soft={soft_count}, ground_z={ground_z:.3f}")
    print("[COMPONENTS]")
    for c in components:
        mesh_triangles = int(c.mesh.get("triangle_count", 0)) if c.mesh else 0
        boundary = component_boundary_mode(Path(c.file), c.role, settings)
        print(
            f"  - {Path(c.file).name}: role={c.role}, boundary={boundary}, "
            f"clearance={float(c.clearance):.1f}, solid={bool(c.solid)}, "
            f"mesh_triangles={mesh_triangles}"
        )
    if ignored:
        print("[IGNORED]")
        for c in ignored:
            print(
                f"  - {Path(c.file).name}: role={c.role}, "
                f"clearance={float(c.clearance):.1f}, solid={bool(c.solid)}"
            )

    print(
        f"[SPACE] x={ranges['x']} y={ranges['y']} z={ranges['z']} "
        f"x_sections={len(xs)} y_sections={len(ys)}"
    )
    print("[SPACE] hard boundary cuts free space; soft boundary is only recorded.")

    # worker 完成顺序是不确定的，先按坐标存入字典，最后再恢复采样顺序。
    completed_by_x: dict[float, dict] = {}
    completed_by_y: dict[float, dict] = {}
    max_workers = int(settings.SECTION_PARALLEL_WORKERS)
    print(f"[PARALLEL] workers={max_workers}")
    print(f"[PARALLEL] sections={len(xs) + len(ys)} (x={len(xs)}, y={len(ys)})")
    print("[PARALLEL] submitting section jobs...")

    # Windows 使用 spawn 启动子进程，因此 compute_axis_section 必须是模块级函数。
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = {}
        for x in xs:
            # 每个 X 采样点提交一项独立的截面任务。
            future = executor.submit(
                compute_axis_section, "x", float(x), obstacle_components, y_range, z_range, settings_values
            )
            futures[future] = ("x", float(x))
        for y in ys:
            # 每个 Y 采样点同样提交一项独立任务。
            future = executor.submit(
                compute_axis_section, "y", float(y), obstacle_components, x_range, z_range, settings_values
            )
            futures[future] = ("y", float(y))
        print(f"[PARALLEL] submitted {len(futures)} jobs, waiting for first section result...")
        completed = 0
        # 按“谁先完成谁先处理”收集结果，避免慢截面阻塞进度输出。
        for fut in as_completed(futures):
            axis, value = futures[fut]
            # fut.result() 会重新抛出 worker 异常，避免静默生成不完整结果。
            section_result = fut.result()
            if axis == "x":
                completed_by_x[value] = section_result
            else:
                completed_by_y[value] = section_result
            completed += 1
            # checkpoint 需要稳定的坐标顺序，因此按原始采样轴重新排列已完成结果。
            ordered_x = [completed_by_x[float(v)] for v in xs if float(v) in completed_by_x]
            ordered_y = [completed_by_y[float(v)] for v in ys if float(v) in completed_by_y]

            # 计算当前截面的轻量统计，只用于日志，不修改原始结果。
            sec = section_result
            free_area = sum(float(r.get("area", 0.0)) for r in sec.get("free_regions", []))
            hard_lines = sum(len(c.get("lines", [])) for c in sec.get("hard_obstacle_section_lines", []))
            soft_lines = sum(len(c.get("lines", [])) for c in sec.get("soft_obstacle_section_lines", []))

            # Dual-axis JSON is larger, so checkpoint periodically instead of
            # rewriting it after every completed worker.
            # 双轴 JSON 较大，每 5 个任务或全部完成时写一次 checkpoint。
            if completed % 5 == 0 or completed == len(futures):
                checkpoint(
                    ordered_x, components, ignored, settings, ranges, axes,
                    ground_z, final=False, y_sections=ordered_y,
                )

            print(
                f"\r[SECTION] {completed:>4}/{len(futures)} "
                f"{completed / len(futures) * 100:6.2f}% | "
                f"{axis}={value:.1f} | free_regions={len(sec.get('free_regions', []))} "
                f"area={free_area:.1f} | hard_lines={hard_lines} soft_lines={soft_lines} | "
                f"checkpoint=ok",
                end="",
                flush=True,
            )

    print()
    # 所有任务完成后按采样坐标恢复最终 X/Y 截面顺序。
    sections = [completed_by_x[float(x)] for x in xs]
    y_sections = [completed_by_y[float(y)] for y in ys]
    # 组装最终 JSON 对象，再分别写主输出和最终 checkpoint。
    result = serialize_space(
        sections, components, ignored, settings, ranges, axes, ground_z,
        is_checkpoint=False, y_sections=y_sections,
    )
    print("\n[SAVE] Writing final planning space JSON...")
    save_json(result, settings.SPACE_JSON)

    print("[SAVE] Writing final checkpoint JSON...")
    checkpoint(
        sections, components, ignored, settings, ranges, axes, ground_z,
        final=True, y_sections=y_sections,
    )

    # 计算总耗时并返回内存中的结果，调用方可继续生成 HTML。
    elapsed = time.time() - t0
    print("[DONE] Planning space completed.")
    print(f"[DONE] SPACE_JSON = {settings.SPACE_JSON}")
    print(f"[DONE] SPACE_CHECKPOINT_JSON = {settings.SPACE_CHECKPOINT_JSON}")
    print(f"[DONE] x_sections = {len(sections)}")
    print(f"[DONE] y_sections = {len(y_sections)}")
    print(f"[DONE] elapsed = {elapsed:.1f}s")

    return result



def polygon_display_mesh(region: dict, max_points: int = 240) -> dict | None:
    """把二维自由区域三角化为 Plotly ``mesh3d`` 所需的索引结构。

    为避免浏览器渲染过重，外轮廓超过 ``max_points`` 时先抽样简化；
    ``representative_point`` 用于过滤三角化后落在多边形外部的三角形。
    """
    # 从 JSON 的外轮廓和孔洞重建 Shapely 多边形。
    ensure_shapely()
    poly = Polygon(region["polygon"], region.get("holes", []))
    # 无效轮廓先修复；完全为空时前端没有可画的内容。
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty:
        return None

    # 只简化外轮廓，孔洞仍从修复后的 poly 中保留。
    exterior = list(poly.exterior.coords)
    if len(exterior) > max_points:
        # 按固定步长抽样，且确保首尾仍然闭合。
        step = max(1, len(exterior) // max_points)
        exterior = exterior[::step]
        if exterior[0] != exterior[-1]:
            exterior.append(exterior[0])
        poly = Polygon(exterior, [list(r.coords) for r in poly.interiors])

    # Plotly 使用顶点数组 + 三角形索引数组表达 mesh3d。
    vertices: list[list[float]] = []
    index: dict[tuple[float, float], int] = {}
    ii: list[int] = []
    jj: list[int] = []
    kk: list[int] = []

    def add_vertex(y, z):
        # 以 1e-6 精度去重，避免相邻三角形重复存储同一个顶点。
        key = (round(float(y), 6), round(float(z), 6))
        if key not in index:
            index[key] = len(vertices)
            vertices.append([float(y), float(z)])
        return index[key]

    # Shapely 可能在孔洞外生成候选三角形，下面用代表点过滤。
    for tri in triangulate(poly):
        probe = tri.representative_point()
        if not poly.contains(probe) and not poly.touches(probe):
            continue
        coords = list(tri.exterior.coords)[:3]
        if len(coords) != 3:
            continue
        # 把三个角点加入共享顶点表，并记录三角形索引。
        a = add_vertex(*coords[0])
        b = add_vertex(*coords[1])
        c = add_vertex(*coords[2])
        ii.append(a)
        jj.append(b)
        kk.append(c)

    # 没有有效顶点/三角形时返回 None，调用方会跳过该区域。
    if not vertices or not ii:
        return None

    return {
        "y": [p[0] for p in vertices],
        "z": [p[1] for p in vertices],
        "i": ii,
        "j": jj,
        "k": kk,
        "outline": [[float(y), float(z)] for y, z in list(poly.exterior.coords)],
        "holes": [
            [[float(y), float(z)] for y, z in list(r.coords)]
            for r in poly.interiors
        ],
    }


def html_x_range(settings) -> tuple[float, float]:
    """计算 HTML 预览使用的 X 范围，并在起终点外留出边距。"""
    # HTML 边距只影响展示，不改变规划空间本身。
    margin = float(getattr(settings, "HTML_X_MARGIN", 1000.0))
    x0 = min(float(settings.START[0]), float(settings.GOAL[0])) - margin
    x1 = max(float(settings.START[0]), float(settings.GOAL[0])) + margin
    return x0, x1


def clip_mesh_to_x_range(mesh: dict, x_range: tuple[float, float] | None) -> dict:
    """删除 X 范围外的三角形，减少 HTML 传输量和浏览器负担。"""
    # 未指定范围时保持原网格对象，避免无意义复制。
    if x_range is None:
        return mesh

    # 解包坐标数组和三角形索引数组。
    x_min, x_max = x_range
    xs = mesh["x"]
    ys = mesh["y"]
    zs = mesh["z"]
    out_x = []
    out_y = []
    out_z = []
    out_i = []
    out_j = []
    out_k = []
    index_map: dict[int, int] = {}

    def add_vertex(old_idx: int) -> int:
        # 裁剪后重新建立连续索引，避免引用原网格中未保留的顶点。
        if old_idx not in index_map:
            index_map[old_idx] = len(out_x)
            out_x.append(xs[old_idx])
            out_y.append(ys[old_idx])
            out_z.append(zs[old_idx])
        return index_map[old_idx]

    # 只有三个顶点都位于范围内的三角形才保留。
    for a, b, c in zip(mesh["i"], mesh["j"], mesh["k"]):
        tri_x = (float(xs[a]), float(xs[b]), float(xs[c]))
        if min(tri_x) < x_min or max(tri_x) > x_max:
            continue
        out_i.append(add_vertex(a))
        out_j.append(add_vertex(b))
        out_k.append(add_vertex(c))

    # 复制其他元数据，再用裁剪后的数组覆盖几何字段。
    clipped = dict(mesh)
    clipped.update({
        "x": out_x,
        "y": out_y,
        "z": out_z,
        "i": out_i,
        "j": out_j,
        "k": out_k,
        "triangle_count": len(out_i),
    })
    return clipped


def html_aspect_ratio(
    x_range: tuple[float, float],
    meshes: list[dict],
    points: list[list[float]] | None = None,
    sections: list[dict] | None = None,
) -> dict:
    """根据模型、点和截面数据计算 Plotly 三维显示比例。"""
    # 收集所有可见对象的 Y/Z 范围，X 范围由调用方直接给出。
    ys = []
    zs = []
    for mesh in meshes:
        ys.extend(float(v) for v in mesh.get("y", []))
        zs.extend(float(v) for v in mesh.get("z", []))

    # 只纳入当前 X 显示范围内的 START/GOAL 点。
    for point in points or []:
        if len(point) >= 3 and float(x_range[0]) <= float(point[0]) <= float(x_range[1]):
            ys.append(float(point[1]))
            zs.append(float(point[2]))

    # 自由区域和硬/软边界线也要纳入比例计算，避免显示被截断。
    for sec in sections or []:
        for reg in sec.get("free_meshes", []):
            ys.extend(float(v) for v in reg.get("y", []))
            zs.extend(float(v) for v in reg.get("z", []))
        for comp_key in ("hard_lines", "soft_lines"):
            for comp in sec.get(comp_key, []):
                for line in comp.get("lines", []):
                    for y, z in line:
                        ys.append(float(y))
                        zs.append(float(z))

    # 各方向至少保留 1 个单位，防止单点数据生成零比例。
    dx = max(float(x_range[1]) - float(x_range[0]), 1.0)
    dy = max(max(ys) - min(ys), 1.0) if ys else 1.0
    dz = max(max(zs) - min(zs), 1.0) if zs else 1.0
    scale = max(dx, dy, dz, 1.0)
    return {"x": dx / scale, "y": dy / scale, "z": dz / scale}


def load_step_meshes_for_html(settings, x_range: tuple[float, float] | None = None) -> list[dict]:
    """读取 STEP 文件并生成 HTML 使用的轻量三角网格列表。"""
    # 仅遍历基础 CAD 目录；ignore 规则的文件不参与预览。
    meshes = []
    for path in scan_step_files(settings.BASE_STEP_DIR):
        role, clearance, solid = component_rule(path, settings)
        if role == "ignore":
            continue
        try:
            # 读取 CAD shape，并使用与规划计算一致的网格精度。
            shape = read_cad_shape(path)
            mesh = triangulate_shape_to_mesh(
                shape,
                settings.MESH_LINEAR_DEFLECTION,
                settings.MESH_ANGULAR_DEFLECTION,
                None,
            )
            # 先裁剪 X 范围，再把必要字段复制到 HTML payload。
            mesh = clip_mesh_to_x_range(mesh, x_range)
            if not mesh["i"]:
                continue
            meshes.append({
                "name": path.name,
                "role": role,
                "boundary": component_boundary_mode(path, role, settings),
                "clearance": float(clearance),
                "solid": bool(solid),
                "x": mesh["x"],
                "y": mesh["y"],
                "z": mesh["z"],
                "i": mesh["i"],
                "j": mesh["j"],
                "k": mesh["k"],
            })
        except Exception as exc:
            # 某个文件预览失败时只记录警告，其他组件仍可显示。
            print(f"[WARN] HTML mesh failed {path.name}: {type(exc).__name__}: {exc}")
    return meshes


def export_planning_space_html(space: dict, settings, output: Path) -> None:
    """生成可交互的 Plotly 三维规划空间 HTML 文件。

    页面同时显示 CAD 三角网格、绿色自由截面、红色硬边界、蓝色软边界以及
    START/GOAL。截面可以按 stride 抽样，避免一次性把过多三角形交给浏览器。
    """
    # stride 越大，HTML 越小；最后一个截面始终补回，保持终点附近可见。
    print("[HTML] building planning space preview...")
    stride = max(1, int(getattr(settings, "SPACE_HTML_SECTION_STRIDE", 4)))
    x_min, x_max = html_x_range(settings)
    # 兼容旧 JSON：优先读取 sections（X 截面别名）。
    sections = space.get("sections", [])
    sections = [sec for sec in sections if x_min - 1e-6 <= float(sec["x"]) <= x_max + 1e-6]
    sampled_sections = sections[::stride]
    if sections and sampled_sections and sampled_sections[-1] is not sections[-1]:
        sampled_sections.append(sections[-1])

    # 将自由区域转成前端 mesh，并保留红/蓝边界线。
    section_records = []
    for sec in sampled_sections:
        free_meshes = []
        for region in sec.get("free_regions", []):
            mesh = polygon_display_mesh(region)
            if mesh is not None:
                free_meshes.append(mesh)
        section_records.append({
            "x": float(sec["x"]),
            "free_meshes": free_meshes,
            "hard_lines": sec.get("hard_obstacle_section_lines", sec.get("obstacle_section_lines", [])),
            "soft_lines": sec.get("soft_obstacle_section_lines", []),
        })

    # 读取并裁剪 CAD 网格，和截面记录一起打包成浏览器 payload。
    meshes = load_step_meshes_for_html(settings, (x_min, x_max))
    # payload 是前端唯一需要的输入，避免把 Python 对象或无关 settings 暴露给浏览器。
    payload = {
        # CAD 三角网格用于显示真实部件外形。
        "meshes": meshes,
        # 抽样后的 X 截面用于显示自由区域和边界线。
        "sections": section_records,
        # start/goal 用于在三维场景中标记路线端点。
        "start": space.get("start", settings.START),
        "goal": space.get("goal", settings.GOAL),
        # x_range 限制相机可见的 X 范围。
        "x_range": [x_min, x_max],
        # aspect_ratio 防止细长 CAD 在浏览器中被压扁。
        "aspect_ratio": html_aspect_ratio(
            (x_min, x_max),
            meshes,
            [space.get("start", settings.START), space.get("goal", settings.GOAL)],
            section_records,
        ),
        # meta 可在调试时通过浏览器控制台查看算法参数。
        "meta": space.get("meta", {}),
    }
    # 直接把数据内嵌到 HTML，打开文件时不需要额外的 JSON 服务。
    data_json = json.dumps(payload, ensure_ascii=False)

    # 下面的字符串是完整的前端页面：Plotly 负责三维网格、线和交互相机。
    html = """
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>Planning Space Preview</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    html, body { margin: 0; height: 100%; font-family: Arial, sans-serif; }
    #plot { width: 100vw; height: 100vh; }
    #panel {
      position: fixed; left: 12px; top: 12px; z-index: 10;
      background: rgba(255,255,255,.86); border: 1px solid #d0d0d0;
      padding: 8px 10px; font-size: 13px; line-height: 1.45;
    }
  </style>
</head>
<body>
  <div id="panel">
    <strong>Planning Space</strong><br>
    Drag: rotate / Shift+drag: pan / Wheel: zoom<br>
    Green: free section area, red: hard boundary, blue: soft boundary
  </div>
  <div id="plot"></div>
  <script>
// 下方常量由 Python 写入内嵌的规划空间 JSON。
const DATA = __DATA__;
const traces = [];

// 组件网格：soft 边界显示蓝色，ground 显示灰色，其余 CAD 显示深灰色。
for (const mesh of DATA.meshes) {
  traces.push({
    type: "mesh3d",
    name: mesh.name,
    x: mesh.x, y: mesh.y, z: mesh.z,
    i: mesh.i, j: mesh.j, k: mesh.k,
    color: mesh.boundary === "soft" ? "#60a5fa" : mesh.role === "ground" ? "#9ca3af" : "#9b9b9b",
    opacity: mesh.boundary === "soft" ? 0.18 : 0.23,
    flatshading: true,
    hoverinfo: "name",
  });
}

// 逐个绘制采样截面中的自由区域和硬/软边界线。
for (const sec of DATA.sections) {
  // 绿色半透明三角面表示可用于布管的自由区域。
  for (const reg of sec.free_meshes) {
    traces.push({
      type: "mesh3d",
      name: "free x=" + sec.x.toFixed(1),
      x: reg.y.map(() => sec.x),
      y: reg.y,
      z: reg.z,
      i: reg.i, j: reg.j, k: reg.k,
      color: "#22c55e",
      opacity: 0.16,
      hoverinfo: "skip",
      showscale: false,
    });
  }
  // 红线表示真正裁剪过自由空间的硬边界。
  for (const comp of sec.hard_lines) {
    for (const line of comp.lines || []) {
      traces.push({
        type: "scatter3d",
        mode: "lines",
        name: "hard " + comp.name,
        x: line.map(() => sec.x),
        y: line.map(p => p[0]),
        z: line.map(p => p[1]),
        line: { color: "#dc2626", width: 2 },
        hoverinfo: "name",
        showlegend: false,
      });
    }
  }
  // 蓝线表示只记录、暂未裁剪自由空间的软边界。
  for (const comp of sec.soft_lines) {
    for (const line of comp.lines || []) {
      traces.push({
        type: "scatter3d",
        mode: "lines",
        name: "soft " + comp.name,
        x: line.map(() => sec.x),
        y: line.map(p => p[0]),
        z: line.map(p => p[1]),
        line: { color: "#2563eb", width: 2 },
        hoverinfo: "name",
        showlegend: false,
      });
    }
  }
}

// 最后增加起点和终点标记，绿色为 START，红色为 GOAL。
traces.push({
  type: "scatter3d",
  mode: "markers+text",
  name: "start / goal",
  x: [DATA.start[0], DATA.goal[0]],
  y: [DATA.start[1], DATA.goal[1]],
  z: [DATA.start[2], DATA.goal[2]],
  text: ["START", "GOAL"],
  textposition: "top center",
  marker: { size: 6, color: ["#16a34a", "#dc2626"] },
});

// 使用预先计算的比例和坐标轴标题初始化图表，并开启旋转/缩放交互。
Plotly.newPlot("plot", traces, {
  margin: { l: 0, r: 0, t: 0, b: 0 },
  scene: {
    aspectmode: "manual",
    aspectratio: DATA.aspect_ratio,
    xaxis: { title: "X mm", range: DATA.x_range },
    yaxis: { title: "Y mm" },
    zaxis: { title: "Z mm" },
  },
  legend: { x: 1, xanchor: "right", y: 1 },
}, { responsive: true, scrollZoom: true });
  </script>
</body>
</html>
""".replace("__DATA__", data_json)

    # 统一输出路径并确保父目录存在。
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    print(f"[HTML] saved: {output}")
