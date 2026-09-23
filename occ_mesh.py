"""CAD 文件读取和三角网格转换工具。

本模块把 ``base/`` 目录中的 CAD 输入统一转换成两种中间结果：

* STEP/IGES 文件先读取为 OpenCASCADE（OCC）的 ``Shape`` 对象，随后可再
  通过 :func:`triangulate_shape_to_mesh` 转换为三角网格。
* STL 文件本身已经是三角网格，因此直接解析为 ``{"x", "y", "z", "i", "j", "k"}``
  字典，不再由 OCC 重新优化网格。

当前支持的扩展名为 ``.stp``、``.step``、``.igs``、``.iges`` 和 ``.stl``。
``.3dxml`` 没有对应解析器，即使加入扫描扩展名，也会在
:func:`read_cad_shape` 中被拒绝。STL 网格质量取决于 CATIA 导出时的网格设置；
因此“3DXML → STL → occ_mesh → main1”可能产生几何失真，输入前应检查网格精度。
"""


from __future__ import annotations # Python 3.7 及更早版本中，**类型注解里引用还没定义的类会报错**,保证向下兼容

from dataclasses import dataclass # 数据类装饰器, 当普通类要手动写 `__init__`、`__repr__`、`__eq__`时，这里可以直接省掉
from pathlib import Path # `pathlib` 是 Python 内置的**面向对象文件路径库**，替代老式的 `os.path`


@dataclass
class StepComponent:
    """扫描到的一个 CAD 组件及其可选的几何数据。

    ``shape`` 保存 STEP/IGES 的 OCC Shape，或 STL 的网格字典；
    ``mesh`` 保存最终统一格式的三角网格。两个字段都延迟加载，便于只扫描文件而不读取几何。
    """

    # 不带扩展名的组件名称，例如 ``ground``。
    name: str
    # CAD 文件的完整路径，使用字符串便于序列化和打印日志。
    file: str
    # 组件用途：通常是 ``ground``、``obstacle`` 或 ``ignore``。
    role: str
    # 路径规划时需要额外避开的距离，单位通常为毫米。
    clearance: float
    # 是否按实体障碍处理；False 可表示薄壳或软边界模型。
    solid: bool
    # 原始 OCC Shape 或 STL 网格；不读取时保持 None。
    shape: object | None = None
    # 统一的三角网格字典；不网格化时保持 None。
    mesh: dict | None = None


# 扫描 CAD 输入目录时允许的后缀；元组便于在循环中复用。
CAD_INPUT_EXTENSIONS = (".stp", ".step", ".igs", ".iges", ".stl")


def read_step_shape(path: Path):
    """读取 STEP 文件并返回 OCC Shape。

    OCC 的导入器采用惰性导入，因此先 ``ReadFile`` 读取文件，再调用
    ``TransferRoots`` 把顶层实体转移到 OCC 数据结构，最后取合并后的 Shape。
    """

    # 这些 OCC 模块启动较慢，只在真正读取 STEP 时导入。
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.STEPControl import STEPControl_Reader

    # 统一接受字符串和 Path 两种调用方式。
    path = Path(path)
    # 创建 STEP 专用读取器。
    reader = STEPControl_Reader()
    # 读取文件；返回值是 OCC 的状态码，而不是 Python 布尔值。
    status = reader.ReadFile(str(path))
    # 只有 RetDone 表示文件成功读入，其他状态统一转成易读异常。
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {path}")
    # 将文件中的根实体转移到当前 OCC 模型。
    reader.TransferRoots()
    # 返回由读取器组合出的单一 Shape，供后续网格化使用。
    return reader.OneShape()


def read_iges_shape(path: Path):
    """读取 IGES 文件并尽量转移所有根实体为 OCC Shape。

    某些 IGES 文件调用 ``TransferRoots`` 后仍没有有效 Shape；此时逐个调用
    ``TransferOneRoot`` 再尝试一次，以兼容包含多个或异常根实体的文件。
    """

    # 延迟导入 IGES 相关 OCC 类，避免仅处理 STL 时加载 OCC。
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.IGESControl import IGESControl_Reader

    # 将输入规范化为 Path，错误信息中也能显示实际路径。
    path = Path(path)
    # 创建 IGES 读取器并执行文件读取。
    reader = IGESControl_Reader()
    status = reader.ReadFile(str(path))
    # 读取失败时立即终止，避免后续对无效读取器继续操作。
    if status != IFSelect_RetDone:
        raise RuntimeError(f"IGES read failed: {path}")

    # 首先按 OCC 的标准流程转移所有根实体。
    reader.TransferRoots()
    # 取得转移结果；部分文件在这里就能得到有效 Shape。
    shape = reader.OneShape()
    # Shape 存在且不是空对象时直接返回。
    if shape is not None and not shape.IsNull():
        return shape

    # 标准转移没有结果时，逐个尝试根实体；索引从 1 开始是 OCC API 的约定。
    for index in range(1, reader.NbRootsForTransfer() + 1):
        try:
            # 只转移当前根，避免一个坏根阻塞整个文件。
            reader.TransferOneRoot(index)
        except Exception:
            # 某个根转移失败时跳过它，继续尝试其他根。
            continue

    # 再次汇总逐根转移的结果。
    shape = reader.OneShape()
    # 所有根都没有产生有效 Shape 时报告明确错误。
    if shape is None or shape.IsNull():
        raise RuntimeError(f"IGES transfer produced no OCC shape: {path}")

    # 返回最终的 IGES 几何对象。
    return shape


def _empty_mesh() -> dict:
    """创建空的统一网格字典。

    ``x/y/z`` 保存顶点坐标，``i/j/k`` 保存每个三角形的三个顶点索引；
    索引从 0 开始，便于直接交给 NumPy、Plotly 或项目中的碰撞代码。
    """

    # 每个字段都使用独立列表，后续可以原地 append。
    return {"x": [], "y": [], "z": [], "i": [], "j": [], "k": [], "triangle_count": 0}


def _append_stl_triangle(mesh: dict, points: list[tuple[float, float, float]]) -> None:
    """把一个 STL 三角形追加到网格字典中。

    STL 通常会重复存储顶点，因此这里不做去重，直接追加三个顶点并建立索引，
    这样可以严格保留文件中的三角形顺序，也能避免顶点去重带来的索引错误。
    """

    # 当前顶点数量就是本三角形第一个顶点的起始索引。
    base = len(mesh["x"])
    # 依次写入三个顶点的坐标，并显式转成 float，保证类型统一。
    for x, y, z in points:
        mesh["x"].append(float(x))
        mesh["y"].append(float(y))
        mesh["z"].append(float(z))
    # 三角形引用刚追加的三个连续顶点。
    mesh["i"].append(base)
    mesh["j"].append(base + 1)
    mesh["k"].append(base + 2)


def _is_binary_stl(path: Path) -> bool:
    """用文件长度和头部三角形数量判断 STL 是否为二进制格式。

    二进制 STL 固定由 80 字节头、4 字节数量和每个 50 字节的三角形记录组成。
    该判断是快速启发式；极少数非标准文件可能需要人工转换后再读取。
    """

    # struct 只在判断 STL 时导入，``<I`` 表示小端无符号 32 位整数。
    import struct

    # 获取文件总字节数，用于和理论二进制长度比较。
    size = path.stat().st_size
    # 连头部和数量字段都不完整的文件不可能是有效二进制 STL。
    if size < 84:
        return False
    # 打开二进制文件并跳过 80 字节 STL 头。
    with path.open("rb") as file:
        file.seek(80)
        # 读取头部中的三角形数量。
        triangle_count = struct.unpack("<I", file.read(4))[0]
    # 只有文件长度恰好符合固定记录大小时才判定为二进制 STL。
    return size == 84 + triangle_count * 50


def read_binary_stl_mesh(path: Path) -> dict:
    """读取二进制 STL，并返回项目统一的三角网格字典。"""

    # 用 struct 按 STL 记录布局解包二进制浮点数。
    import struct

    # 先创建空网格容器。
    mesh = _empty_mesh()
    # 每个三角形记录为 12 个 float（法向量 3 个 + 顶点 9 个）和 1 个无符号短整数。
    record = struct.Struct("<12fH")
    # 以二进制方式打开文件，避免文本编码干扰字节读取。
    with Path(path).open("rb") as file:
        # 跳过 80 字节头部，定位到三角形数量。
        file.seek(80)
        # 读取文件声明的三角形数量。
        triangle_count = struct.unpack("<I", file.read(4))[0]
        # 按声明数量逐条读取三角形记录。
        for _ in range(triangle_count):
            # 每次读取固定 50 字节，并解包成元组。
            values = record.unpack(file.read(50))
            # STL 的 values[0:3] 是法向量，顶点从下标 3 开始，每三个数一个点。
            _append_stl_triangle(
                mesh,
                [
                    (values[3], values[4], values[5]),
                    (values[6], values[7], values[8]),
                    (values[9], values[10], values[11]),
                ],
            )
    # 索引列表的长度就是实际成功解析的三角形数量。
    mesh["triangle_count"] = len(mesh["i"])
    # 返回包含顶点、索引和统计值的网格。
    return mesh


def read_ascii_stl_mesh(path: Path) -> dict:
    """读取 ASCII STL 中的 ``vertex x y z`` 行并组成三角网格。"""

    # 初始化网格及当前正在收集的三角形顶点。
    mesh = _empty_mesh()
    triangle: list[tuple[float, float, float]] = []

    # 忽略 ASCII STL 头部、facet、normal 等行，只关心 vertex 行。
    with Path(path).open("r", encoding="utf-8", errors="ignore") as file:
        # 逐行处理，避免一次性把大型 STL 全部读入内存。
        for line in file:
            # 去掉首尾空白后按空格切分字段。
            parts = line.strip().split()
            # 合法顶点行至少有关键字和三个坐标值。
            if len(parts) >= 4 and parts[0].lower() == "vertex":
                # 解析坐标并加入当前三角形；float 可兼容整数和科学计数法。
                triangle.append((float(parts[1]), float(parts[2]), float(parts[3])))
                # STL 每三个 vertex 组成一个三角形，收齐后立即追加并清空缓存。
                if len(triangle) == 3:
                    _append_stl_triangle(mesh, triangle)
                    triangle = []

    # 以实际索引数量记录解析到的三角形数。
    mesh["triangle_count"] = len(mesh["i"])
    # 返回 ASCII STL 的网格结果。
    return mesh


def read_stl_mesh(path: Path) -> dict:
    """自动识别 STL 格式并读取；空网格会被视为输入错误。"""

    # 统一路径类型，后续判断和错误信息都使用同一个对象。
    path = Path(path)
    # 根据文件结构选择二进制或 ASCII 解析器。
    mesh = read_binary_stl_mesh(path) if _is_binary_stl(path) else read_ascii_stl_mesh(path)
    # 没有任何三角形通常说明文件损坏、为空或格式并非 STL。
    if mesh["triangle_count"] <= 0:
        raise RuntimeError(f"STL mesh has no triangles: {path}")
    # 返回已验证至少含一个三角形的网格。
    return mesh


def is_mesh_dict(value) -> bool:
    """判断对象是否已经是本项目约定的三角网格字典。"""

    # 只检查必要字段存在，不强制检查列表长度或数值类型，以兼容已有调用方。
    return isinstance(value, dict) and all(key in value for key in ("x", "y", "z", "i", "j", "k"))


def read_cad_shape(path: Path):
    """按扩展名读取 CAD 文件，返回 OCC Shape 或 STL 网格字典。"""

    # Path.suffix 只取最后一个扩展名，并统一为小写以兼容大写后缀。
    path = Path(path)
    suffix = path.suffix.lower()
    # STEP 的两种常用扩展名交给 STEP 专用读取器。
    if suffix in {".stp", ".step"}:
        return read_step_shape(path)
    # IGES 的两种常用扩展名交给 IGES 专用读取器。
    if suffix in {".igs", ".iges"}:
        return read_iges_shape(path)
    # STL 已经是网格，直接返回网格字典。
    if suffix == ".stl":
        return read_stl_mesh(path)
    # 其他格式（例如 3DXML）当前没有解析器，明确拒绝而不是静默跳过。
    raise ValueError(f"Unsupported CAD input format: {path}")


def triangulate_shape_to_mesh(
    shape,
    linear_deflection: float = 8.0,
    angular_deflection: float = 0.5,
    max_triangles: int | None = None,
) -> dict:
    """把 OCC Shape 三角化，或直接透传已经存在的网格字典。

    ``linear_deflection`` 控制曲面离散的线性误差，``angular_deflection`` 控制
    角度误差。OCC 的三角索引从 1 开始，本函数会转换为项目使用的 0-based 索引。
    ``max_triangles`` 可用于调试或限制大模型输出规模。
    """

    # STL 读取结果已经是目标格式，不需要再次调用 OCC 网格化。
    if is_mesh_dict(shape):
        return shape
    # 空 Shape 无法提取面，提前报错能避免更深层的 OCC 异常。
    if shape is None:
        raise RuntimeError("Cannot triangulate an empty CAD shape.")

    # 延迟导入网格化及遍历相关 OCC 类，减少仅处理 STL 时的启动开销。
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopLoc import TopLoc_Location

    # 让 OCC 为 Shape 的各个面生成三角网格；参数显式转 float 以避免配置类型问题。
    BRepMesh_IncrementalMesh(
        shape,
        float(linear_deflection),
        False,
        float(angular_deflection),
        True,
    )

    # 分别准备顶点坐标数组和三个三角形索引数组。
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    ii: list[int] = []
    jj: list[int] = []
    kk: list[int] = []

    # 只遍历面（FACE），因为三角化结果挂在面上。
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    # OCC 遍历器在 More() 为真时表示仍有当前面。
    while exp.More():
        # 取得当前面。
        face = exp.Current()
        # loc 用来接收该面的装配位置变换。
        loc = TopLoc_Location()
        # 从当前面读取已经由 BRepMesh 生成的三角剖分。
        tri = BRep_Tool.Triangulation(face, loc)
        # 某些面可能没有三角剖分，跳过它们而继续处理其他面。
        if tri is not None:
            # 取得面所在装配实例的变换矩阵，确保顶点落在全局坐标系。
            trsf = loc.Transformation()
            # 当前面顶点在总顶点数组中的偏移量。
            base = len(xs)

            # OCC 数组通常从 1 开始，因此范围写成 1 到 NbNodes()（含末端）。
            for idx in range(1, tri.NbNodes() + 1):
                try:
                    # 新版 pythonocc 可以直接按索引读取节点。
                    p = tri.Node(idx)
                except Exception:
                    # 兼容旧版绑定：通过 Nodes().Value(idx) 读取同一节点。
                    p = tri.Nodes().Value(idx)
                # 将局部坐标变换到全局坐标，处理装配平移和旋转。
                p = p.Transformed(trsf)
                # 写入 Python float，避免保留 OCC 数值对象。
                xs.append(float(p.X()))
                ys.append(float(p.Y()))
                zs.append(float(p.Z()))

            # 遍历当前面的所有三角形。
            for idx in range(1, tri.NbTriangles() + 1):
                # 达到限制后停止当前面三角形读取。
                if max_triangles is not None and len(ii) >= int(max_triangles):
                    break
                try:
                    # 新版绑定的直接读取方式。
                    t = tri.Triangle(idx)
                except Exception:
                    # 兼容旧版绑定的数组读取方式。
                    t = tri.Triangles().Value(idx)
                # Get() 返回当前三角形引用的三个局部节点编号。
                n1, n2, n3 = t.Get()
                # 加上当前面的顶点偏移，并把 OCC 的 1-based 编号改成 0-based。
                ii.append(base + n1 - 1)
                jj.append(base + n2 - 1)
                kk.append(base + n3 - 1)

        # 如果达到全局三角形上限，退出面遍历；否则移动到下一个面。
        if max_triangles is not None and len(ii) >= int(max_triangles):
            break
        exp.Next()

    # 返回与 STL 相同的网格字段，保证下游无需区分来源格式。
    return {"x": xs, "y": ys, "z": zs, "i": ii, "j": jj, "k": kk, "triangle_count": len(ii)}


def mesh_points(mesh: dict) -> list[list[float]]:
    """把分离存储的 ``x/y/z`` 数组合并成 ``[[x, y, z], ...]``。"""

    # zip 按最短数组对齐坐标，逐项转成普通 float，便于 JSON 序列化。
    return [[float(x), float(y), float(z)] for x, y, z in zip(mesh["x"], mesh["y"], mesh["z"])]


def infer_ground_z(mesh: dict, default_z: float = -300.0) -> float:
    """用网格顶点 Z 坐标的中位数估计地面高度。

    没有顶点时返回 ``default_z``；当前实现保留原项目行为，用中位数而不是最小值
    来降低少量离群点对地面高度推断的影响。
    """

    # 从网格中读取并统一转换所有 Z 坐标。
    zs = [float(z) for z in mesh.get("z", [])]
    # 空网格没有可推断数据，使用调用方提供的默认高度。
    if not zs:
        return float(default_z)
    # 排序后取中间位置；偶数个元素时使用右侧的中间元素，保持原逻辑。
    zs.sort()
    return float(zs[len(zs) // 2])


def scan_cad_files(base_step_dir: Path) -> list[Path]:
    """扫描目录中的支持格式文件，去重后按文件名排序返回。"""

    # 允许调用方传入字符串路径。
    base_step_dir = Path(base_step_dir)
    # 输入目录不存在通常意味着配置错误，因此直接抛出清晰异常。
    if not base_step_dir.exists():
        raise FileNotFoundError(f"BASE_STEP_DIR does not exist: {base_step_dir}")
    # 收集所有支持扩展名的文件。
    files: list[Path] = []
    for suffix in CAD_INPUT_EXTENSIONS:
        # 同时匹配小写后缀和大写后缀；Windows 虽然不区分大小写，但此写法也适用于其他系统。
        files.extend(base_step_dir.glob(f"*{suffix}"))
        files.extend(base_step_dir.glob(f"*{suffix.upper()}"))
    # 以不区分大小写的完整路径作为键去重，值保留实际 Path 对象。
    unique = {str(p).lower(): p for p in files}
    # 用小写文件名排序，使扫描顺序稳定，便于日志和结果复现。
    return sorted(unique.values(), key=lambda p: p.name.lower())


def scan_step_files(base_step_dir: Path) -> list[Path]:
    """兼容旧调用方的函数名；现在会扫描 STEP、IGES 和 STL。"""

    # 委托给新的通用扫描函数，避免维护两套扩展名规则。
    return scan_cad_files(base_step_dir)


def component_rule(step_path: Path, settings) -> tuple[str, float, bool]:
    """根据文件名和 settings 返回组件角色、净空和实体标志。"""

    # 文件名匹配优先使用完整文件名，其次使用不带扩展名的 stem。
    key_file = step_path.name.lower().strip()
    key_stem = step_path.stem.lower().strip()
    # 规则键统一小写并去空格，让配置大小写不敏感。
    rules = {str(k).lower().strip(): v for k, v in settings.SPECIAL_COMPONENT_RULES.items()}

    # 完整文件名优先；没有匹配时再按 stem 查找。
    rule = rules.get(key_file) or rules.get(key_stem)
    if rule is not None:
        # 使用规则值，缺省字段回退到全局默认值。
        return (
            str(rule.get("role", "obstacle")),
            float(rule.get("clearance", settings.DEFAULT_OBSTACLE_CLEARANCE)),
            bool(rule.get("solid", settings.DEFAULT_COMPONENT_SOLID)),
        )
    # 没有显式规则但文件名包含 ground/floor 时，自动识别为地面实体。
    if "ground" in key_stem or "floor" in key_stem:
        return "ground", float(settings.DEFAULT_GROUND_CLEARANCE), True
    # 其余组件按普通障碍物和默认净空处理。
    return "obstacle", float(settings.DEFAULT_OBSTACLE_CLEARANCE), bool(settings.DEFAULT_COMPONENT_SOLID)


def load_components(settings, with_shape: bool = True, with_mesh: bool = True) -> tuple[list[StepComponent], list[StepComponent]]:
    """扫描并加载组件，返回 ``(有效组件, 忽略组件)`` 两个列表。

    ``with_shape`` 控制是否读取原始 CAD 几何，``with_mesh`` 控制是否继续生成
    三角网格。若 ``with_mesh`` 为 True，即使 ``with_shape`` 为 False 也必须先读取
    Shape，因为网格化需要几何输入；原实现的条件判断因此保持为
    ``with_shape or with_mesh``。
    """

    # 分别保存参与规划的组件和规则标记为 ignore 的组件。
    components: list[StepComponent] = []
    ignored: list[StepComponent] = []
    # 扫描 settings 指定的 CAD 输入目录。
    step_files = scan_cad_files(settings.BASE_STEP_DIR)
    # 输出总数，便于用户确认输入目录是否为空或漏文件。
    print(f"[CAD] scan: {len(step_files)} files in {settings.BASE_STEP_DIR}")

    # 按稳定顺序逐个处理文件，并从 1 开始编号显示进度。
    for index, path in enumerate(step_files, start=1):
        # 先根据文件名决定角色、净空和实体属性。
        role, clearance, solid = component_rule(path, settings)
        # 建立组件记录；几何字段暂时为空，稍后按开关填充。
        comp = StepComponent(path.stem, str(path), role, clearance, solid)
        # 打印规则结果，方便发现文件名匹配或默认值问题。
        print(
            f"[CAD] {index}/{len(step_files)} {path.name}: "
            f"role={role}, clearance={clearance:.1f}, solid={solid}"
        )
        # ignore 组件不读取几何，直接放入 ignored 列表。
        if role == "ignore":
            print(f"[CAD] {path.name}: ignored")
            ignored.append(comp)
            continue

        # 生成网格也依赖原始 Shape，因此任一读取开关为真时都先读 CAD。
        if with_shape or with_mesh:
            print(f"[CAD] {path.name}: read start", flush=True)
            comp.shape = read_cad_shape(path)
            print(f"[CAD] {path.name}: read done", flush=True)
        # 需要网格时，根据 settings 中的误差参数执行三角化。
        if with_mesh:
            print(f"[MESH] {path.name}: mesh start", flush=True)
            comp.mesh = triangulate_shape_to_mesh(
                comp.shape,
                settings.MESH_LINEAR_DEFLECTION,
                settings.MESH_ANGULAR_DEFLECTION,
                None,
            )
            # 从网格读取统计值；理论上 comp.mesh 不为空，但这里保留防御性判断。
            tri_count = int(comp.mesh.get("triangle_count", 0)) if comp.mesh else 0
            print(f"[MESH] {path.name}: mesh done, triangles={tri_count}", flush=True)
        # 完成当前组件后加入有效组件列表。
        components.append(comp)

    # 返回两个列表，调用方可以分别处理有效和忽略的文件。
    return components, ignored
