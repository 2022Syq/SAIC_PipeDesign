
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = PROJECT_DIR / "outputs"

# Input CAD files
BASE_STEP_DIR = PROJECT_DIR / "base"
# 用户上传的 CATIA Part 文件目录；upload.py 会把其中的 Part 文件转换为 IGES 后放入 base。
UPLOAD_DIR = PROJECT_DIR / "upload"
"""
# Pipe endpoints and tangent reference points, in mm.
# Desired travel directions are:
#   START_DIR_POINT - START
#   GOAL_DIR_POINT - GOAL
START = [667.89992539, -677.49999999, -123.04919729]
GOAL = [3467.9829, -5.33132836e-06, -49.9114771]
START_DIR_POINT = [677.89992539, -677.49999999, -123.04919729]
GOAL_DIR_POINT = [3487.3493454673044, -5.334146565711675e-06, -34.101965995796895]
"""
START = [36.703776808667, -18.194715779144, 554.539252627615]
GOAL = [303.451906833262, 12.999672776061, 440.22547724222]
START_DIR_POINT = [51.982415501506, -18.194718322136, 554.539254174035]
GOAL_DIR_POINT = [313.451906833115, 12.999671724166, 440.225375445311]

# Hanger mounting points measured from CATIA. main_point.py searches GUADIAN,
# GUADIAN1 ... GUADIAN10 and writes all found points here.
HANGER_POINTS = []
# A valid pipe route should pass within this distance of each hanger point (mm).
HANGER_POINT_DISTANCE_RANGE = 100.0

# 空调管规格。金属管是连续芯管，胶管只在指定管段外包覆。
# 弯曲半径的 minimum 值已经按 max(Excel 明确值, 1.5D) 整理；胶管使用 Excel 的固定值。
ACTIVE_PIPE_SPEC = "metal_19_05"
PIPE_SPECS = {
    "metal_9": {
        "kind": "metal",
        "material": "3003 H14",
        "outer_diameter": 9.0,
        "inner_diameter": 6.5,
        "wall_thickness": 1.25,
        "flexibility": "rigid",
        "recommended_bend_radius": 15.0,
        "minimum_bend_radius": 15.0,
        "straight_recommended": 13.5,
        "straight_minimum": 9.0,
    },
    "metal_12": {
        "kind": "metal",
        "material": "3003 H14",
        "outer_diameter": 12.0,
        "inner_diameter": 8.8,
        "wall_thickness": 1.6,
        "flexibility": "rigid",
        "recommended_bend_radius": 20.0,
        "minimum_bend_radius": 20.0,
        "straight_recommended": 18.0,
        "straight_minimum": 12.0,
    },
    "metal_16": {
        "kind": "metal",
        "material": "3003 H14",
        "outer_diameter": 16.0,
        "inner_diameter": 12.8,
        "wall_thickness": 1.6,
        "flexibility": "rigid",
        "recommended_bend_radius": 25.0,
        "minimum_bend_radius": 25.0,
        "straight_recommended": 20.0,
        "straight_minimum": 16.0,
    },
    "metal_19_05": {
        "kind": "metal",
        "material": "3103 H112",
        "outer_diameter": 19.05,
        "inner_diameter": 16.05,
        "wall_thickness": 1.5,
        "flexibility": "rigid",
        "recommended_bend_radius": 27.5,
        "minimum_bend_radius": 28.575,
        "straight_recommended": 25.0,
        "straight_minimum": 20.0,
    },
    "hose_8DE3": {
        "kind": "hose",
        "material": "8DE3",
        "outer_diameter": 15.0,
        "inner_diameter": 8.4,
        "wall_thickness": 3.3,
        "flexibility": "flexible",
        "recommended_bend_radius": 50.0,
        "minimum_bend_radius": 50.0,
        "straight_recommended": None,
        "straight_minimum": None,
        "aluminum_sleeve_back_recommended": 20.0,
        "aluminum_sleeve_back_minimum": 15.0,
    },
    "hose_11DE3": {
        "kind": "hose",
        "material": "11DE3",
        "outer_diameter": 19.0,
        "inner_diameter": 11.9,
        "wall_thickness": 3.55,
        "flexibility": "flexible",
        "recommended_bend_radius": 60.0,
        "minimum_bend_radius": 60.0,
        "straight_recommended": None,
        "straight_minimum": None,
        "aluminum_sleeve_back_recommended": 20.0,
        "aluminum_sleeve_back_minimum": 12.0,
    },
    "hose_15DE3": {
        "kind": "hose",
        "material": "15DE3",
        "outer_diameter": 24.0,
        "inner_diameter": 15.9,
        "wall_thickness": 4.05,
        "flexibility": "flexible",
        "recommended_bend_radius": 75.0,
        "minimum_bend_radius": 75.0,
        "straight_recommended": None,
        "straight_minimum": None,
        "aluminum_sleeve_back_recommended": 20.0,
        "aluminum_sleeve_back_minimum": 16.0,
    },
}

# 工程师后续会把 CAD 中的起点、终点或管段编号填入这里；空表时使用 ACTIVE_PIPE_SPEC。
PIPE_SEGMENTS = []

# 已确认的空调管通用工艺规则。
BEND_ANGLE_RECOMMENDED_DEG = 10.0
BEND_ANGLE_MINIMUM_DEG = 5.0
WELD_SPACING_RECOMMENDED = 30.0
WELD_SPACING_MINIMUM = 25.0
VALVE_STRAIGHT_RECOMMENDED = 25.0
VALVE_STRAIGHT_MINIMUM = 5.0
ALUMINUM_SLEEVE_AXIS_CLEARANCE_RECOMMENDED = 80.0
ALUMINUM_SLEEVE_AXIS_CLEARANCE_MINIMUM = 65.0
ALUMINUM_SLEEVE_BACK_LENGTHS = {
    "hose_8DE3": (20.0, 15.0),
    "hose_11DE3": (20.0, 12.0),
    "hose_15DE3": (20.0, 16.0),
    "metal_19_05": (30.0, 25.0),
}

# 连接件只记录型号，当前不参与路线约束。
CONNECTOR_CATALOG = (
    "SKC-9AA", "SKC-9AC", "SKC-12AA", "SKC-16AA",
    "SKC-16AC", "SKC-19AF", "SKC-19AB",
)

# 工程师确认 CAD 对应关系后填写；空表不会凭空推断工艺件位置。
WELD_POINTS = []
VALVE_SEATS = []
ALUMINUM_SLEEVES = []

# 规划阶段使用当前测试路线的最大包络半径；目录中的未选规格不参与本次缓冲。
def selected_pipe_spec_ids() -> set[str]:
    ids = {ACTIVE_PIPE_SPEC}
    for segment in PIPE_SEGMENTS:
        ids.add(str(segment.get("core_spec", ACTIVE_PIPE_SPEC)))
        cover = segment.get("cover_spec")
        if cover:
            ids.add(str(cover))
    unknown = ids.difference(PIPE_SPECS)
    if unknown:
        raise ValueError(f"Unknown pipe specs: {sorted(unknown)}")
    return ids


ROUTE_MAX_ENVELOPE_RADIUS = max(
    float(PIPE_SPECS[spec_id]["outer_diameter"]) / 2.0
    for spec_id in selected_pipe_spec_ids()
)
PIPE_DIAMETER = 2.0 * ROUTE_MAX_ENVELOPE_RADIUS
PIPE_RADIUS = ROUTE_MAX_ENVELOPE_RADIUS
DEFAULT_OBSTACLE_CLEARANCE = 20.0
DEFAULT_GROUND_CLEARANCE = 150.0
DEFAULT_COMPONENT_SOLID = True
DEFAULT_SOFT_PENALTY = 1.0

# Component rules, matched by STEP filename or stem, case-insensitive.
SPECIAL_COMPONENT_RULES = {
    "ground": {
        "role": "ground",
        "clearance": 150.0,
        "boundary": "hard",
    },
    "see_through": {
        # 半透明显示对象只用于 CATIA 观察，不参与空间和软障碍评分。
        "role": "ignore",
        "clearance": 0.0,
        "solid": False,
        "boundary": "none",
    },
}

# Section space
SECTION_WIDTH_Y = 2100.0
SECTION_CENTER_Y = 0.0
SECTION_HEIGHT_ABOVE_GROUND = 600.0
SECTION_DX = 20.0
SECTION_DY = 20.0
X_PADDING_BEFORE_START = 100.0
X_PADDING_AFTER_GOAL = 100.0
X_RANGE = None

# Mesh and polygon cleanup
MESH_LINEAR_DEFLECTION = 8.0
MESH_ANGULAR_DEFLECTION = 0.5
MIN_FREE_REGION_AREA = 100.0
POLYGON_SIMPLIFY_TOLERANCE = 0.5
SOLID_LOOP_CLOSE_TOLERANCE = 8.0
SOLID_MIN_FILL_AREA = MIN_FREE_REGION_AREA
SOLID_FILL_FROM_BUFFER = True

# Section generation
SECTION_PARALLEL_WORKERS = 6
ENABLE_SPACE_CHECKPOINT = True

# Bidirectional route planner
ROUTE_SEGMENT_SAMPLE_STEP = 8.0
BIDIR_STATE_KEEP = 12
BIDIR_LOOKAHEAD_SECTIONS = 4

BIDIR_SMALL_STEP = 42.0
BIDIR_MEDIUM_STEP = 78.0
BIDIR_LARGE_Y_STEP = 125.0

BIDIR_TRUNK_STEP_DY_LIMIT = 45.0
BIDIR_TRUNK_STEP_DZ_LIMIT = 28.0
BIDIR_TRUNK_CLEAR_STRAIGHT_BONUS = 520.0

BIDIR_MIN_BRANCH_PROGRESS = 0.58
BIDIR_BRANCH_FORCE_AFTER_PROGRESS = 0.86
BIDIR_BRANCH_BLEND_DISTANCE = 520.0
BIDIR_BRANCH_TRIGGER_AREA_RATIO = 0.30

BIDIR_Z_OVERSHOOT_ALLOW = 28.0
BIDIR_Z_UP_PENALTY = 18.0
BIDIR_Z_ABOVE_GOAL_PENALTY = 120.0
BIDIR_END_DIRECTION_WEIGHT = 1800.0
BIDIR_DYNAMIC_CONNECT_DISTANCE = 80.0
BIDIR_DYNAMIC_MAX_STEPS = 1000

# Joint X/Y section lattice planner
JOINT_GRID_DZ = 20.0
JOINT_MAX_EXPANDED_NODES = 250000
JOINT_HEURISTIC_WEIGHT = 1.08
JOINT_ENDPOINT_TANGENT_LENGTH = 30.0
JOINT_SMOOTH_LOOKAHEAD = 80
JOINT_ENDPOINT_CONNECTOR_MAX_ANGLE_DEG = 35.0
JOINT_ENDPOINT_CURVE_SAMPLE_STEP = 6.0
JOINT_ENDPOINT_TRANSITION_LOOKAHEAD = 5
JOINT_ENDPOINT_MIN_BEND_RADIUS = 100.0
JOINT_ENDPOINT_MAX_TANGENT_ERROR_DEG = 3.0
JOINT_REQUIRE_SMOOTH_ENDPOINTS = True

# Main2 额外安全余量（mm）。允许范围为 0～20 mm；20 表示在零件原有净空要求之外再增加 20 mm。
MAIN2_EXTRA_CLEARANCE = 20.0

SOFT_BAD_WEIGHT = 18000.0
ENGINEER_SOFT_BAD_WEIGHT = 1800.0
HARD_BAD_WEIGHT = 260000.0
SOFT_OBSTACLE_ENABLED = False
SMALL_BEND_PENALTY = 120.0

# Distance plot
DISTANCE_SAMPLE_STEP = 20.0
DISTANCE_Y_MAX = 40.0

# Outputs
SPACE_JSON = OUTPUT_DIR / "planning_space.json"
SPACE_CHECKPOINT_JSON = OUTPUT_DIR / "planning_space_checkpoint.json"
SPACE_HTML = OUTPUT_DIR / "planning_space.html"
SPACE_HTML_SECTION_STRIDE = 4
HTML_X_MARGIN = 1000.0
BIDIR_RESULT_JSON = OUTPUT_DIR / "route_bidirectional.json"
BIDIR_HTML = OUTPUT_DIR / "route_bidirectional.html"
BIDIR_DISTANCE_PNG = OUTPUT_DIR / "route_bidirectional_distance.png"

# Engineering pipe reconstruction from the main2 guide routes
# None means all main2 routes are converted into engineered candidates.
ENGINEERING_SOURCE_ROUTE_NAME = None
# 仅作为旧 route JSON 缺少半径字段时的几何兜底；空调管主流程使用 PIPE_SPECS。
ENGINEERING_BEND_RADIUS = 150.0
# Main3 转角两侧直线的最小内部夹角（度），小于该值的尖角不允许；同时作为单段圆弧最大分段角。
ENGINEERING_MAX_BEND_ANGLE_DEG = 90.0
ENGINEERING_ARC_SAMPLE_ANGLE_DEG = 4.0
ENGINEERING_SIMPLIFY_TOLERANCE = 25.0
# Main3 主要布置转角数量上限，不包含起点和终点各自的必要切向过渡。
ENGINEERING_MAX_TURN_COUNT = 10
# 旧参数保留给历史 route JSON；空调管工程化阶段改用规格表中的 minimum_bend_radius。
ENGINEERING_MIN_BEND_RADIUS = 25.0
# Main3 自动缩小圆弧半径时每次递减的步长（mm）。
ENGINEERING_BEND_RADIUS_STEP = 10.0
# Main3 最终双截面验收的附加数值安全带（mm），用于抵消切片间距和插值误差。
ENGINEERING_VALIDATION_EXTRA_CLEARANCE = 2.0
# Main3 的 Z 高度简化容差（mm）。0 表示关闭，防止无碰撞约束的 Z 插值侵入车身。
ENGINEERING_Z_SIMPLIFY_TOLERANCE = 0.0
ENGINEERING_MIN_SEGMENT_LENGTH = 40.0
ENGINEERING_S_BEND_MIN_TURN_DEG = 110.0
ENGINEERING_S_BEND_MAX_SPAN = 420.0
ENGINEERING_S_BEND_MIN_DETOUR = 60.0
ENGINEERING_WIGGLE_SMOOTH_ROUTE_NAMES = ["04_bidir_axis_xy"]
ENGINEERING_WIGGLE_MIN_TURN_DEG = 55.0
ENGINEERING_WIGGLE_MAX_SPAN = 430.0
ENGINEERING_WIGGLE_MIN_DETOUR = 45.0
ENGINEERING_TUBE_SEGMENTS = 16
ENGINEERING_RESULT_JSON = OUTPUT_DIR / "route_engineered.json"
ENGINEERING_HTML = OUTPUT_DIR / "route_engineered.html"
