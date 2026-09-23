
"""把上传的 CATIA Part 文件转换为路径规划流程使用的 IGES 文件。

默认执行方式::

    python upload.py

脚本会扫描 ``./upload``，找到 CATPart、CATProduct 等 Part 文件，
再调用本机 CATIA 的 ``ExportData`` 接口，将结果写入 ``./base``。
其中：
-------------------------------------------
1.`ExportData` 是 **CATIA V5 Automation** 的文档对象方法，用于将打开的 `PartDocument` / `ProductDocument` / `DrawingDocument` 导出为中性 CAD 文件，对应界面上「另存为」的导出功能，VBA、Python (pycatia) 均可调用Dassault S...
2. CATPart 和 CATProduct 是 CATIA V5/V6 最核心的两类原生文件，本质是**「单个零件」和「装配体**」。product依赖引用的 Part / 子 Product，缺文件会报错.
3. ExportData具体代码：
```
Document.ExportData( FilePath As String, Format As String )
```
- **参数 1 FilePath**：完整输出文件路径，一般输出的文件格式为：


- **参数 2 Format**：导出格式标识,一般能导出3D 的有：
STP(.stp/.step)通用三维中性文件;
IGS(.igs/.iges)IGES 曲面实体交换格式;
STL(.stl)三维打印格式;
VRML(.wrl)虚拟现实建模语言;
3DXML(.3dxml)DS 轻量化三维XML格式;
-------------------------------------------
* ``upload/`` 是原始 CAD 模型的输入目录；
* ``upload.py`` 负责把 CATIA 原生文件转换成通用的 IGES  文件；
* ``base/`` 保存转换结果，后续路径规划代码会读取这些结果；
* CATIA 转换依赖本机安装的 CATIA 和 Python 的 ``pywin32`` 包。

脚本有两种运行角色：

* 主进程负责扫描、筛选、规划任务并汇总结果；
* ``--_worker`` 子进程只负责转换一个文件，主进程对每个文件单独设置超时。
"""

from __future__ import annotations

import argparse  # 解析命令行参数，例如 --dry-run、--overwrite。
import re  # 用正则表达式清理输出文件名中的非法字符。
import subprocess  # 让主进程为每个 CAD 文件启动独立的 worker 子进程。
import sys  # 获取当前 Python 解释器路径，确保 worker 使用同一环境。
from pathlib import Path  # 以跨平台方式处理文件和文件夹路径。

import settings  # 读取项目目录、upload 目录和 base 目录等集中配置。


# 允许扫描的 CATIA Part 文件扩展名；集合比较时会统一转为小写。
DEFAULT_PART_EXTENSIONS = (".CATPart", ".CATProduct", ".catpart", ".catproduct", ".part", ".prt")


def parse_args() -> argparse.Namespace:
    """定义并解析命令行参数。

    普通模式使用输入目录扫描多个文件；内部 worker 模式使用
    ``--_source`` 和 ``--_target`` 只转换一个文件。带下划线的参数
    是主进程内部调用的实现细节，通常不需要用户手动输入。
    """

    # 创建参数解析器；description 会显示在 python upload.py --help 中。
    parser = argparse.ArgumentParser(
        description="Convert uploaded Part files from upload/ to IGES files in base/."
    )
    # 指定原始 CAD 文件所在目录；没有传参时使用 settings.UPLOAD_DIR。
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path(getattr(settings, "UPLOAD_DIR", settings.PROJECT_DIR / "upload")),
        help="Folder containing uploaded Part files. Default: ./upload.",
    )
    # 指定 IGES 输出目录；没有传参时使用 settings.BASE_STEP_DIR。
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(getattr(settings, "BASE_STEP_DIR", settings.PROJECT_DIR / "base")),
        help="Folder where .igs files are written. Default: ./base.",
    )
    # 如果目标 .igs 已存在，允许删除旧文件并重新导出。
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing .igs files in the output folder.",
    )
    # 是否让 CATIA 窗口可见；默认隐藏 CATIA，适合批处理。
    parser.add_argument(
        "--visible",
        action="store_true",
        help="Show CATIA while converting.",
    )
    # 只处理文件名中包含指定文本的文件，比较时不区分大小写。
    parser.add_argument(
        "--only",
        help="Convert only files whose name contains this text.",
    )
    # 限制最多处理的文件数量；限制发生在 --only 筛选之后。
    parser.add_argument(
        "--limit",
        type=int,
        help="Convert at most this many files after filtering.",
    )
    # 只显示计划，不启动 CATIA，也不写出 IGES 文件。
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="List conversion actions without opening CATIA.",
    )
    # 导出后关闭“由本脚本打开”的文档；默认不关闭，避免 CATIA 弹出保存提示。
    parser.add_argument(
        "--close-after-export",
        action="store_true",
        help="Close documents opened by this script after export. Disabled by default to avoid CATIA save prompts.",
    )
    # 每个 worker 子进程允许使用的最大秒数，防止单个文件无限等待。
    parser.add_argument(
        "--per-file-timeout",
        type=int,
        default=600,
        help="Maximum seconds allowed for each single-file CATIA export. Default: 600.",
    )
    # 以下三个参数只供主进程启动 worker 使用，因此隐藏在 --help 输出中。
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--_source", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--_target", type=Path, help=argparse.SUPPRESS)
    # 返回 argparse 解析出的命名空间对象，例如 args.input_dir、args.dry_run。
    return parser.parse_args()


def safe_stem(value: str) -> str:
    """把源文件名转换为适合作为输出文件名的安全主名。

    字母、数字、下划线、连字符和中文会保留；其他字符统一替换为
    下划线。清理后如果没有任何有效字符，则使用 ``part`` 作为兜底名。
    """

    # 将不在允许范围内的连续字符替换为一个下划线，并去掉首尾下划线。
    stem = re.sub(r"[^0-9A-Za-z_\-\u4e00-\u9fff]+", "_", str(value)).strip("_")
    # 空文件名不能直接用于输出，因此使用固定的默认主名。
    return stem or "part"


def scan_part_files(input_dir: Path) -> list[Path]:
    """扫描输入目录并返回排序后的 CATIA Part 文件列表。

    这里只扫描当前目录，不递归进入子目录；临时锁定文件 ``~$...``
    会跳过，扩展名比较不区分大小写。目录不存在或路径不是目录时，
    立即抛出明确异常，让主流程显示问题所在。
    """

    # 统一转换为 Path，兼容调用方传入字符串路径的情况。
    input_dir = Path(input_dir)
    # 输入目录不存在时不能继续扫描，抛出文件不存在异常。
    if not input_dir.exists():
        raise FileNotFoundError(f"Upload folder does not exist: {input_dir}")
    # 路径存在但不是目录时，同样不能执行 iterdir() 扫描。
    if not input_dir.is_dir():
        raise NotADirectoryError(f"Upload path is not a folder: {input_dir}")

    # 用小写扩展名建立集合，使 .CATPart 和 .catpart 都能被识别。
    suffixes = {suffix.lower() for suffix in DEFAULT_PART_EXTENSIONS}
    # 逐个检查目录项：必须是文件、不能是 Office 临时文件、扩展名必须匹配。
    files = [
        path for path in input_dir.iterdir()
        if path.is_file()
        and not path.name.startswith("~$")
        and path.suffix.lower() in suffixes
    ]
    # 按文件名小写排序，保证每次规划任务的顺序稳定。
    return sorted(files, key=lambda p: p.name.lower())


def output_path_for(source: Path, output_dir: Path, used: set[Path]) -> Path:
    """为一个源文件生成不重复的 ``.igs`` 输出路径。

    ``used`` 记录当前批次已经分配过的目标路径，解决两个源文件清理
    后得到同名主名时的冲突，例如 ``a.CATPart`` 和 ``a.part``。
    """

    # 先使用“清理后的源文件主名 + .igs”作为首选目标路径。
    base = output_dir / f"{safe_stem(source.stem)}.igs"
    # candidate 是当前尝试使用的目标路径。
    candidate = base
    # 出现重名时，从 _2 开始递增后缀。
    index = 2
    # 只要本批次已经占用该路径，就继续寻找下一个候选名。
    while candidate in used:
        candidate = output_dir / f"{safe_stem(source.stem)}_{index}.igs"
        index += 1
    # 记录本次分配结果，避免后续源文件再次使用同一路径。
    used.add(candidate)
    # 返回最终确定的目标路径。
    return candidate


def import_catia_client():
    """导入 pywin32 的 CATIA COM 客户端模块。

    延迟到真正需要转换时才导入，这样 ``--dry-run`` 和文件扫描功能
    在没有安装 pywin32/CATIA 的环境中仍然可以使用。
    """

    try:
        # win32com.client 负责创建和调用 CATIA 的 Windows COM 对象。
        import win32com.client
        # 返回模块对象，调用方随后使用 Dispatch 创建 CATIA 实例。
        return win32com.client
    except Exception as exc:
        # 将底层导入错误转换成面向用户的安装提示，同时保留原异常链。
        raise RuntimeError(
            "upload.py requires pywin32 and a local CATIA installation. "
            "Install pywin32 in the Python environment used to run this script."
        ) from exc


def close_document(doc) -> None:
    """尽力关闭一个 CATIA 文档。

    关闭前先尝试标记为已保存，避免 CATIA 因未保存状态弹出交互式提示。
    清理动作本身不应覆盖导出过程的原始异常，因此这里会吞掉关闭阶段异常。
    """

    try:
        try:
            # 告诉 CATIA 文档当前无需保存；某些文档对象可能不支持该属性。
            doc.Saved = True
        except Exception:
            # 不支持 Saved 属性时继续尝试 Close，不让清理阻断主流程。
            pass
        # 请求 CATIA 关闭文档。
        doc.Close()
    except Exception:
        # 关闭失败只影响清理，不改变导出结果或异常状态。
        pass


def find_open_document(catia, source: Path):
    """在当前 CATIA 实例中查找已经打开的同名文档。

    找到时复用现有文档，避免重复打开；无法读取文档集合或遍历某个
    文档失败时跳过该项并继续搜索，最终找不到则返回 ``None``。
    """

    # CATIA 文档名称通常只包含文件名，因此使用小写文件名比较。
    source_name = source.name.lower()
    try:
        # COM 集合使用 1-based 索引，先读取当前打开文档数量。
        count = int(catia.Documents.Count)
    except Exception:
        # 无法读取集合时按“未打开”处理，让调用方尝试 Open。
        return None
    # 遍历 CATIA.Documents 的第 1 项到第 count 项。
    for index in range(1, count + 1):
        try:
            # 从 CATIA 文档集合中取出一个文档对象。
            doc = catia.Documents.Item(index)
            # 只按不区分大小写的名称匹配当前源文件。
            if str(doc.Name).lower() == source_name:
                # 返回已打开文档，后续直接导出，不重复打开。
                return doc
        except Exception:
            # 单个文档读取失败不影响其他文档的查找。
            continue
    # 遍历完成仍未找到匹配文档。
    return None


def export_igs(catia, source: Path, target: Path, close_after_export: bool = False) -> None:
    """使用 CATIA 将一个源文件导出为 IGES。

    如果源文件已经在 CATIA 中打开，则复用该文档；否则由本函数打开。
    只有“由本函数打开”的文档才会在 ``close_after_export=True`` 时关闭，
    用户原先打开的文档不会被脚本擅自关闭。
    """

    # 保存当前文档对象；发生异常时 finally 会据此执行清理。
    doc = None
    # 标记文档是否由本脚本打开，用于决定是否允许自动关闭。
    opened_by_script = False
    try:
        # 先查找已打开文档，避免 CATIA 中出现重复文档。
        doc = find_open_document(catia, source)
        if doc is None:
            # 没有已打开文档时，让 CATIA 打开源文件。
            print(f"[CATIA] open: {source}", flush=True)
            doc = catia.Documents.Open(str(source))
            # 只有这里打开的文档才归脚本负责关闭。
            opened_by_script = True
        else:
            # 复用现有文档，并在日志中明确说明。
            print(f"[CATIA] reuse open document: {source.name}", flush=True)
        # 调用 CATIA ExportData，以 IGES 格式写出目标文件。
        print(f"[CATIA] export IGES: {target}", flush=True)
        doc.ExportData(str(target), "igs")
    finally:
        # 无论导出成功还是失败，都按用户选项清理脚本打开的文档。
        if doc is not None and opened_by_script and close_after_export:
            close_document(doc)


def main() -> None:
    """脚本主入口：扫描文件、生成任务、逐个启动 CATIA worker 并汇总结果。"""

    # 先解析命令行参数，后续所有行为都由 args 控制。
    args = parse_args()
    # worker 模式由主进程为单个文件启动，直接进入 CATIA 转换分支。
    if args._worker:
        # worker 必须同时拿到源文件和目标文件，否则无法执行转换。
        if args._source is None or args._target is None:
            raise ValueError("worker mode requires --_source and --_target")
        # 只有真正转换时才加载 pywin32/CATIA 客户端。
        win32 = import_catia_client()
        # 通过 COM 启动或连接本机 CATIA 应用程序。
        catia = win32.Dispatch("CATIA.Application")
        try:
            # 根据 --visible 设置 CATIA 窗口是否可见。
            catia.Visible = bool(args.visible)
        except Exception:
            # 某些 CATIA/COM 版本不支持 Visible 属性时继续执行导出。
            pass
        # 将路径解析为绝对路径，调用单文件导出函数。
        export_igs(
            catia,
            args._source.resolve(),
            args._target.resolve(),
            close_after_export=bool(args.close_after_export),
        )
        # worker 完成后返回，不再执行主进程的批量调度逻辑。
        return

    # 将输入和输出目录都解析为绝对路径，日志和子进程使用同一位置。
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    # 输出目录不存在时自动创建；已存在时不报错。
    output_dir.mkdir(parents=True, exist_ok=True)

    # 扫描 upload 目录中的候选 Part 文件。
    part_files = scan_part_files(input_dir)
    if args.only:
        # 将筛选关键词转小写，实现不区分大小写的文件名过滤。
        needle = args.only.lower()
        # 仅保留文件名包含关键词的候选文件。
        part_files = [path for path in part_files if needle in path.name.lower()]
    if args.limit is not None:
        # 将 limit 限制为不小于 0，并截取筛选后的前 N 个文件。
        part_files = part_files[: max(0, int(args.limit))]
    # 输出本次扫描使用的目录和候选文件数量。
    print("=== Upload: Part -> IGES ===", flush=True)
    print(f"UPLOAD_DIR: {input_dir}", flush=True)
    print(f"BASE_STEP_DIR: {output_dir}", flush=True)
    print(f"part files: {len(part_files)}", flush=True)

    if not part_files:
        # 没有候选文件时无需启动 CATIA，正常结束。
        print("[DONE] no Part files found", flush=True)
        return

    # used 防止本批次内的不同源文件生成相同目标名。
    used: set[Path] = set()
    # jobs 保存真正需要执行的 (源文件, 目标文件) 对。
    jobs: list[tuple[Path, Path]] = []
    # skipped 保存因目标已存在且未指定 --overwrite 而跳过的文件。
    skipped: list[Path] = []
    # 为每个候选文件规划唯一输出路径，并决定跳过还是加入任务队列。
    for source in part_files:
        # 根据源文件主名生成一个不重复的 .igs 目标路径。
        target = output_path_for(source, output_dir, used)
        if target.exists() and not args.overwrite:
            # 默认保护已有结果，不覆盖旧文件。
            skipped.append(target)
            print(f"[SKIP] exists: {target.name} (use --overwrite to replace)", flush=True)
            continue
        # 目标不存在，或用户明确允许覆盖，加入待转换任务。
        jobs.append((source, target))
        print(f"[PLAN] {source.name} -> {target.name}", flush=True)

    if args.dry_run:
        # dry-run 到此结束，只展示计划，不触碰 CATIA 或输出文件。
        print("[DONE] dry run only", flush=True)
        return
    if not jobs:
        # 所有候选文件都已有结果且被跳过时，无需启动任何 worker。
        print(f"[DONE] nothing to convert, skipped={len(skipped)}", flush=True)
        return

    # converted 记录成功完成的任务数量。
    converted = 0
    # failed 保存失败源文件及原因，最后统一汇总并抛出错误。
    failed: list[tuple[Path, str]] = []
    # 每个文件单独执行，保证一个文件失败不会阻止后续文件继续处理。
    for source, target in jobs:
        try:
            if target.exists() and args.overwrite:
                # 覆盖前先删除旧 IGES，避免 CATIA 对已有文件的处理不一致。
                target.unlink()
            # 使用当前 Python 解释器重新运行本文件，并切换到单文件 worker 模式。
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--_worker",
                "--_source",
                str(source),
                "--_target",
                str(target),
            ]
            if args.visible:
                # 将主进程的 CATIA 可见性选项传给 worker。
                command.append("--visible")
            if args.close_after_export:
                # 将主进程的文档清理选项传给 worker。
                command.append("--close-after-export")
            # 启动 worker；失败返回码会抛出 CalledProcessError，超时会抛出 TimeoutExpired。
            subprocess.run(
                command,
                cwd=str(settings.PROJECT_DIR),
                check=True,
                timeout=max(1, int(args.per_file_timeout)),
            )
            # worker 正常返回，成功数加一并输出成功日志。
            converted += 1
            print(f"[OK] {source.name} -> {target.name}", flush=True)
        except subprocess.TimeoutExpired:
            # CATIA 超过单文件时限，记录失败并继续下一个文件。
            failed.append((source, f"TimeoutExpired: exceeded {args.per_file_timeout}s"))
            print(f"[FAIL] {source.name}: exceeded {args.per_file_timeout}s", flush=True)
        except subprocess.CalledProcessError as exc:
            # worker 以非零退出码结束，记录具体退出码。
            failed.append((source, f"CalledProcessError: exit={exc.returncode}"))
            print(f"[FAIL] {source.name}: worker exit={exc.returncode}", flush=True)
        except Exception as exc:
            # 捕获其他意外错误，保留异常类型和文本便于排查。
            failed.append((source, f"{type(exc).__name__}: {exc}"))
            print(f"[FAIL] {source.name}: {type(exc).__name__}: {exc}", flush=True)

    # 输出本轮转换的成功、跳过、失败数量以及结果目录。
    print(
        f"[DONE] converted={converted}, skipped={len(skipped)}, failed={len(failed)}, "
        f"output={output_dir}",
        flush=True,
    )
    if failed:
        # 只要有任意失败，就用异常通知调用方本轮并未完全成功。
        raise RuntimeError(f"{len(failed)} file(s) failed to convert")


if __name__ == "__main__":
    # 仅在直接执行 python upload.py 时进入主流程；被 import 时不自动运行。
    main()
    
