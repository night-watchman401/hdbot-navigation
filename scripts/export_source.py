#!/usr/bin/env python3
"""导出模块源码审阅目录；不包含外部依赖，不执行 Git 或网络操作。"""

import argparse
from pathlib import Path
import shutil


MODULES = (
    'hdbot_gtsam_backend', 'hdbot_gnss_msgs', 'nav2_custom_planner',
    'nav2_custom_controller', 'autopartol_robot',
)


def export_source(destination):
    """按明确的白名单导出，拒绝覆盖现有目录。"""
    root = Path(__file__).resolve().parents[1]
    destination = Path(destination).expanduser().resolve()
    if destination == root or root in destination.parents:
        raise ValueError('导出目录必须位于项目之外，避免递归复制或混入工作区')
    if destination.exists():
        raise FileExistsError(f'目标已经存在，不覆盖：{destination}')
    sources = [root / 'README.md', root / 'THIRD_PARTY.md', root / 'LICENSE',
               root / 'NOTICE', root / '.gitignore',
               root / 'docs', root / 'scripts']
    sources.extend(root / 'chapt8_ws' / 'src' / module for module in MODULES)
    for source in sources:
        if not source.exists():
            raise FileNotFoundError(f'缺少导出内容：{source}')
    ignore = shutil.ignore_patterns(
        '__pycache__', '.pytest_cache', '*.pyc', '*.pyo', '*.egg-info',
        'build', 'install', 'log', '.vscode', '.git')
    destination.mkdir(parents=True)
    for source in sources:
        target = destination / source.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.is_dir():
            shutil.copytree(source, target, ignore=ignore)
        else:
            shutil.copy2(source, target)
    print(f'已导出 {len(MODULES)} 个模块：{destination}')
    print('已包含 Apache-2.0 许可证和外部依赖说明，未执行发布。')


def main():
    """从命令行接收导出路径。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', help='项目之外且尚不存在的导出目录')
    args = parser.parse_args()
    try:
        export_source(args.destination)
    except (ValueError, OSError) as error:
        parser.exit(1, f'导出失败：{error}\n')


if __name__ == '__main__':
    main()
