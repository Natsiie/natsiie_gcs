"""机器无关的配置文件路径解析。

约定：
- 配置里的数据文件（标定、单应等）写相对路径，相对于 ``NATSIIE_DATA_DIR``
  （默认 ``~/gcs``）。两台机器各自准备自己的数据目录即可，配置文件本身不用改。
- 运行期状态文件留空表示使用默认状态目录
  ``NATSIIE_STATE_DIR``（默认 ``~/.local/state/natsiie_gcs``）。
- 绝对路径和 ``~`` 原样展开，方便临时覆盖。
"""

import os
from pathlib import Path

DATA_ENV = 'NATSIIE_DATA_DIR'
STATE_ENV = 'NATSIIE_STATE_DIR'
DEFAULT_DATA_DIR = '~/gcs'
DEFAULT_STATE_DIR = '~/.local/state/natsiie_gcs'


def _expand(value):
    return os.path.expandvars(os.path.expanduser(str(value)))


def data_path(value):
    """把配置中的数据文件路径解析为 Path。

    绝对路径原样使用；相对路径相对 ``$NATSIIE_DATA_DIR``（默认 ``~/gcs``）。
    """
    if not value:
        raise ValueError('data file path cannot be empty')
    expanded = _expand(value)
    if os.path.isabs(expanded):
        return Path(expanded)
    return Path(_expand(os.environ.get(DATA_ENV, DEFAULT_DATA_DIR))) / expanded


def state_path(value, default_name):
    """解析运行期状态文件路径；``value`` 为空时使用默认状态目录。"""
    state_dir = Path(_expand(os.environ.get(STATE_ENV, DEFAULT_STATE_DIR)))
    if not value:
        return state_dir / default_name
    expanded = _expand(value)
    if os.path.isabs(expanded):
        return Path(expanded)
    return state_dir / expanded
