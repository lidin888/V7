import os
import platform
import logging
from pathlib import Path

# 避免从openpilot.system.hardware导入PC以防止循环导入：
# 该模块在硬件初始化期间使用。通过检查TICI标记文件的存在来本地确定PC。
PC = not os.path.isfile('/TICI')

# 默认下载缓存根目录
DEFAULT_DOWNLOAD_CACHE_ROOT = "/tmp/comma_download_cache"

# 配置日志记录
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class Paths:
  """用于处理系统路径的工具类

  该类提供了一组静态方法来处理系统各个部分的路径，包括：
  - 日志文件
  - 缓存目录
  - 配置文件
  - 持久化数据
  - 共享内存
  等
  """

  @staticmethod
  def comma_home() -> str:
    """返回comma的主目录路径

    返回值:
        str: 主目录的完整路径，包含可能的OPENPILOT_PREFIX环境变量
    """
    try:
      prefix = os.environ.get("OPENPILOT_PREFIX", "")
      return os.path.join(str(Path.home()), ".comma" + prefix)
    except Exception as e:
      logger.error(f"获取comma主目录失败: {e}")
      return os.path.join(str(Path.home()), ".comma")

  @staticmethod
  def log_root() -> str:
    """返回日志根目录路径

    返回值:
        str: 日志目录的完整路径
    """
    try:
      if os.environ.get('LOG_ROOT', False):
        return os.environ['LOG_ROOT']
      elif PC:
        return str(Path(Paths.comma_home()) / "media" / "0" / "realdata")
      else:
        return '/data/media/0/realdata/'
    except Exception as e:
      logger.error(f"获取日志根目录失败: {e}")
      return '/tmp/realdata/'  # 返回一个临时目录作为后备

  @staticmethod
  def swaglog_root() -> str:
    """返回swaglog日志目录路径

    返回值:
        str: swaglog目录的完整路径
    """
    try:
      return os.path.join(Paths.comma_home(), "log") if PC else "/data/log/"
    except Exception as e:
      logger.error(f"获取swaglog目录失败: {e}")
      return "/tmp/log/"

  @staticmethod
  def swaglog_ipc() -> str:
    """返回swaglog IPC路径

    返回值:
        str: IPC路径字符串
    """
    try:
      prefix = os.environ.get("OPENPILOT_PREFIX", "")
      return f"ipc:///tmp/logmessage{prefix}"
    except Exception as e:
      logger.error(f"获取swaglog IPC路径失败: {e}")
      return "ipc:///tmp/logmessage"

  @staticmethod
  def download_cache_root() -> str:
    """返回下载缓存根目录路径

    返回值:
        str: 缓存目录的完整路径
    """
    try:
      if os.environ.get('COMMA_CACHE', False):
        return os.environ['COMMA_CACHE'] + "/"
      prefix = os.environ.get("OPENPILOT_PREFIX", "")
      return DEFAULT_DOWNLOAD_CACHE_ROOT + prefix + "/"
    except Exception as e:
      logger.error(f"获取下载缓存目录失败: {e}")
      return DEFAULT_DOWNLOAD_CACHE_ROOT + "/"

  @staticmethod
  def persist_root() -> str:
    """返回持久化数据根目录路径

    返回值:
        str: 持久化目录的完整路径
    """
    try:
      return os.path.join(Paths.comma_home(), "persist") if PC else "/persist/"
    except Exception as e:
      logger.error(f"获取持久化目录失败: {e}")
      return "/tmp/persist/"

  @staticmethod
  def stats_root() -> str:
    """返回统计数据根目录路径

    返回值:
        str: 统计数据目录的完整路径
    """
    try:
      return str(Path(Paths.comma_home()) / "stats") if PC else "/data/stats/"
    except Exception as e:
      logger.error(f"获取统计数据目录失败: {e}")
      return "/tmp/stats/"

  @staticmethod
  def config_root() -> str:
    """返回配置文件根目录路径

    返回值:
        str: 配置目录的完整路径
    """
    try:
      return Paths.comma_home() if PC else "/tmp/.comma"
    except Exception as e:
      logger.error(f"获取配置目录失败: {e}")
      return "/tmp/.comma"

  @staticmethod
  def shm_path() -> str:
    """返回共享内存路径

    在macOS上返回/tmp，因为它没有真正的共享内存

    返回值:
        str: 共享内存路径
    """
    try:
      if PC and platform.system() == "Darwin":
        return "/tmp"  # macOS上没有真正的共享内存，使用/tmp作为替代
      return "/dev/shm"
    except Exception as e:
      logger.error(f"获取共享内存路径失败: {e}")
      return "/tmp"

  @staticmethod
  def data_root() -> str:
    """返回数据根目录路径

    在设备上使用/data，在PC上使用用户特定的路径

    返回值:
        str: 数据根目录的完整路径
    """
    try:
      return Paths.comma_home() if PC else "/data"
    except Exception as e:
      logger.error(f"获取数据根目录失败: {e}")
      return "/tmp/data"  # 返回临时目录作为后备
