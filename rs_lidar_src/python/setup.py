from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os
from pathlib import Path

# -------------------------- 项目基础信息配置 --------------------------
__version__ = "0.3.6"
AUTHOR = "Zhejiang University of Finance and Economics - Point Cloud Team: Zhang Zhongqian, Li Huaiyuan, Cao Yiyun 浙江财经大学信息技术与人工智能学院点云组:张忠谦,李怀苑,曹宜云"
AUTHOR_EMAIL = "2789632062@qq.com"
PROJECT_NAME = "rs_lidar"
DESCRIPTION = "基于RoboSense LiDAR SDK的Python封装库,支持在线/PCAP点云读取与Numpy转换"
URL = "https://github.com/GoldenPigeon123"

# -------------------------- 基础依赖库配置 --------------------------
REQUIRES = [
    "pybind11>=2.6.0",
    "numpy>=1.18.0",
]

# -------------------------- 扩展模块配置 --------------------------
def get_extensions():
    # 定位.so文件（相对于setup.py所在目录，向上一级找cpp目录）
    so_path = Path(__file__).parent / "../cpp" / "build" / "bin" / "rs_lidar.cpython-310-x86_64-linux-gnu.so"
    so_path = so_path.resolve()  # 转换为绝对路径，避免相对路径解析错误
    
    # 验证文件存在性
    if not so_path.exists():
        raise FileNotFoundError(f"未找到编译好的扩展模块: {so_path}\n请检查路径是否正确或重新编译C++代码")

    # 扩展模块直接命名为"rs_lidar"（顶层模块）
    ext = Extension(
        name="rs_lidar",  # 关键：与pybind11定义的模块名一致
        sources=[],  # 已编译好的.so无需源文件
        library_dirs=[str(so_path.parent)],
        extra_link_args=[str(so_path)]
    )
    return [ext]

# -------------------------- 编译配置 --------------------------
class BuildExt(build_ext):
    def build_extension(self, ext):
        # 目标路径：生成的模块文件直接为rs_lidar.so（或对应平台的扩展格式）
        dest_path = self.get_ext_fullpath(ext.name)  # ext.name为"rs_lidar"
        dest_dir = os.path.dirname(dest_path)
        
        # 创建目标目录（确保安装路径存在）
        os.makedirs(dest_dir, exist_ok=True)
        
        # 复制.so文件到安装路径
        so_path = Path(__file__).parent / "../cpp" / "build" / "bin" / "rs_lidar.cpython-310-x86_64-linux-gnu.so"
        so_path = so_path.resolve()
        
        self.copy_file(str(so_path), dest_path)
        self.announce(f"已复制扩展模块到: {dest_path}", level=3)

# -------------------------- 核心setup配置 --------------------------
setup(
    name=PROJECT_NAME,
    version=__version__,
    author=AUTHOR,
    author_email=AUTHOR_EMAIL,
    description=DESCRIPTION,
    url=URL,
    ext_modules=get_extensions(),
    cmdclass={"build_ext": BuildExt},
    install_requires=REQUIRES,
    python_requires=">=3.6, <=3.14",
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Robotics",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    keywords=["robosense", "lidar", "pointcloud", "numpy"],
)