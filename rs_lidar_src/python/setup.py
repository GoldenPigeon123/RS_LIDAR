from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os
from pathlib import Path

# -------------------------- 项目基础信息配置 --------------------------
__version__ = "0.3.7"
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

# ----------------------------工具函数 -----------------------------
def find_so_file(bin_dir,so_name):
    """在指定目录查找符合 .so 文件"""
    
    # 查找所有符合 rs_lidar.*.so 的文件（*匹配任意字符）
    so_files = list(bin_dir.glob(f"{so_name}.*.so"))
    
    # 处理匹配结果
    if not so_files:
        raise FileNotFoundError(
            f"未在目录 {bin_dir} 中找到符合 '{so_name}.*.so' 模式的文件\n"
            "请检查C++代码是否已编译，或编译产物路径是否正确"
        )
    if len(so_files) > 1:
        raise RuntimeError(
            f"在目录 {bin_dir} 中找到多个匹配文件：{so_files}\n"
            "请确保仅存在一个编译产物（删除多余版本后重试）"
        )
    return so_files[0] 

# -------------------------- 扩展模块配置 --------------------------

ext_name = "rs_lidar"
bin_dir = Path(__file__).parent / "../cpp" / "build" / "bin"
bin_dir = bin_dir.resolve()  # 转换为绝对路径，避免相对路径问题
so_path = find_so_file(bin_dir,ext_name)

# -------------------------- 扩展模块配置 --------------------------
def get_extensions():
    
    # 扩展模块配置（保持不变）
    ext = Extension(
        name=ext_name,  # 与pybind11定义的模块名一致
        sources=[], 
        library_dirs=[str(so_path.parent)],
        extra_link_args=[str(so_path)]
    )
    return [ext]

# -------------------------- 编译配置 --------------------------
class BuildExt(build_ext):
    def build_extension(self, ext):

        dest_path = self.get_ext_fullpath(ext.name)
        dest_dir = os.path.dirname(dest_path)
        
        os.makedirs(dest_dir, exist_ok=True)
        
        # 复制文件到安装路径
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
    python_requires=">=3.6, <=3.12",
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