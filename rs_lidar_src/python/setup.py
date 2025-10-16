from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os
import subprocess
from pathlib import Path

# -------------------------- 项目基础信息配置 --------------------------
__version__ = "0.3.0"
AUTHOR = "Zhejiang University of Finance and Economics - Point Cloud Team: Zhang Zhongqian, Li Huaiyuan, Cao Yiyun 浙江财经大学信息技术与人工智能学院点云组:张忠谦,李怀苑,曹宜云"
AUTHOR_EMAIL = "2789632062@qq.com"
PROJECT_NAME = "rs_lidar"
DESCRIPTION = "基于RoboSense LiDAR SDK的Python封装库,支持在线/PCAP点云读取与Numpy转换"
URL = "https://github.com/GoldenPigeon123"

# -------------------------- 项目说明文档读取 --------------------------
long_description = ""
readme_path = Path(__file__).parent / "README.md"
if readme_path.exists():
    with open(readme_path, "r", encoding="utf-8") as f:
        long_description = f.read()

# -------------------------- 基础依赖库配置 --------------------------
REQUIRES = [
    "pybind11>=2.6.0",  # Python与C++绑定的核心库
    "numpy>=1.18.0",    # 点云数据与Numpy数组转换的依赖（适配Jetson默认版本）
]

# -------------------------- PCL库检测函数 --------------------------
def find_pcl_like_cmake():
    """
    等效CMake的：
    find_package(PCL REQUIRED)
    if(NOT PCL_FOUND) message(FATAL_ERROR ...) endif()
    作用：检测PCL是否存在，返回PCL编译参数（cflags）和链接参数（libs）
    """
    try:
        # 1. 调用pkg-config获取PCL编译参数（对应CMake的${PCL_DEFINITIONS}和PCL_INCLUDE_DIRS）
        # 包含PCL头文件路径、PCL自带宏定义（如-DPCL_NO_PRECOMPILE）
        pcl_cflags = subprocess.check_output(
            ["pkg-config", "--cflags", "pcl_common pcl_io pcl_point_types"],  # 核心PCL模块，与CMake依赖一致
            encoding="utf-8",
            stderr=subprocess.STDOUT  # 捕获错误输出，便于排查
        ).strip().split()

        # 2. 调用pkg-config获取PCL链接参数（对应CMake的${PCL_LIBRARIES}）
        pcl_libs = subprocess.check_output(
            ["pkg-config", "--libs", "pcl_common pcl_io pcl_point_types"],
            encoding="utf-8",
            stderr=subprocess.STDOUT
        ).strip().split()

        # 3. 验证PCL参数有效性（避免空参数导致编译失败）
        if not pcl_cflags or not pcl_libs:
            raise RuntimeError("PCL找到但参数为空，可能是PCL安装不完整")

        return pcl_cflags, pcl_libs

    except subprocess.CalledProcessError as e:
        # 对应CMake的NOT PCL_FOUND，抛出明确错误
        raise RuntimeError(
            f"PCL库未找到!请先安装PCL并确保pkg-config可检测到:\n"
            f"1. Ubuntu/Debian系统安装命令:sudo apt install libpcl-dev\n"
            f"2. 验证PCL是否被pkg-config识别:pkg-config --list-all | grep pcl\n"
            f"3. 若已安装但仍报错,需手动设置PKG_CONFIG_PATH(如:export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH)\n"
            f"错误详情：{e.output.strip()}"
        )
    except Exception as e:
        raise RuntimeError(f"PCL检测过程异常：{str(e)}")

# -------------------------- 扩展模块配置 --------------------------
def get_extensions():
    # 基础路径配置（适配RoboSense SDK和自定义类型头文件）
    rs_driver_include = os.getenv("RS_DRIVER_INCLUDE", "/usr/local/rs_driver/include/")  # 环境变量优先，默认系统路径
    rs_type_include = os.path.join(os.path.dirname(__file__), "./../src/rs_type/include")        # 自定义点类型头文件
    rs_reader_include = os.path.join(os.path.dirname(__file__), "./../src/rs_reader/include")    # LiDAR读取器头文件

    # 源文件列表
    sources = [
        os.path.join(os.path.dirname(__file__), "./../src/rs_reader/src/LidarReader.cpp"),  # C++核心读取逻辑
        os.path.join(os.path.dirname(__file__), "rs_bindings.cpp"),                # Python绑定代码
    ]

    # -------------------------- 基础编译/链接选项（无PCL时的默认配置）--------------------------
    extra_compile_args = [
        "-std=c++17",          # C++17
        "-g",                  # 保留调试信息（便于开发排查）
        "-ltbb",               # 并行计算依赖
        # -O0	无优化 编译快，保留完整调试信息 开发 / 调试阶段
        # -O1	低优化 编译快，保留部分调试信息 初步测试阶段
        # -O2	中优化 编译快，保留部分调试信息 生产环境 / 发布版本
        # -O3	高优化 编译慢，不保留调试信息 对性能要求极高的场景
        # -Os	优化代码大小，不保留调试信息 对性能要求极高的场景
        "-O3",                   # O3优化（用户可根据需求切换注释的优化等级）
        # "-DPRINT_DEBUG",           # 启用lidar调试日志打印（用户按需开启）
        "-DPRINT_MSG=ON",            # 启用lidar运行日志打印（用户按需开启）
        "-DPRINT_PARAMETER=OFF",     # 启用lidar参数配置打印（用户按需注释关闭）
        "-DRS_TIME_RECORD=ON",       # 代码耗时日志（用户按需注释关闭）
        "-DEXECUTIONLIB=OFF",        # include <execution>
        f"-I{rs_driver_include}",  # 引入RoboSense SDK driver头文件
    ]

    extra_link_args = [
        "-lpthread",  # 多线程依赖
        "-lpcap",     # PCAP解析依赖（在线/离线点云读取必需，不可注释）
        "-ltbb",      # 并行计算依赖
    ]

    # -------------------------- PCL启用分支（通过环境变量USE_PCL控制，默认禁用）--------------------------
    # 逻辑：用户设置USE_PCL=1/true/yes时启用PCL，否则禁用
    use_pcl = os.getenv("USE_PCL", "0").strip().lower() in ("1", "true", "yes")
    if use_pcl:
        print(f"[INFO] 已启用PCL点云类型，正在检测PCL依赖...")
        # 1. 模拟CMake的find_package(PCL REQUIRED)
        pcl_cflags, pcl_libs = find_pcl_like_cmake()

        # 2. 对应CMake的add_definitions(${PCL_DEFINITIONS})：添加PCL自带宏定义
        extra_compile_args.extend(pcl_cflags)

        # 3. 对应CMake的add_definitions(-DUSE_PCL_POINT_TYPE)：启用PCL点类型分支
        extra_compile_args.append("-DUSE_PCL_POINT_TYPE")  # 仅定义宏（无需赋值，对应#ifdef判断，启用PCL时不可注释）

        # 4. 对应CMake的add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)：消除Eigen对齐警告
        extra_compile_args.append("-DEIGEN_MAX_ALIGN_BYTES=32")  # 启用PCL时建议保留，避免编译警告

        # 5. 对应CMake的链接PCL库：添加PCL链接参数
        extra_link_args.extend(pcl_libs)

        print(f"[INFO] PCL依赖检测完成，已添加PCL编译/链接参数")
    else:
        print(f"[INFO] 默认禁用PCL点云类型，使用自定义点类型（无PCL依赖）")

    # -------------------------- 定义扩展模块（统一两种模式的输出）--------------------------
    ext = Extension(
        name=PROJECT_NAME,  # Python包名（import rs_lidar即可使用）
        sources=sources,
        include_dirs=[
            rs_type_include,       # 自定义点类型头文件路径
            rs_reader_include,     # LiDAR读取器头文件路径
            rs_driver_include,     # RoboSense SDK头文件路径
            *[p for p in os.getenv("PYTHONPATH", "").split(":") if p],  # 兼容Python头文件路径
        ],
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
        language="c++",    # 指定C++编译器（默认C编译器，导致C++特性不可用，不可修改）
    )

    return [ext]

# -------------------------- 自定义编译类（适配不同编译器，优化符号冲突）--------------------------
class BuildExt(build_ext):
    def build_extensions(self):
        # 编译器类型判断（仅处理Unix-like系统，如Linux）
        ct = self.compiler.compiler_type
        if ct == "unix":
            # 为所有扩展模块添加"-fvisibility=hidden"：隐藏内部符号，避免与其他库冲突（建议保留，减少符号污染）
            for ext in self.extensions:
                if "-fvisibility=hidden" not in ext.extra_compile_args:
                    ext.extra_compile_args.append("-fvisibility=hidden")
            print(f"[INFO] Unix编译器（{ct}）已添加符号隐藏优化")
        # Windows系统可后续扩展（如添加/MSVC选项）
        super().build_extensions()  # 调用父类方法完成编译（不可修改）

# -------------------------- 核心setup配置（完整声明项目信息，支持Python 3.6-3.12）--------------------------
setup(
    name=PROJECT_NAME,
    version=__version__,
    author=AUTHOR,
    author_email=AUTHOR_EMAIL,
    description=DESCRIPTION,
    long_description=long_description,
    long_description_content_type="text/markdown",  # README.md为Markdown格式（若用rst需修改为text/x-rst）
    url=URL,
    license="BSD-3-Clause",  # 与RoboSense SDK协议一致（必须保持合规，不可修改）
    license_files=["LICENSE"],  # 声明许可证文件（需确保项目根目录有LICENSE文件，不可删除）
    ext_modules=get_extensions(),  # 关联扩展模块（不可修改）
    cmdclass={"build_ext": BuildExt},  # 关联自定义编译类（不可修改）
    install_requires=REQUIRES,  # 自动安装基础依赖（不可删除，确保依赖自动拉取）
    python_requires=">=3.6, <=3.12",  # 明确支持Python 3.6-3.12（用户可根据实际兼容范围调整）
    classifiers=[  # 分类器（便于PyPI检索，标注项目特性，可按需补充）
        "Development Status :: 4 - Beta",  # 项目状态：测试版（稳定后可改Stable为"5 - Production/Stable"）
        "License :: OSI Approved :: BSD License",  # 许可证分类（BSD-3-Clause，不可修改）
        "Programming Language :: C++",  # 核心语言（不可修改）
        "Programming Language :: Python :: 3",  # 主语言（不可修改）
        # 完整列出支持的Python版本（3.6-3.12，可根据实际兼容情况增删）
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: POSIX :: Linux",  # 目标系统：Linux（适配Jetson，不可修改）
        "Topic :: Scientific/Engineering :: Robotics",  # 领域：机器人（可按需补充）
        "Topic :: Software Development :: Libraries :: Python Modules",  # 类型：Python库（不可修改）
        "Intended Audience :: Developers",  # 目标用户：开发者（不可修改）
    ],
    keywords=["robosense", "lidar", "pointcloud", "pcl", "numpy", "jetson"],  # 搜索关键词（可按需补充）
)