from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os
import subprocess
from pathlib import Path

# -------------------------- 项目基础信息配置 --------------------------
__version__ = "0.3.1"
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
    "numpy>=1.18.0",    # 点云数据与Numpy数组转换的依赖库
]

# -------------------------- 获取PCL+VTK编译/链接参数（修复VTK依赖） --------------------------
def get_pcl_vtk_flags():
    include_dirs = [
        "/usr/local/include/pcl-1.15",
        "/usr/include/eigen3",
        "/usr/include/ni",
        "/usr/include/openni2",
        "/usr/include",
        "/usr/include/vtk-9.1"  # VTK头文件路径
    ]

    # PCL库列表：移除-lpcl_console
    pcl_libs = [
        "-lpcl_common",
        "-lpcl_kdtree",
        "-lpcl_octree",
        "-lpcl_search",
        "-lpcl_sample_consensus",
        "-lpcl_filters",
        "-lpcl_io_ply",
        "-lpcl_io",
        "-lpcl_features",
        "-lpcl_ml",
        "-lpcl_segmentation",
        "-lpcl_visualization",
        "-lpcl_surface",
        "-lpcl_registration",
        "-lpcl_keypoints",
        "-lpcl_tracking",
        "-lpcl_recognition",
        "-lpcl_stereo",
        "-lpcl_apps",
        "-lpcl_outofcore",
        "-lpcl_people"  # 移除了-lpcl_console
    ]

    # VTK库和其他依赖不变
    vtk_libs = [
        "-lvtkChartsCore-9.1",
        "-lvtkCommonColor-9.1",
        "-lvtkCommonComputationalGeometry-9.1",
        "-lvtkCommonCore-9.1",
        "-lvtkCommonDataModel-9.1",
        "-lvtkCommonExecutionModel-9.1",
        "-lvtkCommonMath-9.1",
        "-lvtkCommonMisc-9.1",
        "-lvtkCommonTransforms-9.1",
        "-lvtkFiltersCore-9.1",
        "-lvtkFiltersExtraction-9.1",
        "-lvtkFiltersGeneral-9.1",
        "-lvtkFiltersGeometry-9.1",
        "-lvtkFiltersModeling-9.1",
        "-lvtkFiltersSources-9.1",
        "-lvtkImagingCore-9.1",
        "-lvtkImagingSources-9.1",
        "-lvtkInteractionImage-9.1",
        "-lvtkInteractionStyle-9.1",
        "-lvtkInteractionWidgets-9.1",
        "-lvtkIOCore-9.1",
        "-lvtkIOGeometry-9.1",
        "-lvtkIOImage-9.1",
        "-lvtkIOLegacy-9.1",
        "-lvtkIOPLY-9.1",
        "-lvtkRenderingAnnotation-9.1",
        "-lvtkRenderingCore-9.1",
        "-lvtkRenderingContext2D-9.1",
        "-lvtkRenderingLOD-9.1",
        "-lvtkRenderingFreeType-9.1",
        "-lvtkViewsCore-9.1",
        "-lvtkViewsContext2D-9.1",
        "-lvtkRenderingOpenGL2-9.1",
        "-lvtkRenderingContextOpenGL2-9.1",
        "-lvtkGUISupportQt-9.1"
    ]

    other_libs = [
        "-lboost_system", "-lboost_iostreams", "-lboost_filesystem", "-lboost_serialization",
        "-lOpenNI", "-lOpenNI2", "-lusb-1.0", "-lflann_cpp", "-lqhull", "-lGL", "-lglut"
    ]

    link_dirs = ["-L/usr/local/lib", "-L/usr/lib/x86_64-linux-gnu"]

    extra_compile_args = [
        "-I" + dir for dir in include_dirs
    ] + [
        "-std=c++17", "-DEIGEN_MAX_ALIGN_BYTES=32", "-Wno-deprecated-declarations", "-fPIC"
    ]

    extra_link_args = link_dirs + pcl_libs + vtk_libs + other_libs

    return include_dirs, extra_compile_args, extra_link_args
# -------------------------- 扩展模块配置 --------------------------
def get_extensions():
    # 基础路径配置
    rs_driver_include = os.getenv("RS_DRIVER_INCLUDE", "/usr/local/rs_driver/include/")
    rs_type_include = os.path.join(os.path.dirname(__file__), "./../src/rs_type/include")
    rs_reader_include = os.path.join(os.path.dirname(__file__), "./../src/rs_reader/include")
    rs_viewer_include = os.path.join(os.path.dirname(__file__), "./../src/rs_viewer/include")

    # 源文件列表
    sources = [
        os.path.join(os.path.dirname(__file__), "./../src/rs_reader/src/LidarReader.cpp"),
        os.path.join(os.path.dirname(__file__), "./../src/rs_viewer/src/LidarViewer.cpp"),
        os.path.join(os.path.dirname(__file__), "rs_bindings.cpp"),
    ]

    # 基础编译/链接选项
    extra_compile_args = [
        "-std=c++17", "-g", "-ltbb", "-O3",
        "-DUSE_PCL_POINT_TYPE", "-DPRINT_MSG", "-DRS_TIME_RECORD_PY",
        f"-I{rs_driver_include}",
    ]
    
    # PCL特定编译选项
    if "-DUSE_PCL_POINT_TYPE" in extra_compile_args: 
        extra_compile_args.extend([
            "-DEIGEN_MAX_ALIGN_BYTES=32",  # 消除Eigen对齐警告
            "-fPIC", "-Wno-deprecated-declarations"  # PCL兼容性选项
        ])

    extra_link_args = ["-lpthread", "-lpcap"]
    if "-DEXECUTIONLIB" in extra_compile_args:
        extra_link_args.append("-ltbb")

    # 注入PCL+VTK依赖
    if "-DUSE_PCL_POINT_TYPE" in extra_compile_args:
        pcl_vtk_include, pcl_vtk_compile, pcl_vtk_link = get_pcl_vtk_flags()
        # 合并路径和参数（去重）
        include_dirs_base = list(set([
            rs_type_include, rs_reader_include, rs_driver_include, rs_viewer_include,
            *[p for p in os.getenv("PYTHONPATH", "").split(":") if p],
            *pcl_vtk_include
        ]))
        extra_compile_args = list(set(extra_compile_args + pcl_vtk_compile))
        extra_link_args = list(set(extra_link_args + pcl_vtk_link))
    else:
        include_dirs_base = [
            rs_type_include, rs_reader_include, rs_driver_include, rs_viewer_include,
            *[p for p in os.getenv("PYTHONPATH", "").split(":") if p],
        ]

    # 定义扩展模块
    ext = Extension(
        name=PROJECT_NAME,
        sources=sources,
        include_dirs=include_dirs_base,
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
        language="c++",
    )

    return [ext]

# -------------------------- 自定义编译类 --------------------------
class BuildExt(build_ext):
    def build_extensions(self):
        ct = self.compiler.compiler_type
        if ct == "unix":
            for ext in self.extensions:
                if "-fvisibility=hidden" not in ext.extra_compile_args:
                    ext.extra_compile_args.append("-fvisibility=hidden")
            print(f"[INFO] Unix编译器（{ct}）已添加符号隐藏优化")
        super().build_extensions()

# -------------------------- 核心setup配置 --------------------------
setup(
    name=PROJECT_NAME,
    version=__version__,
    author=AUTHOR,
    author_email=AUTHOR_EMAIL,
    description=DESCRIPTION,
    long_description=long_description,
    long_description_content_type="text/markdown",
    url=URL,
    license="BSD-3-Clause",
    license_files=["LICENSE"],
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
        "Programming Language :: Python :: 3.13",
        "Programming Language :: Python :: 3.14",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Robotics",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Intended Audience :: Developers",
    ],
    keywords=["robosense", "lidar", "pointcloud", "pcl", "numpy", "jetson"],
)