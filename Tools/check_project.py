"""检查 Keil 工程引用；可选使用 ARM GCC 对全部 C 文件做语法检查。"""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-gcc", help="arm-none-eabi-gcc 路径")
    parser.add_argument("--newlib-include", help="独立工具链的 newlib 头文件目录")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    project_dir = root / "MDK-ARM"
    tree = ET.parse(project_dir / "SkyClawH7.uvprojx")
    sources = []
    for entry in tree.findall(".//File"):
        source = (project_dir / entry.findtext("FilePath").replace("\\", "/")).resolve()
        if not source.is_file():
            raise SystemExit(f"缺少工程文件：{source}")
        if source.suffix == ".c":
            sources.append(source)
    expected = {root / "APL/mechanism" / name for name in
                ("mechanism.c", "mechanism_config.c", "mechanism_app.c")}
    if not expected.issubset(sources):
        raise SystemExit("机构源文件未全部加入 Keil 工程")
    includes = [(project_dir / item.replace("\\", "/")).resolve() for item in
                tree.find(".//Cads/VariousControls/IncludePath").text.split(";")]
    for directory in includes:
        if not directory.is_dir():
            raise SystemExit(f"缺少头文件目录：{directory}")
    ioc = (root / "SkyClawH7.ioc").read_text()
    if "ProjectManager.ProjectName=SkyClawH7" not in ioc:
        raise SystemExit("CubeMX 工程名称不一致")
    print(f"Keil 引用检查通过：{len(sources)} 个 C 文件")
    if args.arm_gcc:
        command = [args.arm_gcc, "-std=c11", "-mcpu=cortex-m7", "-mthumb",
                   "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-fsyntax-only",
                   "-DUSE_PWR_LDO_SUPPLY", "-DUSE_HAL_DRIVER", "-DSTM32H723xx"]
        for directory in includes:
            command.extend(["-I", str(directory)])
        if args.newlib_include:
            command.extend(["-isystem", args.newlib_include])
        # RVDS 的汇编接口由 Keil ARM Compiler 6 编译。
        checked = [source for source in sources if "portable/RVDS" not in source.as_posix()]
        subprocess.run(command + [str(source) for source in checked], check=True)
        print(f"ARM C 语法检查通过：{len(checked)} 个文件；RVDS port 由 Keil 验证")


if __name__ == "__main__":
    main()
