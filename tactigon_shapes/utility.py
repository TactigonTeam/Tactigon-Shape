import subprocess
import cpuinfo
import sys

def is_deep_speech_supported():
    info = cpuinfo.get_cpu_info()
    features = info.get('flags', [])

    if 'avx' in features or 'fma' in features:
        return True
    return False

def install_package(package_name: str, no_deps: bool = False):
    try:
        command = [sys.executable, "-m", "pip", "install", package_name]
        package_name_without_version = package_name.split("=", 1)[0]

        if no_deps:
            command.append("--no-deps")

        if not is_package_installed(package_name_without_version):
            subprocess.check_call(command)
            
    except subprocess.CalledProcessError as e:
        print(f"Failed to install package '{package_name}'. {e}")


def is_package_installed(package_name):
    try:
        subprocess.check_output(["pip", "show", package_name], stderr=subprocess.STDOUT)
        return True
    except subprocess.CalledProcessError:
        return False
    

def has_voice():
    return is_deep_speech_supported()