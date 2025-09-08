# Copyright 2025 HPMicro
#
# SPDX-License-Identifier: BSD-3-Clause

import subprocess
import re
import os
import sys

# Python 2.7 compatibility
if sys.version_info[0] == 2:
    # Python 2.7 compatibility imports
    try:
        from subprocess import check_output
    except ImportError:
        check_output = None
else:
    # Python 3+
    from subprocess import check_output


def run_subprocess(args, timeout=None):
    """
    Run subprocess with Python 2.7 and 3+ compatibility.

    Args:
        args (list): Command and arguments
        timeout (int): Timeout in seconds (Python 3+ only)

    Returns:
        tuple: (returncode, stdout, stderr)
    """
    if sys.version_info[0] == 2:
        # Python 2.7 compatibility
        try:
            result = subprocess.Popen(
                args, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            stdout, stderr = result.communicate()
            return result.returncode, stdout, stderr
        except Exception:
            return 1, "", ""
    else:
        # Python 3+
        try:
            result = subprocess.run(
                args, capture_output=True, text=True, timeout=timeout
            )
            return result.returncode, result.stdout, result.stderr
        except Exception:
            return 1, "", ""


def extract_info_from_verbose(compiler_path):
    """
    Extract default architecture, ABI, and multilib information from --verbose output.

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        dict: Dictionary containing extracted information
    """
    info = {
        "default_arch": None,
        "default_abi": None,
        "multilib_generator": None,
        "target": None,
        "configured_with": None,
        "isa_spec": None,
        "tune": None,
        "pkgversion": None,
        "is_nds32": False,
        "architecture_family": None,
    }

    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--verbose"], timeout=15
        )
        verbose_output = stderr

        if returncode == 0:  # Verbose info is in stderr

            # Extract target
            target_match = re.search(r"Target:\s+(\S+)", verbose_output)
            if target_match:
                info["target"] = target_match.group(1)

                # Check if this is an NDS32 toolchain
                target = target_match.group(1).lower()
                if "nds32" in target or "andes" in target:
                    info["is_nds32"] = True
                    info["architecture_family"] = "riscv"
                elif "riscv" in target:
                    info["architecture_family"] = "riscv"
                elif "arm" in target:
                    info["architecture_family"] = "arm"
                elif "x86" in target or "i386" in target or "i686" in target:
                    info["architecture_family"] = "x86"
                elif "aarch64" in target:
                    info["architecture_family"] = "aarch64"
                else:
                    info["architecture_family"] = "unknown"

            # Extract configured with line
            configured_match = re.search(r"Configured with:\s+(.+)", verbose_output)
            if configured_match:
                info["configured_with"] = configured_match.group(1)

                # Extract default architecture from --with-arch
                arch_match = re.search(
                    r"--with-arch=([^\s]+)", configured_match.group(1)
                )
                if arch_match:
                    info["default_arch"] = arch_match.group(1)

                # Extract default ABI from --with-abi
                abi_match = re.search(r"--with-abi=([^\s]+)", configured_match.group(1))
                if abi_match:
                    info["default_abi"] = abi_match.group(1)

                # Extract multilib generator
                multilib_match = re.search(
                    r"--with-multilib-generator=\'([^\']+)\'", configured_match.group(1)
                )
                if multilib_match:
                    info["multilib_generator"] = multilib_match.group(1)

                # Extract ISA specification
                isa_spec_match = re.search(
                    r"--with-isa-spec=([^\s]+)", configured_match.group(1)
                )
                if isa_spec_match:
                    info["isa_spec"] = isa_spec_match.group(1)

                # Extract tune parameter
                tune_match = re.search(
                    r"--with-tune=([^\s]+)", configured_match.group(1)
                )
                if tune_match:
                    info["tune"] = tune_match.group(1)

                # Extract package version
                pkgversion_match = re.search(
                    r"--with-pkgversion=([^\s]+)", configured_match.group(1)
                )
                if pkgversion_match:
                    info["pkgversion"] = pkgversion_match.group(1)

    except Exception:
        pass
    return info


def parse_multilib_generator(multilib_generator_str):
    """
    Parse the multilib generator string to extract individual configurations.

    Args:
        multilib_generator_str (str): Multilib generator string from verbose output

    Returns:
        list: List of dictionaries containing multilib information
    """
    multilibs = []

    if not multilib_generator_str:
        return multilibs

    # Split by semicolon and parse each configuration
    configs = multilib_generator_str.split(";")
    for config in configs:
        config = config.strip()
        if not config:
            continue

        # Format: "rv32i-ilp32--" or "rv32imafc-ilp32f--"
        parts = config.split("-")
        if len(parts) >= 2:
            arch = parts[0]
            abi = parts[1]

            # Skip empty configurations (ending with --)
            if abi and not abi.endswith("--"):
                # Check for B extension support
                has_b_extension = (
                    "b" in arch.lower()
                    or "zbb" in arch.lower()
                    or "zbs" in arch.lower()
                    or "zba" in arch.lower()
                )

                # Parse architecture extensions
                extensions = parse_architecture_extensions(arch)

                multilib_info = {
                    "path": "{}/{}".format(arch, abi),
                    "abi": abi,
                    "arch": arch,
                    "flags": "@march={}@mabi={}".format(arch, abi),
                    "has_b_extension": has_b_extension,
                    "extensions": extensions,
                }
                multilibs.append(multilib_info)

    return multilibs


def parse_architecture_extensions(arch_string):
    """
    Parse RISC-V architecture string to extract individual extensions.

    Args:
        arch_string (str): Architecture string like 'rv32imafdc' or 'rv32imafdcb'

    Returns:
        dict: Dictionary containing extension information
    """
    extensions = {
        "base": None,
        "m": False,  # Integer Multiplication and Division
        "a": False,  # Atomic Instructions
        "f": False,  # Single-Precision Floating-Point
        "d": False,  # Double-Precision Floating-Point
        "c": False,  # Compressed Instructions
        "b": False,  # Bit Manipulation (B extension)
        "v": False,  # Vector Extension
        "k": False,  # Cryptography Extension
        "h": False,  # Hypervisor Extension
        "j": False,  # Dynamically Translated Languages
        "p": False,  # Packed-SIMD Extension
        "q": False,  # Quad-Precision Floating-Point
        "l": False,  # Decimal Floating-Point
        "n": False,  # User-Level Interrupts
        "s": False,  # Supervisor Mode
        "u": False,  # User Mode
        "x": False,  # Non-Standard Extensions
        "zbb": False,  # Basic Bit Manipulation
        "zbs": False,  # Single-Bit Instructions
        "zba": False,  # Address Generation
        "zbc": False,  # Carry-less Multiplication
        "zbe": False,  # Extract and Deposit
        "zbf": False,  # Bit Field Place
        "zbm": False,  # Matrix Multiply
        "zbp": False,  # Permutation
        "zbr": False,  # CRC
        "zbt": False,  # Ternary
        "other": [],  # Other extensions
    }

    if not arch_string or not arch_string.startswith("rv"):
        return extensions

    # Extract base architecture (rv32, rv64, etc.)
    base_match = re.match(r"rv(\d+)", arch_string)
    if base_match:
        extensions["base"] = "rv{}".format(base_match.group(1))

    # Parse extensions
    ext_part = arch_string[4:] if len(arch_string) > 4 else ""  # Skip 'rv32' or 'rv64'

    for char in ext_part:
        if char in extensions:
            extensions[char] = True
        else:
            extensions["other"].append(char)

    # Check for Z extensions (sub-extensions of B)
    z_extensions = re.findall(r"z[a-z]+", arch_string)
    for z_ext in z_extensions:
        if z_ext in extensions:
            extensions[z_ext] = True
        else:
            extensions["other"].append(z_ext)

    return extensions


def check_b_extension_support(compiler_path):
    """
    Check if the compiler supports B extension (Bit Manipulation).

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        dict: Dictionary containing B extension support information
    """
    b_support = {
        "has_b_extension": False,
        "b_variants": [],
        "supported_b_extensions": [],
    }

    # Get verbose information
    verbose_info = extract_info_from_verbose(compiler_path)
    if verbose_info["multilib_generator"]:
        multilibs = parse_multilib_generator(verbose_info["multilib_generator"])

        for multilib in multilibs:
            if multilib.get("has_b_extension", False):
                b_support["has_b_extension"] = True
                b_support["b_variants"].append(multilib["arch"])

                # Check specific B sub-extensions
                extensions = multilib.get("extensions", {})
                for ext_name, ext_supported in extensions.items():
                    if (
                        ext_supported
                        and ext_name.startswith("z")
                        and ext_name
                        in [
                            "zbb",
                            "zbs",
                            "zba",
                            "zbc",
                            "zbe",
                            "zbf",
                            "zbm",
                            "zbp",
                            "zbr",
                            "zbt",
                        ]
                    ):
                        if ext_name not in b_support["supported_b_extensions"]:
                            b_support["supported_b_extensions"].append(ext_name)

    return b_support


def check_isa_spec_support(compiler_path):
    """
    Check ISA specification support and version.

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        dict: Dictionary containing ISA specification information
    """
    isa_info = {
        "isa_spec": None,
        "isa_spec_version": None,
        "is_isa_spec_20191213": False,
        "is_isa_spec_2p2": False,
        "tune": None,
        "pkgversion": None,
    }

    # Get verbose information
    verbose_info = extract_info_from_verbose(compiler_path)

    if verbose_info["isa_spec"]:
        isa_info["isa_spec"] = verbose_info["isa_spec"]
        isa_info["isa_spec_version"] = verbose_info["isa_spec"]

        # Check for specific ISA spec versions
        isa_spec = verbose_info["isa_spec"]
        isa_info["is_isa_spec_20191213"] = "20191213" in isa_spec
        isa_info["is_isa_spec_2p2"] = "2.2" in isa_spec

    if verbose_info["tune"]:
        isa_info["tune"] = verbose_info["tune"]

    if verbose_info["pkgversion"]:
        isa_info["pkgversion"] = verbose_info["pkgversion"]

    return isa_info


def check_nds32_toolchain(compiler_path):
    """
    Check if the compiler is an NDS32 (Andes Technology) toolchain by examining --print-multi-lib output.

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        dict: Dictionary containing NDS32 detection results
    """
    nds32_info = {
        "is_nds32": False,
        "architecture_family": None,
        "target": None,
        "compiler_name_nds32": False,
        "detection_method": None,
        "andes_found_in_multilib": False,
        "multilib_output": None,
    }

    # Check compiler executable name for NDS32 patterns
    compiler_name = os.path.basename(compiler_path).lower()
    nds32_patterns = [
        "nds32",
        "andes",
        "ae320p",
        "ae350p",
        "ae360p",
        "ae370p",
        "ae380p",
        "ae390p",
    ]

    for pattern in nds32_patterns:
        if pattern in compiler_name:
            nds32_info["is_nds32"] = True
            nds32_info["architecture_family"] = "riscv"
            nds32_info["compiler_name_nds32"] = True
            nds32_info["detection_method"] = "compiler_name"
            break

    # If not detected by name, check --print-multi-lib output for "andes"
    if not nds32_info["is_nds32"]:
        try:
            returncode, stdout, stderr = run_subprocess(
                [compiler_path, "--print-multi-lib"], timeout=10
            )
            if returncode == 0:
                multilib_output = stdout
                nds32_info["multilib_output"] = multilib_output

                # Search for "andes" in the multilib output (case-insensitive)
                if "andes" in multilib_output.lower():
                    nds32_info["is_nds32"] = True
                    nds32_info["architecture_family"] = "riscv"
                    nds32_info["andes_found_in_multilib"] = True
                    nds32_info["detection_method"] = "print_multi_lib_andes"

                    # Try to extract target information from verbose output
                    verbose_info = extract_info_from_verbose(compiler_path)
                    if verbose_info["target"]:
                        nds32_info["target"] = verbose_info["target"]
        except Exception:
            pass

    # Fallback: check verbose output if still not detected
    if not nds32_info["is_nds32"]:
        verbose_info = extract_info_from_verbose(compiler_path)
        if verbose_info["is_nds32"]:
            nds32_info["is_nds32"] = True
            nds32_info["architecture_family"] = verbose_info["architecture_family"]
            nds32_info["target"] = verbose_info["target"]
            if not nds32_info["detection_method"]:
                nds32_info["detection_method"] = "target_string"

    return nds32_info


def detect_default_arch(compiler_path):
    """
    Detect the default architecture for the given compiler.

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        str: Default architecture string (e.g., 'rv32i', 'rv32im', 'rv32imac')
    """
    # Primary method: Extract from --verbose output
    verbose_info = extract_info_from_verbose(compiler_path)
    if verbose_info["default_arch"]:
        return verbose_info["default_arch"]

    # Fallback: Try to get default architecture using --print-multi-arch
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--print-multi-arch"], timeout=10
        )
        if returncode == 0:
            archs = stdout.strip().split()
            if archs:
                return archs[0]  # First arch is typically the default
    except Exception:
        pass

    # Fallback: try to get arch from --print-multi-lib
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--print-multi-lib"], timeout=10
        )
        if returncode == 0:
            lines = stdout.strip().split("\n")
            for line in lines:
                if ";" in line:
                    flags_part = (
                        line.split(";")[1].strip() if len(line.split(";")) > 1 else ""
                    )
                    if flags_part:
                        # Look for -march=xxx or @march=xxx
                        arch_match = re.search(r"[-@]march=([^\s@]+)", flags_part)
                        if arch_match:
                            return arch_match.group(1)
    except Exception:
        pass

    # Final fallback: try to detect from compiler version or use common defaults
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--version"], timeout=10
        )
        if returncode == 0:
            version_output = stdout.lower()
            if "riscv" in version_output:
                # For RISC-V, common default is rv32i
                return "rv32i"
    except Exception:
        pass

    return "rv32i"  # Default fallback


def detect_default_abi(compiler_path):
    """
    Detect the default ABI for the given compiler.

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        str: Default ABI string (e.g., 'ilp32', 'ilp32f', 'ilp32d', 'lp64', 'lp64f', 'lp64d')
    """
    # Primary method: Extract from --verbose output
    verbose_info = extract_info_from_verbose(compiler_path)
    if verbose_info["default_abi"]:
        return verbose_info["default_abi"]

    # Fallback: Try to get default ABI using --print-multi-abi
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--print-multi-abi"], timeout=10
        )
        if returncode == 0:
            abis = stdout.strip().split()
            if abis:
                # Handle combined arch/abi format like "rv32i/ilp32"
                first_abi = abis[0]
                if "/" in first_abi:
                    return first_abi.split("/")[1]  # Extract ABI part
                return first_abi
    except Exception:
        pass

    # Fallback: try to get ABI from --print-multi-lib
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--print-multi-lib"], timeout=10
        )
        if returncode == 0:
            lines = stdout.strip().split("\n")
            for line in lines:
                if ";" in line:
                    flags_part = (
                        line.split(";")[1].strip() if len(line.split(";")) > 1 else ""
                    )
                    if flags_part:
                        # Look for -mabi=xxx or @mabi=xxx
                        abi_match = re.search(r"[-@]mabi=([^\s@]+)", flags_part)
                        if abi_match:
                            return abi_match.group(1)
    except Exception:
        pass

    # Final fallback: try to detect from compiler version or use common defaults
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--version"], timeout=10
        )
        if returncode == 0:
            version_output = stdout.lower()
            if "riscv" in version_output:
                # For RISC-V, common default is ilp32 for 32-bit
                return "ilp32"
    except Exception:
        pass

    return "ilp32"  # Default fallback


def detect_supported_abis(compiler_path):
    """
    Detect all supported ABIs for the given compiler.

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        list: List of supported ABI strings
    """
    abis = []

    # Primary method: Extract from --verbose output
    verbose_info = extract_info_from_verbose(compiler_path)
    if verbose_info["multilib_generator"]:
        multilibs = parse_multilib_generator(verbose_info["multilib_generator"])
        for multilib in multilibs:
            if multilib["abi"] and multilib["abi"] not in abis:
                abis.append(multilib["abi"])
        if abis:
            return abis

    # Fallback: Try to get all ABIs using --print-multi-abi
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--print-multi-abi"], timeout=10
        )
        if returncode == 0:
            raw_abis = stdout.strip().split()
            if raw_abis:
                # Handle combined arch/abi format like "rv32i/ilp32"
                for raw_abi in raw_abis:
                    if "/" in raw_abi:
                        abi_part = raw_abi.split("/")[1]  # Extract ABI part
                        if abi_part not in abis:
                            abis.append(abi_part)
                    else:
                        if raw_abi not in abis:
                            abis.append(raw_abi)
                if abis:
                    return abis
    except Exception:
        pass

    # Fallback: parse from --print-multi-lib
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--print-multi-lib"], timeout=10
        )
        if returncode == 0:
            lines = stdout.strip().split("\n")
            for line in lines:
                if ";" in line:
                    flags_part = (
                        line.split(";")[1].strip() if len(line.split(";")) > 1 else ""
                    )
                    if flags_part:
                        # Look for -mabi=xxx or @mabi=xxx
                        abi_match = re.search(r"[-@]mabi=([^\s@]+)", flags_part)
                        if abi_match:
                            abi = abi_match.group(1)
                            if abi not in abis:
                                abis.append(abi)
    except Exception:
        pass

    # If no ABIs detected, return common RISC-V ABIs
    if not abis:
        abis = ["ilp32", "ilp32f", "ilp32d", "lp64", "lp64f", "lp64d"]

    return abis


def detect_supported_multilibs(compiler_path):
    """
    Detect all supported multilib configurations for the given compiler.

    Args:
        compiler_path (str): Path to the compiler executable

    Returns:
        list: List of dictionaries containing multilib information
              Each dict has keys: 'path', 'abi', 'arch', 'flags'
    """
    multilibs = []

    # Primary method: Extract from --verbose output
    verbose_info = extract_info_from_verbose(compiler_path)
    if verbose_info["multilib_generator"]:
        multilibs = parse_multilib_generator(verbose_info["multilib_generator"])
        if multilibs:
            return multilibs

    # Fallback: Parse from --print-multi-lib
    try:
        returncode, stdout, stderr = run_subprocess(
            [compiler_path, "--print-multi-lib"], timeout=10
        )
        if returncode == 0:
            lines = stdout.strip().split("\n")
            for line in lines:
                if ";" in line:
                    parts = line.split(";")
                    path_part = parts[0].strip()
                    flags_part = parts[1].strip() if len(parts) > 1 else ""

                    # Parse flags to extract ABI and architecture
                    abi = None
                    arch = None

                    if flags_part:
                        # Look for -mabi=xxx or @mabi=xxx
                        abi_match = re.search(r"[-@]mabi=([^\s@]+)", flags_part)
                        if abi_match:
                            abi = abi_match.group(1)

                        # Look for -march=xxx or @march=xxx
                        arch_match = re.search(r"[-@]march=([^\s@]+)", flags_part)
                        if arch_match:
                            arch = arch_match.group(1)

                    # Check for B extension support
                    has_b_extension = (
                        "b" in arch.lower()
                        or "zbb" in arch.lower()
                        or "zbs" in arch.lower()
                        or "zba" in arch.lower()
                    )

                    # Parse architecture extensions
                    extensions = parse_architecture_extensions(arch)

                    multilib_info = {
                        "path": path_part if path_part != "." else "",
                        "abi": abi,
                        "arch": arch,
                        "flags": flags_part,
                        "has_b_extension": has_b_extension,
                        "extensions": extensions,
                    }
                    multilibs.append(multilib_info)
    except Exception:
        pass

    return multilibs


def get_compiler_info(platform="gcc", exec_path=None, cc="riscv32-unknown-elf-gcc"):
    """
    Get comprehensive compiler information including ABI and multilib support.

    Args:
        platform (str): Platform type (default: 'gcc')
        exec_path (str): Path to the compiler directory
        cc (str): Compiler executable name

    Returns:
        dict: Dictionary containing compiler information
    """
    info = {
        "default_arch": None,
        "default_abi": None,
        "supported_abis": [],
        "supported_multilibs": [],
        "compiler_path": None,
        "verbose_info": None,
        "b_extension_support": None,
        "isa_spec_support": None,
        "nds32_toolchain": None,
    }

    # Determine compiler path based on platform
    if platform == "gcc":
        if exec_path is None:
            exec_path = "/opt/riscv-gnu-gcc/bin"  # Default path

        compiler_path = os.path.join(exec_path, cc)
        if os.name == "nt":
            compiler_path += ".exe"

        if os.path.exists(compiler_path):
            info["compiler_path"] = compiler_path
            # Get verbose information first (used by other detection functions)
            info["verbose_info"] = extract_info_from_verbose(compiler_path)

            # Use the enhanced detection functions
            info["default_arch"] = detect_default_arch(compiler_path)
            info["default_abi"] = detect_default_abi(compiler_path)

            # Check if this is an NDS32 toolchain
            info["nds32_toolchain"] = check_nds32_toolchain(compiler_path)

            if not info["nds32_toolchain"]["is_nds32"]:
                info["supported_abis"] = detect_supported_abis(compiler_path)
                info["supported_multilibs"] = detect_supported_multilibs(compiler_path)
                # Check B extension support
                info["b_extension_support"] = check_b_extension_support(compiler_path)
                # Check ISA specification support
                info["isa_spec_support"] = check_isa_spec_support(compiler_path)

    return info


def find_gcc_executable(search_path):
    """
    Find GCC executable from the specified path.

    Args:
        search_path (str): Path to search for GCC executable

    Returns:
        str or None: Path to the GCC executable if found, None otherwise
    """
    if not search_path or not os.path.exists(search_path):
        return None

    # Common GCC executable names
    gcc_names = [
        "riscv32-unknown-elf-gcc",
        "riscv64-unknown-elf-gcc",
        "riscv-none-elf-gcc",
        "riscv32-elf-gcc",
    ]

    # Add .exe extension for Windows
    if os.name == "nt":
        gcc_names.extend([name + ".exe" for name in gcc_names])

    # Search in the specified directory
    for gcc_name in gcc_names:
        gcc_path = os.path.join(search_path, gcc_name)
        if os.path.isfile(gcc_path) and os.access(gcc_path, os.X_OK):
            return gcc_path

    # If not found in the directory, search in subdirectories
    try:
        for root, dirs, files in os.walk(search_path):
            for gcc_name in gcc_names:
                gcc_path = os.path.join(root, gcc_name)
                if os.path.isfile(gcc_path) and os.access(gcc_path, os.X_OK):
                    return gcc_path
    except (OSError, PermissionError):
        pass

    return None


def extract_triplet_from_gcc(gcc_path):
    """
    Extract the target triplet from GCC executable.

    Args:
        gcc_path (str): Path to the GCC executable

    Returns:
        str or None: Target triplet if found, None otherwise
    """
    if not gcc_path or not os.path.exists(gcc_path):
        return None

    try:
        # Method 1: Try to get target from --print-target
        returncode, stdout, stderr = run_subprocess(
            [gcc_path, "--print-target"], timeout=10
        )
        if returncode == 0:
            target = stdout.strip()
            if target:
                return target
    except Exception:
        pass

    try:
        # Method 2: Try to get target from --verbose output
        returncode, stdout, stderr = run_subprocess([gcc_path, "--verbose"], timeout=15)
        if returncode == 0:
            verbose_output = stderr
            target_match = re.search(r"Target:\s+(\S+)", verbose_output)
            if target_match:
                return target_match.group(1)
    except Exception:
        pass

    try:
        # Method 3: Try to extract from executable name
        gcc_basename = os.path.basename(gcc_path)
        # Remove .exe extension if present
        if gcc_basename.endswith(".exe"):
            gcc_basename = gcc_basename[:-4]

        # Common patterns for extracting triplet from name
        # riscv32-unknown-elf-gcc -> riscv32-unknown-elf
        # arm-none-eabi-gcc -> arm-none-eabi
        # x86_64-linux-gnu-gcc -> x86_64-linux-gnu
        if gcc_basename.endswith("-gcc"):
            triplet = gcc_basename[:-4]  # Remove '-gcc' suffix
            # Validate that it looks like a triplet (has at least 2 dashes)
            if triplet.count("-") >= 2:
                return triplet
    except Exception:
        pass

    return None


def find_gcc_and_extract_triplet(search_path):
    """
    Find GCC executable from specified path and extract its triplet.

    Args:
        search_path (str): Path to search for GCC executable

    Returns:
        dict: Dictionary containing 'gcc_path' and 'triplet' keys
              Both values will be None if not found
    """
    result = {"gcc_path": None, "triplet": None}

    # Find GCC executable
    gcc_path = find_gcc_executable(search_path)
    if gcc_path:
        result["gcc_path"] = gcc_path

        # Extract triplet
        triplet = extract_triplet_from_gcc(gcc_path)
        if triplet:
            result["triplet"] = triplet

    return result
