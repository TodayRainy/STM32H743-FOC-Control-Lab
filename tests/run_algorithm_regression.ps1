param(
    [string]$Compiler = "g++"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$outputDir = Join-Path $repoRoot "tmp"
$outputRelative = "tmp/foc_algorithm_regression.exe"
$output = Join-Path $repoRoot $outputRelative

if (-not (Get-Command $Compiler -ErrorAction SilentlyContinue)) {
    throw "C++ compiler not found: $Compiler"
}

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$compilerArgs = @(
    "-std=c++17",
    "-O2",
    "-Wall",
    "-Wextra",
    "-Wno-unknown-pragmas",
    "-Wno-missing-field-initializers",
    "-Itests/stubs",
    "-Ialgorithm/foc/core",
    "-Ialgorithm/foc/modulation",
    "-Ialgorithm/foc/control",
    "-Ialgorithm/foc/ident",
    "-Ialgorithm/foc/sensor",
    "tests/foc_algorithm_regression.cpp",
    "algorithm/foc/core/foc_math.cpp",
    "algorithm/foc/core/foc_pid.cpp",
    "algorithm/foc/modulation/foc_pwm.cpp",
    "algorithm/foc/control/foc_ctrl.cpp",
    "algorithm/foc/control/foc_speed.cpp",
    "-o",
    $outputRelative
)

Push-Location $repoRoot
try {
    & $Compiler $compilerArgs
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    & $output
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
