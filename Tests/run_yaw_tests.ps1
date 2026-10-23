param([string]$Compiler = 'D:/Dev-Cpp/MinGW64/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$testExe = Join-Path ([System.IO.Path]::GetTempPath()) ('yaw-' + [guid]::NewGuid().ToString('N') + '.exe')
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    foreach ($inputMode in @(1, 0)) {
        & $Compiler -std=c99 -Wall -Wextra -Werror -Wno-unused-parameter `
            "-DTEST_YAW_RC=$inputMode" -ITests/yaw_stubs -IApp -ISupport `
            Tests/chassis_yaw_test.c App/chassis_yaw.c App/chassis.c -o $testExe -lm
        if ($LASTEXITCODE -ne 0) { throw 'Yaw test compilation failed.' }
        & $testExe
        if ($LASTEXITCODE -ne 0) { throw 'Yaw tests failed.' }
    }
    & $Compiler -std=c99 -Wall -Wextra -Werror -Wno-unused-parameter `
        -ITests/yaw_stubs -IApp -ISupport Tests/chassis_switch_test.c `
        App/chassis.c App/chassis_yaw.c App/control.c Support/pid.c -o $testExe -lm
    if ($LASTEXITCODE -ne 0) { throw 'Chassis switch test compilation failed.' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Chassis switch tests failed.' }
} finally {
    Pop-Location
    if (Test-Path -LiteralPath $testExe) { Remove-Item -LiteralPath $testExe }
}
