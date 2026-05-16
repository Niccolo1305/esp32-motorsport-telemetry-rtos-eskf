param(
    [ValidateSet("docs", "python", "atoms3", "atoms3r", "firmware", "bench", "can", "all")]
    [string]$Target = "docs",

    [string]$Python = "python",
    [string]$PlatformIO = "pio"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

function Invoke-External {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Label,

        [Parameter(Mandatory = $true)]
        [string]$FilePath,

        [Parameter(Mandatory = $true)]
        [string[]]$Arguments
    )

    Write-Host ""
    Write-Host "==> $Label"
    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Label failed with exit code $LASTEXITCODE"
    }
}

function Invoke-DocsValidation {
    Invoke-External "Git whitespace check (unstaged)" "git" @("diff", "--check", "--")
    Invoke-External "Git whitespace check (staged)" "git" @("diff", "--cached", "--check", "--")
}

function Get-PythonValidationFiles {
    $tracked = @(git ls-files -- "*.py")
    $untracked = @(git ls-files --others --exclude-standard -- "*.py")

    return @($tracked + $untracked) |
        Where-Object { $_ -match "^(tools|firmware|scripts)/" } |
        Sort-Object -Unique
}

function Invoke-PythonValidation {
    $files = @(Get-PythonValidationFiles)
    if ($files.Count -eq 0) {
        Write-Host ""
        Write-Host "==> Python syntax check"
        Write-Host "No Python files found under tools/, firmware/, or scripts/."
        return
    }

    $arguments = @("-m", "py_compile") + $files
    Invoke-External "Python syntax check ($($files.Count) files)" $Python $arguments
}

function Invoke-PlatformIOBuild {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Environment
    )

    Invoke-External "PlatformIO build: $Environment" $PlatformIO @("run", "-e", $Environment)
}

switch ($Target) {
    "docs" {
        Invoke-DocsValidation
    }
    "python" {
        Invoke-PythonValidation
    }
    "atoms3" {
        Invoke-PlatformIOBuild "m5stack-atoms3"
    }
    "atoms3r" {
        Invoke-PlatformIOBuild "m5stack-atoms3r"
    }
    "firmware" {
        Invoke-PlatformIOBuild "m5stack-atoms3"
        Invoke-PlatformIOBuild "m5stack-atoms3r"
    }
    "bench" {
        Invoke-PlatformIOBuild "atoms3_bench"
        Invoke-PlatformIOBuild "atoms3r_bench"
    }
    "can" {
        Invoke-PlatformIOBuild "can_sniffer"
        Invoke-PlatformIOBuild "can_obd2_validator"
    }
    "all" {
        Invoke-DocsValidation
        Invoke-PythonValidation
        Invoke-PlatformIOBuild "m5stack-atoms3"
        Invoke-PlatformIOBuild "m5stack-atoms3r"
    }
}

Write-Host ""
Write-Host "Validation target '$Target' completed."
