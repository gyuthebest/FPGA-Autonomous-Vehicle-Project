$ErrorActionPreference = 'Stop'

$carlaExe = 'C:\Users\jiho0\CARLA_0.9.16\CarlaUE4.exe'
$pythonExe = 'C:\Users\jiho0\AppData\Local\Programs\Python\Python312\python.exe'
$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not (Get-Process -Name 'CarlaUE4' -ErrorAction SilentlyContinue)) {
    Start-Process -FilePath $carlaExe -ArgumentList '-vulkan', '-quality-level=Low'
}

$deadline = [DateTime]::UtcNow.AddMinutes(3)
do {
    Start-Sleep -Seconds 2
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $connect = $client.ConnectAsync('127.0.0.1', 2000)
        $ready = $connect.Wait(1000) -and $client.Connected
    }
    catch {
        $ready = $false
    }
    finally {
        $client.Dispose()
    }
} until ($ready -or [DateTime]::UtcNow -ge $deadline)

if (-not $ready) {
    throw 'CARLA server did not open TCP port 2000 within 3 minutes.'
}

$env:FPGA_ENABLED = '0'
$env:PL_VERIFY_LOG = '1'
$env:PL_VERIFY_SAMPLE_RATE_HZ = '20'
$env:PL_VERIFY_FLUSH_EVERY = '1'
$env:CARLA_MAP = 'Town04'
Set-Location -LiteralPath $projectDir
& $pythonExe (Join-Path $projectDir 'main.py')
