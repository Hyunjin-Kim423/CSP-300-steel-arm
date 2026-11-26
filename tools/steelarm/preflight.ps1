# SteelArm preflight (Windows): checks the basics before the dashboard starts. Run by start_steelarm.bat.
#   1. WSL can start
#   2. usbipd is installed, and the robot arm (USB-serial chip 1a86:55d3) is plugged in and registered (bound)
#      -> not registered: asks, then registers it with one admin prompt (needed once per device)
#   3. adb sees the headset
# Exit code 0 = start the dashboard, 1 = quit.
# Attaching the arm to WSL is done later by the dashboard (it needs WSL running).

$ErrorActionPreference = 'Continue'
$ArmId = '1a86:55d3'          # CH343 USB-serial chip on the SO-101 board (the leader arm uses the same chip!)
$problems = @()

function Say($tag, $text, $color) {
    Write-Host ('  [{0}] ' -f $tag) -ForegroundColor $color -NoNewline
    Write-Host $text
}
function Ok($t)   { Say ' OK ' $t Green }
function Warn($t) { Say ' !! ' $t Yellow }
function Bad($t)  { Say 'FAIL' $t Red }

function Ask($question, $keys) {
    while ($true) {
        $k = (Read-Host "      $question").Trim().ToUpper()
        if ($k -eq '' -and $keys.Count -gt 0) { $k = $keys[0] }
        if ($keys -contains $k) { return $k }
    }
}

# Rows of "usbipd list" between "Connected:" and "Persisted:" -> objects (BusId, Id, Name, State)
function Get-UsbDevices {
    $out = & usbipd list 2>&1 | Out-String
    $rows = @()
    $inConnected = $false
    foreach ($line in ($out -split "`r?`n")) {
        if ($line -match '^Connected:') { $inConnected = $true; continue }
        if ($line -match '^Persisted:') { break }
        if ($inConnected -and $line -match '^(\S+)\s+([0-9a-fA-F]{4}:[0-9a-fA-F]{4})\s+(.+?)\s{2,}(Not shared|Shared.*|Attached.*)\s*$') {
            $rows += [pscustomobject]@{ BusId = $Matches[1]; Id = $Matches[2].ToLower(); Name = $Matches[3].Trim(); State = $Matches[4].Trim() }
        }
    }
    return $rows
}

Write-Host ''
Write-Host '  SteelArm preflight' -ForegroundColor Cyan
Write-Host '  ------------------------------------------------------------'

# ---------------------------------------------------------------- 1. WSL
& wsl.exe -e true 2>$null
if ($LASTEXITCODE -eq 0) { Ok 'WSL starts' }
else { Bad 'WSL does not start. Install it: wsl --install -d Ubuntu-24.04 (see README_allSetup.md)'; $problems += 'WSL' }

# ---------------------------------------------------------------- 2. robot arm (usbipd)
if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    Bad 'usbipd is not installed: winget install usbipd   (then open a new window)'
    $problems += 'usbipd'
} else {
    while ($true) {
        $arms = @(Get-UsbDevices | Where-Object { $_.Id -eq $ArmId })
        if ($arms.Count -eq 0) {
            Warn "Robot arm not found (no USB device $ArmId)."
            Write-Host '      Plug in the arm USB and turn the motor power on.'
            $k = Ask '[R] check again   [S] skip (no arm this time)   [Q] quit' @('R', 'S', 'Q')
            if ($k -eq 'Q') { exit 1 }
            if ($k -eq 'S') { $problems += 'arm'; break }
            continue
        }
        if ($arms.Count -gt 1) {
            Warn "$($arms.Count) devices with the arm's USB chip ($ArmId). The dashboard cannot tell them apart:"
            $arms | ForEach-Object { Write-Host ("        {0,-6} {1}  [{2}]" -f $_.BusId, $_.Name, $_.State) }
            Write-Host '      Unplug the ones that are not the robot arm (e.g. the leader arm).'
            $k = Ask '[R] check again   [S] continue anyway   [Q] quit' @('R', 'S', 'Q')
            if ($k -eq 'Q') { exit 1 }
            if ($k -eq 'S') { $problems += 'arm'; break }
            continue
        }
        $arm = $arms[0]
        if ($arm.State -like 'Not shared*') {
            Warn "Robot arm found (BUSID $($arm.BusId)) but not registered with usbipd yet (needed once)."
            $k = Ask 'Register it now? An admin prompt will appear.  [Y] yes   [N] no' @('Y', 'N')
            if ($k -eq 'N') { $problems += 'arm'; break }
            try {
                Start-Process -FilePath (Get-Command usbipd).Source -ArgumentList "bind --busid $($arm.BusId)" -Verb RunAs -Wait -WindowStyle Hidden
            } catch {
                Bad 'Registration was cancelled or failed.'
            }
            continue   # check the state again
        }
        Ok "Robot arm: BUSID $($arm.BusId), $($arm.State)  (the dashboard attaches it to WSL)"
        break
    }
}

# ---------------------------------------------------------------- 3. headset (adb)
if (-not (Get-Command adb -ErrorAction SilentlyContinue)) {
    Bad 'adb is not found: install Android platform-tools and add it to PATH'
    $problems += 'adb'
} else {
    while ($true) {
        $lines = (& adb devices 2>&1 | Out-String) -split "`r?`n" | Select-Object -Skip 1 | Where-Object { $_.Trim() }
        $ready = @($lines | Where-Object { $_ -match '\sdevice$' })
        $unauth = @($lines | Where-Object { $_ -match 'unauthorized' })
        if ($ready.Count -gt 0) { Ok "Headset: $(($ready[0] -split '\s+')[0])"; break }
        if ($unauth.Count -gt 0) { Warn 'Headset connected but USB debugging is not allowed: put it on and press Allow.' }
        else { Warn 'Headset not found: connect it with a USB data cable.' }
        $k = Ask '[R] check again   [S] skip (the dashboard keeps checking)   [Q] quit' @('R', 'S', 'Q')
        if ($k -eq 'Q') { exit 1 }
        if ($k -eq 'S') { $problems += 'headset'; break }
    }
}

# ---------------------------------------------------------------- result
Write-Host '  ------------------------------------------------------------'
if ($problems.Count -eq 0) {
    Write-Host '  All set. Starting the dashboard...' -ForegroundColor Green
    Start-Sleep -Seconds 1
    exit 0
}
Write-Host ('  Not ready: {0}' -f ($problems -join ', ')) -ForegroundColor Yellow
if ($problems -contains 'WSL') { Read-Host '  WSL is required. Press Enter to close'; exit 1 }
$k = Ask 'Start the dashboard anyway? (the app, YOLO and LOCK parts work without the arm)  [Y] yes   [N] no' @('Y', 'N')
if ($k -eq 'Y') { exit 0 } else { exit 1 }
