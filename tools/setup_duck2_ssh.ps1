[CmdletBinding()]
param(
    [string]$RobotHost = "duck2.local",
    [string]$RobotUser = "duckie"
)

$ErrorActionPreference = "Stop"
if ($RobotHost -notmatch '^[A-Za-z0-9][A-Za-z0-9.-]*$' -or $RobotUser -notmatch '^[A-Za-z_][A-Za-z0-9_-]*$') {
    throw "Robot host or user contains unsupported characters."
}
$sshDirectory = Join-Path $env:USERPROFILE ".ssh"
$keyPath = Join-Path $sshDirectory "duck2_ed25519"
$knownHostsPath = Join-Path $sshDirectory "known_hosts"
$configPath = Join-Path $sshDirectory "config"
$alias = "duck2"
$hostKeyAlias = "duck2-hotspot"

New-Item -ItemType Directory -Path $sshDirectory -Force | Out-Null
if (-not (Test-Path -LiteralPath $keyPath)) {
    Write-Host "Create a dedicated protected key now. Enter a passphrase when OpenSSH asks."
    & ssh-keygen.exe -t ed25519 -f $keyPath -C "duck2 Windows SSH key"
    if ($LASTEXITCODE -ne 0) { throw "ssh-keygen did not create the duck2 key." }
}

$addresses = [System.Net.Dns]::GetHostAddresses($RobotHost) |
    Where-Object { $_.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork } |
    ForEach-Object { $_.IPAddressToString } | Select-Object -Unique
if (-not $addresses) { throw "No IPv4 address resolved for $RobotHost. Start the hotspot and wait for duck2." }

$knownLines = @()
foreach ($address in $addresses) {
    $knownLines += & ssh-keygen.exe -F $address -f $knownHostsPath 2>$null |
        Where-Object { $_ -notmatch '^#' -and $_.Trim() }
}
if (-not $knownLines) {
    throw "No previously trusted host key exists for the current duck2 address. Do not bypass verification. Verify and save its key first."
}

$existing = if (Test-Path -LiteralPath $knownHostsPath) { Get-Content -LiteralPath $knownHostsPath } else { @() }
foreach ($line in $knownLines) {
    $parts = $line -split '\s+', 3
    $aliasLine = "$hostKeyAlias $($parts[1]) $($parts[2])"
    if ($existing -notcontains $aliasLine) {
        Add-Content -LiteralPath $knownHostsPath -Value $aliasLine
        $existing += $aliasLine
    }
}

$configBlock = @"
# BEGIN duck2 managed configuration
Host $alias
    HostName $RobotHost
    User $RobotUser
    IdentityFile $keyPath
    IdentitiesOnly yes
    StrictHostKeyChecking yes
    HostKeyAlias $hostKeyAlias
    UserKnownHostsFile $knownHostsPath
    ConnectTimeout 5
    NumberOfPasswordPrompts 0
# END duck2 managed configuration
"@
$oldConfig = if (Test-Path -LiteralPath $configPath) { Get-Content -LiteralPath $configPath -Raw } else { "" }
$newConfig = [regex]::Replace($oldConfig, '(?ms)^# BEGIN duck2 managed configuration.*?^# END duck2 managed configuration\s*', '')
$newline = [Environment]::NewLine
Set-Content -LiteralPath $configPath -Value (($newConfig.TrimEnd() + $newline + $newline + $configBlock).Trim() + $newline) -NoNewline

$agent = Get-Service ssh-agent -ErrorAction SilentlyContinue
if ($agent -and $agent.Status -ne "Running") {
    try { Start-Service ssh-agent } catch { Write-Warning "Start the OpenSSH Authentication Agent once in an elevated PowerShell, then rerun this script."; throw }
}
& ssh-add.exe $keyPath
if ($LASTEXITCODE -ne 0) { throw "ssh-add could not unlock the new key." }

$publicKey = Get-Content -LiteralPath ($keyPath + ".pub") -Raw
$installCommand = "umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; grep -qxF '$($publicKey.Trim())' ~/.ssh/authorized_keys || printf '%s\n' '$($publicKey.Trim())' >> ~/.ssh/authorized_keys"
Write-Host "Enter the existing duck2 account password once to authorize the new key."
& ssh.exe -o StrictHostKeyChecking=yes -o HostKeyAlias=$hostKeyAlias -o UserKnownHostsFile=$knownHostsPath "$RobotUser@$RobotHost" $installCommand
if ($LASTEXITCODE -ne 0) { throw "The public key was not installed. Existing robot access was left unchanged." }

& ssh.exe $alias "echo key-authenticated; hostname"
if ($LASTEXITCODE -ne 0) { throw "Key authentication did not verify. Existing password access remains available." }
Write-Host "duck2 SSH setup complete. Future read-only checks: tools\Start-Duck2-Check.cmd"
