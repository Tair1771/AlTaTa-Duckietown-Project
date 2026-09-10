# Setup and startup

Run commands from the repository root in Windows PowerShell. This project uses
duck2's existing installed ROS environment. Keep it stationary during setup.

## Laptop prerequisites

Install Windows OpenSSH Client and Python 3.10+ with Tcl/Tk and the Python
launcher. Install Pillow using the interpreter that runs the app:

```powershell
py -3 -m pip install -r requirements-desktop.txt
```

The shortcuts prefer an existing bundled Python on the development laptop;
otherwise they use `py -3`. No model account or API key is required. Docker
Desktop is optional for local image builds, not source-mounted robot startup.

## One-time trusted SSH setup

The robot account is `duckie`. Its password is separate from the hotspot,
Windows and laptop Ubuntu passwords. Obtain its account credentials and trusted
host fingerprint from the owner; keep them outside this repository.

Find duck2's current IP in the hotspot's connected-device list. Replace
`ROBOT_IP` in `ssh duckie@ROBOT_IP`, connect once and compare the displayed
fingerprint with the trusted fingerprint before accepting, then exit. The setup
script requires that trusted IP entry; it does not establish trust itself.
Never accept a changed identity without verification. In administrator
PowerShell, enable the agent:

```powershell
Set-Service -Name ssh-agent -StartupType Automatic
Start-Service ssh-agent
```

In normal PowerShell run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File tools/setup_duck2_ssh.ps1
```

Enter the key passphrase privately, then the robot account password when asked.
The script preserves existing authorized keys and creates the strict `duck2`
alias. Private material stays in the Windows user's `.ssh` directory. Do not
reinstall the key for each connection. If it needs unlocking after reboot:

```powershell
ssh-add "$env:USERPROFILE\.ssh\duck2_ed25519"
```

## Each startup

1. Enable the configured hotspot, power on duck2 and allow 2–5 minutes to boot.
2. Run `tools/Start-Duck2-Check.cmd` for read-only SSH, ROS and ownership checks.
3. While stationary, run `tools/Start-Duck2-DrivingMode.cmd`, then open
   `laptop/Start-Duck2Companion.cmd` from the same checkout.
4. Connect, start viewing, select a route and confirm placement. Only the
   operator's Start begins movement. See [USAGE](USAGE.md).

The temporary backend does not automatically restart after robot boot. Opening
the app or tunnel alone does not create it.

## Recovery

- **Name/network failure:** confirm hotspot, robot power and connection. An
  address change must still match the trusted SSH key.
- **Locked key:** unlock with `ssh-add`; do not repeatedly try the hotspot
  password at an SSH account-password prompt.
- **Camera timed out:** check the tunnel and backend. Camera uses local port
  8766 and control uses 8765.
- **Camera service reached but no fresh preview:** the backend responded but
  has no usable frame. Stop, close the app and prepare the updated backend.
- **Port occupied:** close the old project SSH tunnel before another preparation;
  leave unrelated SSH sessions alone.
- **Preparation refuses an existing session:** Stop and follow
  [cleanup](USAGE.md#finish-and-recover), then prepare again.

Reconnection never guesses the robot's lane or starts movement automatically.
