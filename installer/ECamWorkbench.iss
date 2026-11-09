; Electronic CAM Workbench: Windows installer (Inno Setup 6).
; Built by installer\build_installer.ps1, which stages the deployed app, the firmware and the documents in
; StageDir and passes the version from src\Brand.h. Output: dist\ECamWorkbench-<version>-setup.exe.

#define AppName "Electronic CAM Workbench"
#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif
#ifndef StageDir
  #define StageDir "..\ProjectData\build\installer_stage"
#endif

[Setup]
AppId={{BDBA138B-1C76-4E50-A881-3E424C1E5C1F}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher=Adam G. Sweeney
AppPublisherURL=https://github.com/AGSweeney/Electronic_CAM_Workbench
AppSupportURL=https://github.com/AGSweeney/Electronic_CAM_Workbench
VersionInfoVersion={#AppVersion}
DefaultDirName={autopf}\Electronic CAM Workbench
DefaultGroupName=Electronic CAM Workbench
DisableProgramGroupPage=yes
OutputDir=..\dist
OutputBaseFilename=ECamWorkbench-{#AppVersion}-setup
SetupIconFile=..\resources\icons\ecam_app_icon.ico
UninstallDisplayIcon={app}\ECamWorkbench.exe
UninstallDisplayName={#AppName} {#AppVersion}
LicenseFile=..\LICENSE
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=admin
PrivilegesRequiredOverridesAllowed=dialog
ChangesAssociations=yes
CloseApplications=yes

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked
Name: "associate"; Description: "Open .ecam projects with the {#AppName}"; GroupDescription: "File types:"

[Files]
; The app with its Qt and MSVC runtime (windeployqt output, test programs left out)
Source: "{#StageDir}\app\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
; ClearCore firmware, its flasher and the EtherNet/IP EDS file
Source: "{#StageDir}\firmware\*"; DestDir: "{app}\firmware"; Flags: ignoreversion recursesubdirs createallsubdirs
; Example projects
Source: "{#StageDir}\samples\*"; DestDir: "{app}\samples"; Flags: ignoreversion recursesubdirs createallsubdirs
; Documents and licences
Source: "{#StageDir}\docs\*"; DestDir: "{app}\docs"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\ECamWorkbench.exe"
Name: "{group}\Flash the ClearCore firmware (USB)"; Filename: "{app}\firmware\flash_firmware.cmd"; WorkingDir: "{app}\firmware"
Name: "{group}\Firmware and EtherNet/IP files"; Filename: "{app}\firmware"
Name: "{group}\Example projects"; Filename: "{app}\samples"
Name: "{group}\{cm:UninstallProgram,{#AppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\ECamWorkbench.exe"; Tasks: desktopicon

[Registry]
Root: HKA; Subkey: "Software\Classes\.ecam"; ValueType: string; ValueName: ""; ValueData: "ECamWorkbench.Project"; Flags: uninsdeletevalue; Tasks: associate
Root: HKA; Subkey: "Software\Classes\ECamWorkbench.Project"; ValueType: string; ValueName: ""; ValueData: "Electronic CAM project"; Flags: uninsdeletekey; Tasks: associate
Root: HKA; Subkey: "Software\Classes\ECamWorkbench.Project\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\ECamWorkbench.exe,0"; Tasks: associate
Root: HKA; Subkey: "Software\Classes\ECamWorkbench.Project\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\ECamWorkbench.exe"" ""%1"""; Tasks: associate

[Run]
Filename: "{app}\ECamWorkbench.exe"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
