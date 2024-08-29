$directoryPath = 'D:\FtcRobotController-9.0.1\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\'
$filePattern = '*.java'

$files = Get-ChildItem -Path $directoryPath -Filter $filePattern -Recurse
$totalLines = 0

foreach ($file in $files) {
    $lines = Get-Content -Path $file.FullName -Encoding UTF8
    $totalLines += $lines.Count
}

Write-Host "java文件总行数: $totalLines"