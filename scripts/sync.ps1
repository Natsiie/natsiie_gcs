<#
  联调同步仪表盘（Windows 侧）
  一条命令看清：origin/main 上多了什么、你欠什么、下一步该跑什么。

  用法：
    .\scripts\sync.ps1            # 先 fetch 再显示
    .\scripts\sync.ps1 -NoFetch   # 不联网，只看本地已知的 origin/main
#>
param([switch]$NoFetch)

$ErrorActionPreference = 'Continue'

function Say($text) { Write-Host $text }

if (-not (Test-Path '.git')) { Say "✗ 当前目录不是 git 仓库：$(Get-Location)"; exit 1 }

if (-not $NoFetch) {
  Say '== 正在 fetch origin ...'
  & git fetch --prune origin 2>&1 | ForEach-Object { Write-Host "   $_" }
}

$branch = (& git rev-parse --abbrev-ref HEAD 2>$null)
$short  = (& git rev-parse --short HEAD 2>$null)
$counts = (& git rev-list --left-right --count 'HEAD...origin/main' 2>$null) -split '\s+'
$ahead  = [int]$counts[0]
$behind = [int]$counts[1]

Say ''
Say "== 当前分支: $branch ($short)"
Say "   相对 origin/main:  领先 $ahead, 落后 $behind"

$dirty = (& git status --porcelain 2>$null | Measure-Object).Count
if ($dirty -gt 0) { Say "   ⚠ 工作区有 $dirty 处未提交改动（联调前先处理干净）" }

function Classify($files) {
  $tags = New-Object System.Collections.Generic.List[string]
  $proto = $false
  foreach ($f in $files) {
    if ($f -match '^(stm32_firmware/Devices/Serial\.(c|h)|src/identify_pack/identify_pack/serial/)') { $proto = $true }
  }
  if ($proto) { $tags.Add('协议') | Out-Null }
  if ($files -match '^stm32_firmware/')            { $tags.Add('固件') | Out-Null }
  if ($files -match '^(src/|udev/|scripts/)')      { $tags.Add('视觉') | Out-Null }
  if ($files -match '^(DEVLOG|PROTOCOL|README)')   { $tags.Add('文档') | Out-Null }
  if ($tags.Count -eq 0) { $tags.Add('其他') | Out-Null }
  return ($tags -join '+')
}

if ($behind -gt 0) {
  Say ''
  Say "== origin/main 上你还没拿到的 $behind 个提交（新 → 旧）"
  $log = & git log --pretty=format:'%h|%ad|%an|%s' --date=short 'HEAD..origin/main' 2>$null
  $seenProto = $false; $seenVision = $false
  foreach ($line in $log) {
    $p = $line -split '\|', 4
    $files = & git show --name-only --pretty=format: $p[0] 2>$null | Where-Object { $_ -ne '' }
    $tag = Classify $files
    if ($tag -match '协议') { $seenProto = $true }
    if ($tag -match '视觉') { $seenVision = $true }
    Say ("   [{0,-8}] {1} {2}  {3}" -f $tag, $p[0], $p[1], $p[3])
  }
  Say ''
  Say '== 建议'
  Say '   git pull --rebase origin main'
  if ($seenVision) { Say '   colcon build --packages-select identify_pack && source install/setup.bash   # 视觉侧有更新' }
  if ($seenProto)  { Say '   ⚠ 有协议相关改动 → 先看 PROTOCOL.md 是否同步，再烧录/重跑' }
}

if ($ahead -gt 0) {
  Say ''
  Say "== 你本地有 $ahead 个提交还没推上去"
  & git log --pretty=format:'   %h %ad %an  %s' --date=short 'origin/main..HEAD' 2>$null | ForEach-Object { Say $_ }
  Say '   git push origin HEAD'
}

if ($behind -eq 0 -and $ahead -eq 0) { Say ''; Say '== 已与 origin/main 一致' }
