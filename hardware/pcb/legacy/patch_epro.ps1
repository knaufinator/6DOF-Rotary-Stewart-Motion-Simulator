Add-Type -AssemblyName System.IO.Compression

$origPath = "c:\Users\Chris\Documents\GitHub\6DOF-Rotary-Stewart-Motion-Simulator\docs\hardware\ProPrj_6dof2_2026-02-28.epro"
$eproPath = "c:\Users\Chris\Documents\GitHub\6DOF-Rotary-Stewart-Motion-Simulator\docs\hardware\ProPrj_6dof2_v2.epro"
$workDir = "c:\Users\Chris\Documents\GitHub\6DOF-Rotary-Stewart-Motion-Simulator\docs\hardware\epro_work"

Copy-Item $origPath $eproPath -Force

$zip = [System.IO.Compression.ZipFile]::Open($eproPath, [System.IO.Compression.ZipArchiveMode]::Update)

$entries = @(
    @("project.json", "$workDir\project.json"),
    @("SHEET/84828f41ae134accae351f318ccf05d8/1.esch", "$workDir\SHEET\84828f41ae134accae351f318ccf05d8\1.esch"),
    @("SYMBOL/b66762e10a8c08e298a029b7d38a8941.esym", "$workDir\SYMBOL\b66762e10a8c08e298a029b7d38a8941.esym"),
    @("SYMBOL/e5cf02de32fad4a8ebb9effca8a488db.esym", "$workDir\SYMBOL\e5cf02de32fad4a8ebb9effca8a488db.esym"),
    @("FOOTPRINT/9450d6bedca6b6eedb6f7c7dab5a1063.efoo", "$workDir\FOOTPRINT\9450d6bedca6b6eedb6f7c7dab5a1063.efoo")
)

foreach ($pair in $entries) {
    $entryName = $pair[0]
    $filePath = $pair[1]
    $existing = $zip.GetEntry($entryName)
    if ($existing -ne $null) { $existing.Delete() }
    $entry = $zip.CreateEntry($entryName)
    $stream = $entry.Open()
    $writer = New-Object System.IO.StreamWriter($stream)
    $content = [System.IO.File]::ReadAllText($filePath)
    $writer.Write($content)
    $writer.Close()
    $stream.Close()
    Write-Host "Updated: $entryName ($($content.Length) chars)"
}

$zip.Dispose()
Write-Host "Done! Size: $((Get-Item $eproPath).Length) bytes"
