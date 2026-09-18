# Script para gerar binário unificado (Single Binary) para gravação via Flash Download Tool
param(
    [string]$OutputFile = "build\blue_mechanic_unified.bin"
)

$PioPkg = "$env:USERPROFILE\.platformio\packages\tool-esptoolpy"
$env:PYTHONPATH = $PioPkg
$Python = ".\.venv\Scripts\python.exe"
$Esptool = "$PioPkg\esptool.py"

Write-Host "Mesclando bootloader, partition-table e firmware em arquivo unico..." -ForegroundColor Yellow

& $Python $Esptool --chip esp32s3 merge_bin -o $OutputFile --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x49000 build\ota_data_initial.bin 0x50000 build\blue_mechanic_v1.bin

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[OK] Binário unificado gerado com sucesso em: $OutputFile" -ForegroundColor Green
    Write-Host "Endereço para gravação: 0x0" -ForegroundColor Cyan
} else {
    Write-Host "`n[ERRO] Falha ao gerar binário unificado." -ForegroundColor Red
}
