# Script para gerar binário unificado (Single Binary) para gravação via Flash Download Tool
param(
    [string]$OutputFile = "build\blue_mechanic_unified.bin"
)

# Localiza interpretador Python com esptool disponível
$Python = ""
if (Test-Path "C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe") {
    $Python = "C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe"
} elseif (Test-Path ".\.venv\Scripts\python.exe") {
    $Python = ".\.venv\Scripts\python.exe"
} else {
    $Python = "python"
}

Write-Host "Mesclando bootloader, partition-table e firmware em arquivo unico..." -ForegroundColor Yellow

# Verifica se existe partição OTA personalizada ou padrão
if (Test-Path "build\ota_data_initial.bin") {
    & $Python -m esptool --chip esp32s3 merge_bin -o $OutputFile --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x49000 build\ota_data_initial.bin 0x50000 build\blue_mechanic_v1.bin
} else {
    & $Python -m esptool --chip esp32s3 merge_bin -o $OutputFile --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\blue_mechanic_v1.bin
}

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[OK] Binário unificado gerado com sucesso em: $OutputFile" -ForegroundColor Green
    Write-Host "Endereço para gravação na Flash Download Tool: 0x0" -ForegroundColor Cyan
} else {
    Write-Host "`n[ERRO] Falha ao gerar binário unificado." -ForegroundColor Red
}
