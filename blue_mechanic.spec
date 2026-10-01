# -*- mode: python ; coding: utf-8 -*-
import os
import sys
from PyInstaller.utils.hooks import collect_all

block_cipher = None

# Collect all resources and submodules for can and python_app
can_datas, can_binaries, can_hidden = collect_all('can')
python_app_datas, python_app_binaries, python_app_hidden = collect_all('python_app')

datas = can_datas + python_app_datas
binaries = can_binaries + python_app_binaries
hiddenimports = [
    'PyQt6',
    'PyQt6.QtCore',
    'PyQt6.QtGui',
    'PyQt6.QtWidgets',
    'serial',
    'serial.tools',
    'serial.tools.list_ports',
    'can',
    'can.interfaces',
    'can.interfaces.pcan',
    'can.interfaces.virtual',
    'can.interfaces.serial',
    'can.interfaces.slcan',
] + can_hidden + python_app_hidden

a = Analysis(
    ['python_app/app.py'],
    pathex=['.'],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='BlueMechanic_v2.0',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
