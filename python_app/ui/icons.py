"""
Ícones vetoriais (SVG em linha, traço 2 px, grade 24x24) desenhados para o app.

Substituem os emojis dos botões: renderizam iguais em qualquer Windows/fonte, ficam
nítidos em telas de alta densidade e seguem a cor do tema.
"""

from PyQt6.QtCore import QByteArray, QRectF, Qt
from PyQt6.QtGui import QIcon, QPainter, QPixmap
from PyQt6.QtSvg import QSvgRenderer

from .theme import PALETTE

_PATHS = {
    # Grade de painéis
    "dashboard": '<rect x="3" y="3" width="7" height="9" rx="1.5"/><rect x="14" y="3" width="7" height="5" rx="1.5"/>'
                 '<rect x="14" y="12" width="7" height="9" rx="1.5"/><rect x="3" y="16" width="7" height="5" rx="1.5"/>',
    # Controles deslizantes
    "sliders": '<path d="M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0"/>'
               '<circle cx="16" cy="6" r="2"/><circle cx="10" cy="12" r="2"/><circle cx="18" cy="18" r="2"/>',
    # Janela de terminal
    "terminal": '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M7 9l3 3-3 3M12 15h5"/>',
    # Setas em ciclo (automação)
    "repeat": '<path d="M17 2l4 4-4 4"/><path d="M3 11V9a3 3 0 0 1 3-3h15"/>'
              '<path d="M7 22l-4-4 4-4"/><path d="M21 13v2a3 3 0 0 1-3 3H3"/>',
    # Nuvem com seta para cima (OTA)
    "upload": '<path d="M7 18a4 4 0 0 1-.5-7.97A6 6 0 0 1 18 9a4 4 0 0 1 0 8"/><path d="M12 12v9M8.5 15.5L12 12l3.5 3.5"/>',
    # Octógono de parada
    "stop": '<path d="M8 2h8l6 6v8l-6 6H8l-6-6V8z"/><path d="M8 12h8"/>',
    # Plugue
    "plug": '<path d="M9 2v6M15 2v6"/><path d="M6 8h12v4a6 6 0 0 1-12 0z"/><path d="M12 18v4"/>',
    # Atualizar
    "refresh": '<path d="M21 12a9 9 0 1 1-2.64-6.36"/><path d="M21 3v6h-6"/>',
    # Engrenagem simplificada (marca)
    "gear": '<circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M4.2 4.2l2.1 2.1M17.7 17.7l2.1 2.1'
            'M2 12h3M19 12h3M4.2 19.8l2.1-2.1M17.7 6.3l2.1-2.1"/>',
    # Rede de nós
    "nodes": '<circle cx="12" cy="5" r="2.5"/><circle cx="5" cy="19" r="2.5"/><circle cx="19" cy="19" r="2.5"/>'
             '<path d="M12 7.5v4M12 11.5l-5.5 5.5M12 11.5l5.5 5.5"/>',
}


def _svg(name: str, color: str) -> bytes:
    body = _PATHS[name]
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="{color}" '
        f'stroke-width="2" stroke-linecap="round" stroke-linejoin="round">{body}</svg>'
    ).encode("utf-8")


def icon(name: str, color: str = None, size: int = 20) -> QIcon:
    """QIcon do ícone `name` na cor dada (padrão: texto do tema), com variante desabilitada."""
    result = QIcon()
    for mode, tone in ((QIcon.Mode.Normal, color or PALETTE["text"]),
                       (QIcon.Mode.Disabled, PALETTE["text_dim"])):
        renderer = QSvgRenderer(QByteArray(_svg(name, tone)))
        for scale in (1, 2):  # 2x para telas de alta densidade
            pixmap = QPixmap(size * scale, size * scale)
            pixmap.fill(Qt.GlobalColor.transparent)
            painter = QPainter(pixmap)
            renderer.render(painter, QRectF(0, 0, size * scale, size * scale))
            painter.end()
            pixmap.setDevicePixelRatio(scale)
            result.addPixmap(pixmap, mode)
    return result
