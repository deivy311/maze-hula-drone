"""
QR code generation module for maze cell sides.
"""

import json
from typing import Optional, Tuple
import qrcode
from PIL import Image


def create_qr_code(
    r: int, 
    c: int, 
    side: str, 
    is_open: bool, 
    to: Optional[Tuple[int, int]] = None
) -> Image.Image:
    """
    Create a QR code image for a cell side.
    
    Args:
        r: Row index
        c: Column index
        side: One of 'N', 'E', 'S', 'W'
        is_open: Whether movement through this side is possible
        to: Optional neighbor coordinates [nr, nc] if is_open is True
    
    Returns:
        PIL Image of the QR code
    """
    # Build JSON payload
    payload = {
        'r': r,
        'c': c,
        'side': side,
        'open': is_open
    }
    
    if is_open and to is not None:
        payload['to'] = list(to)
    
    # Minify JSON (no spaces)
    json_str = json.dumps(payload, separators=(',', ':'))
    
    # Create QR code
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=2,
    )
    qr.add_data(json_str)
    qr.make(fit=True)
    
    # Convert to PIL Image
    qr_img = qr.make_image(fill_color="black", back_color="white")
    
    return qr_img
