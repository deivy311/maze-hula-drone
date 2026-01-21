"""
Board rendering module using Pillow.
"""

from typing import Optional
from PIL import Image, ImageDraw, ImageFont
from maze import Maze
from qr import create_qr_code


def render_board(
    maze: Maze,
    cell_px: int = 260,
    wall_thickness: int = 4,
    show_coords: bool = False,
    qr_padding: int = 10
) -> Image.Image:
    """
    Render the entire maze board as a PNG image.
    
    Args:
        maze: The maze to render
        cell_px: Pixel size of each cell (square)
        wall_thickness: Thickness of wall lines in pixels
        show_coords: Whether to show coordinate labels in cell centers
        qr_padding: Padding around QR codes in pixels
    
    Returns:
        PIL Image of the rendered board
    """
    # Calculate image dimensions
    img_width = maze.cols * cell_px
    img_height = maze.rows * cell_px
    
    # Create white background
    img = Image.new('RGB', (img_width, img_height), color='white')
    draw = ImageDraw.Draw(img)
    
    # Try to load a font, fallback to default if not available
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
    except:
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf", 16)
        except:
            font = ImageFont.load_default()
    
    # Draw walls and place QR codes for each cell
    for r in range(maze.rows):
        for c in range(maze.cols):
            x0 = c * cell_px
            y0 = r * cell_px
            x1 = x0 + cell_px
            y1 = y0 + cell_px
            
            # Draw walls
            if maze.has_wall(r, c, 'N'):
                draw.rectangle(
                    [x0, y0, x1, y0 + wall_thickness],
                    fill='black'
                )
            if maze.has_wall(r, c, 'E'):
                draw.rectangle(
                    [x1 - wall_thickness, y0, x1, y1],
                    fill='black'
                )
            if maze.has_wall(r, c, 'S'):
                draw.rectangle(
                    [x0, y1 - wall_thickness, x1, y1],
                    fill='black'
                )
            if maze.has_wall(r, c, 'W'):
                draw.rectangle(
                    [x0, y0, x0 + wall_thickness, y1],
                    fill='black'
                )
            
            # Calculate QR code size for 2x2 grid with spacing
            # Available space = cell_px - 2*qr_padding (padding from edges)
            # We need: 2 QR codes + 1 gap (half QR size) = 2.5 * qr_size
            available_space = cell_px - 2 * qr_padding
            qr_size = int(available_space / 2.5)
            gap = qr_size // 2  # Half of QR code size
            
            # Calculate starting position to center the 2x2 grid
            total_grid_size = 2 * qr_size + gap
            start_x = x0 + (cell_px - total_grid_size) // 2
            start_y = y0 + (cell_px - total_grid_size) // 2
            
            # Create QR codes for each side
            neighbor_n = maze.get_neighbor(r, c, 'N')
            qr_n = create_qr_code(r, c, 'N', not maze.has_wall(r, c, 'N'), neighbor_n)
            qr_n = qr_n.resize((qr_size, qr_size), Image.Resampling.LANCZOS)
            
            neighbor_e = maze.get_neighbor(r, c, 'E')
            qr_e = create_qr_code(r, c, 'E', not maze.has_wall(r, c, 'E'), neighbor_e)
            qr_e = qr_e.resize((qr_size, qr_size), Image.Resampling.LANCZOS)
            
            neighbor_s = maze.get_neighbor(r, c, 'S')
            qr_s = create_qr_code(r, c, 'S', not maze.has_wall(r, c, 'S'), neighbor_s)
            qr_s = qr_s.resize((qr_size, qr_size), Image.Resampling.LANCZOS)
            
            neighbor_w = maze.get_neighbor(r, c, 'W')
            qr_w = create_qr_code(r, c, 'W', not maze.has_wall(r, c, 'W'), neighbor_w)
            qr_w = qr_w.resize((qr_size, qr_size), Image.Resampling.LANCZOS)
            
            # Place QR codes in 2x2 grid:
            # [N] [E]
            # [W] [S]
            # Top-left: N
            img.paste(qr_n, (int(start_x), int(start_y)))
            # Top-right: E
            img.paste(qr_e, (int(start_x + qr_size + gap), int(start_y)))
            # Bottom-left: W
            img.paste(qr_w, (int(start_x), int(start_y + qr_size + gap)))
            # Bottom-right: S
            img.paste(qr_s, (int(start_x + qr_size + gap), int(start_y + qr_size + gap)))
            
            # Draw black spacing rectangles between QR codes
            # Horizontal gap (between top and bottom rows)
            draw.rectangle(
                [start_x, start_y + qr_size, start_x + total_grid_size, start_y + qr_size + gap],
                fill='black'
            )
            # Vertical gap (between left and right columns)
            draw.rectangle(
                [start_x + qr_size, start_y, start_x + qr_size + gap, start_y + total_grid_size],
                fill='black'
            )
            
            # Draw coordinate label in center (optional)
            if show_coords:
                label = f"({r},{c})"
                # Get text bounding box
                bbox = draw.textbbox((0, 0), label, font=font)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
                text_x = x0 + (cell_px - text_width) // 2
                text_y = y0 + (cell_px - text_height) // 2
                # Draw text with slight background for readability
                draw.rectangle(
                    [text_x - 2, text_y - 2, text_x + text_width + 2, text_y + text_height + 2],
                    fill='white',
                    outline='gray',
                    width=1
                )
                draw.text((text_x, text_y), label, fill='black', font=font)
    
    return img
