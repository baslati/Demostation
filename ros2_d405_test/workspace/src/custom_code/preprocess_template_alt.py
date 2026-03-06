#!/usr/bin/env python3
"""
Preprocessing-Script für Punktwolken-Templates.

Liest einen rohen Scan (.pcd), entfernt Tisch und Rauschen,
und speichert ein sauberes Template für das spätere Matching.

Workflow:
  1. Passthrough-Filter: Entfernt alles außerhalb einer definierten Bounding Box (Höhe über Tisch)
  2. Statistical Outlier Removal: Entfernt isolierte Rauschpunkte
  3. Optional: Voxel-Downsampling für gleichmäßige Punktdichte
  4. Speichern als sauberes Template
"""
import argparse
import os
import sys

import numpy as np
import open3d as o3d


def load_pointcloud(path):
    """Lädt eine Punktwolke und zeigt Grundinfos."""
    if not os.path.exists(path):
        print(f"Fehler: Datei nicht gefunden: {path}")
        sys.exit(1)

    pcd = o3d.io.read_point_cloud(path)
    print(f"Geladen: {path}")
    print(f"  Punkte: {len(pcd.points)}")
    print(f"  Hat Farben: {pcd.has_colors()}")
    return pcd


def show_bounds(pcd):
    """Zeigt die Min/Max-Werte der Punktwolke."""
    points = np.asarray(pcd.points)
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    print(f"  Wertebereiche:")
    print(f"    X: {mins[0]:.4f} bis {maxs[0]:.4f} m")
    print(f"    Y: {mins[1]:.4f} bis {maxs[1]:.4f} m")
    print(f"    Z: {mins[2]:.4f} bis {maxs[2]:.4f} m")
    return mins, maxs


def crop_bounding_box(pcd, z_min, z_max, x_range, y_range):
    """
    Schneidet die Punktwolke auf eine Bounding Box zu.
    Entfernt den Tisch (unter z_min) und Hintergrund.

    Args:
        pcd: Open3D PointCloud
        z_min: Minimale Höhe (m) - alles darunter wird entfernt (Tisch)
        z_max: Maximale Höhe (m) - alles darüber wird entfernt
        x_range: Tuple (x_min, x_max) in Metern
        y_range: Tuple (y_min, y_max) in Metern
    """
    points = np.asarray(pcd.points)

    print(f"  Filter: X=[{x_range[0]:.3f}, {x_range[1]:.3f}], Y=[{y_range[0]:.3f}, {y_range[1]:.3f}], Z=[{z_min:.3f}, {z_max:.3f}]")

    # Bounding Box anwenden
    mask = (
        (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
        (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
        (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    )

    indices = np.where(mask)[0]
    cropped = pcd.select_by_index(indices.tolist())
    print(f"  Nach Crop: {len(cropped.points)} Punkte (von {len(points)})")

    if len(cropped.points) == 0:
        print(f"\n  WARNUNG: Alle Punkte entfernt! Die Crop-Werte passen nicht zu den Daten.")
        print(f"  Nutze die oben angezeigten Wertebereiche um passende --z-min/--z-max etc. zu setzen.")

    return cropped


def remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    Entfernt statistische Ausreißer.

    Args:
        nb_neighbors: Anzahl Nachbarn für die Berechnung
        std_ratio: Standardabweichungs-Schwellwert (niedriger = aggressiver)
    """
    filtered, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    removed = len(pcd.points) - len(filtered.points)
    print(f"  Outlier Removal: {removed} Punkte entfernt, {len(filtered.points)} übrig")
    return filtered


def voxel_downsample(pcd, voxel_size=0.001):
    """
    Voxel-Downsampling für gleichmäßige Punktdichte.

    Args:
        voxel_size: Voxelgröße in Metern (0.001 = 1mm)
    """
    downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"  Nach Downsampling ({voxel_size*1000:.1f}mm): {len(downsampled.points)} Punkte")
    return downsampled


def estimate_normals(pcd):
    """Berechnet Normalen (nützlich für späteres ICP-Matching)."""
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
    )
    print(f"  Normalen berechnet")
    return pcd


def visualize(pcd, window_name="Preview"):
    """Zeigt die Punktwolke zur visuellen Kontrolle."""
    print(f"  Visualisierung geöffnet. Fenster schließen zum Fortfahren...")
    o3d.visualization.draw_geometries(
        [pcd],
        window_name=window_name,
        width=800, height=600
    )


def main():
    parser = argparse.ArgumentParser(
        description="Punktwolken-Template Preprocessing",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Beispiele:
  # Standard-Preprocessing mit Vorschau:
  python3 preprocess_template.py scans/pliers_long.pcd --preview

  # Aggressiveres Cropping (z.B. Tisch bei 0.01m, Zange bis 0.05m):
  python3 preprocess_template.py scans/pliers_long.pcd --z-min 0.01 --z-max 0.05

  # Ohne Downsampling (volle Auflösung behalten):
  python3 preprocess_template.py scans/pliers_long.pcd --no-downsample

  # Eigenen Ausgabepfad angeben:
  python3 preprocess_template.py scans/pliers_long.pcd -o templates/pliers_long_clean.pcd
        """
    )
    parser.add_argument("input", help="Pfad zur rohen .pcd Datei")
    parser.add_argument("-o", "--output", help="Ausgabepfad (Standard: templates/<name>_clean.pcd)")
    parser.add_argument("--z-min", type=float, default=0.005,
                        help="Min. Höhe in Metern über Kamera-Nullpunkt (Standard: 0.005)")
    parser.add_argument("--z-max", type=float, default=0.10,
                        help="Max. Höhe in Metern (Standard: 0.10)")
    parser.add_argument("--x-range", type=float, nargs=2, default=[-0.15, 0.15],
                        help="X-Bereich in Metern (Standard: -0.15 0.15)")
    parser.add_argument("--y-range", type=float, nargs=2, default=[-0.15, 0.15],
                        help="Y-Bereich in Metern (Standard: -0.15 0.15)")
    parser.add_argument("--voxel-size", type=float, default=0.001,
                        help="Voxelgröße für Downsampling in Metern (Standard: 0.001 = 1mm)")
    parser.add_argument("--no-downsample", action="store_true",
                        help="Kein Voxel-Downsampling anwenden")
    parser.add_argument("--outlier-neighbors", type=int, default=20,
                        help="Nachbarn für Outlier Removal (Standard: 20)")
    parser.add_argument("--outlier-std", type=float, default=2.0,
                        help="Standardabweichungs-Schwellwert (Standard: 2.0)")
    parser.add_argument("--preview", action="store_true",
                        help="Vorher/Nachher Visualisierung anzeigen")
    parser.add_argument("--normals", action="store_true",
                        help="Normalen berechnen (empfohlen für ICP-Matching)")

    args = parser.parse_args()

    # Laden
    print("=" * 50)
    print("Punktwolken-Template Preprocessing")
    print("=" * 50)
    pcd = load_pointcloud(args.input)
    show_bounds(pcd)

    if args.preview:
        visualize(pcd, "VORHER (Roher Scan)")

    # Crop
    print("\n[1/4] Bounding Box Crop...")
    pcd = crop_bounding_box(pcd, args.z_min, args.z_max,
                            tuple(args.x_range), tuple(args.y_range))

    if len(pcd.points) == 0:
        print("\nAbbruch: Keine Punkte nach Crop übrig. Passe die Crop-Parameter an.")
        sys.exit(1)

    # Outlier Removal
    print("\n[2/4] Statistical Outlier Removal...")
    pcd = remove_outliers(pcd, args.outlier_neighbors, args.outlier_std)

    # Downsampling
    if not args.no_downsample:
        print("\n[3/4] Voxel Downsampling...")
        pcd = voxel_downsample(pcd, args.voxel_size)
    else:
        print("\n[3/4] Downsampling übersprungen")

    # Normalen
    if args.normals:
        print("\n[4/4] Normalen berechnen...")
        pcd = estimate_normals(pcd)
    else:
        print("\n[4/4] Normalen übersprungen (--normals zum Aktivieren)")

    if args.preview:
        visualize(pcd, "NACHHER (Bereinigtes Template)")

    # Ausgabepfad bestimmen
    if args.output:
        output_path = args.output
    else:
        base_dir = os.path.dirname(args.input)
        template_dir = os.path.join(os.path.dirname(base_dir) if base_dir else ".", "templates")
        os.makedirs(template_dir, exist_ok=True)
        name = os.path.splitext(os.path.basename(args.input))[0]
        output_path = os.path.join(template_dir, f"{name}_clean.pcd")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    o3d.io.write_point_cloud(output_path, pcd)

    print(f"\n{'=' * 50}")
    print(f"Template gespeichert: {output_path}")
    print(f"Endgültige Punktanzahl: {len(pcd.points)}")
    print(f"{'=' * 50}")


if __name__ == "__main__":
    main()
