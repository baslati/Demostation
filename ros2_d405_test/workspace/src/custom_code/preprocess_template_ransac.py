#!/usr/bin/env python3
"""
Preprocessing-Script für Punktwolken-Templates.

Liest einen rohen Scan (.pcd), entfernt den Tisch per RANSAC und Rauschen,
und speichert ein sauberes Template für das spätere Matching.

Workflow:
  1. RANSAC Plane Detection: Findet und entfernt die Tischebene automatisch
  2. Cluster Extraction: Findet den größten Cluster (= Zange)
  3. Statistical Outlier Removal: Entfernt isolierte Rauschpunkte
  4. Optional: Voxel-Downsampling für gleichmäßige Punktdichte
  5. Speichern als sauberes Template
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


def remove_table_ransac(pcd, distance_threshold=0.005, iterations=1000):
    """
    Entfernt die Tischebene automatisch per RANSAC.

    Args:
        pcd: Open3D PointCloud
        distance_threshold: Max Abstand eines Punktes zur Ebene (in Metern)
        iterations: Anzahl RANSAC Iterationen

    Returns:
        objects_pcd: Punktwolke ohne Tisch
    """
    if len(pcd.points) < 100:
        print("  Zu wenige Punkte für RANSAC")
        return pcd

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=iterations)

    [a, b, c, d] = plane_model
    print(f"  Ebene gefunden: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
    print(f"  Tisch-Punkte (Inliers): {len(inliers)}")

    # Alles außer der Ebene behalten
    objects_pcd = pcd.select_by_index(inliers, invert=True)
    print(f"  Objekt-Punkte (übrig): {len(objects_pcd.points)}")
    return objects_pcd


def extract_largest_cluster(pcd, eps=0.005, min_points=100):
    """
    Findet den größten zusammenhängenden Cluster (= die Zange).
    Entfernt kleine Fragmente und Rauschen.

    Args:
        pcd: Open3D PointCloud
        eps: Max Abstand zwischen Punkten im selben Cluster (in Metern)
        min_points: Min. Punkte pro Cluster
    """
    if len(pcd.points) < min_points:
        print(f"  Zu wenige Punkte ({len(pcd.points)}) für Clustering")
        return pcd

    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))

    if len(labels) == 0 or labels.max() < 0:
        print("  Keine Cluster gefunden")
        return pcd

    n_clusters = labels.max() + 1
    print(f"  {n_clusters} Cluster gefunden")

    # Größten Cluster finden
    cluster_sizes = []
    for i in range(n_clusters):
        size = np.sum(labels == i)
        cluster_sizes.append(size)
        print(f"    Cluster {i}: {size} Punkte")

    largest_idx = np.argmax(cluster_sizes)
    largest_mask = labels == largest_idx
    largest_pcd = pcd.select_by_index(np.where(largest_mask)[0].tolist())

    print(f"  Größter Cluster ({largest_idx}): {len(largest_pcd.points)} Punkte")
    return largest_pcd


def remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    Entfernt statistische Ausreißer.

    Args:
        nb_neighbors: Anzahl Nachbarn für die Berechnung
        std_ratio: Standardabweichungs-Schwellwert (niedriger = aggressiver)
    """
    if len(pcd.points) < nb_neighbors:
        print("  Zu wenige Punkte für Outlier Removal")
        return pcd

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
  # Standard-Preprocessing (Tisch automatisch entfernen):
  python3 preprocess_template.py scans/pliers_long.pcd --preview

  # Empfindlicheren RANSAC (dünnerer Tisch / raue Oberfläche):
  python3 preprocess_template.py scans/pliers_long.pcd --ransac-threshold 0.008

  # Ohne Downsampling (volle Auflösung behalten):
  python3 preprocess_template.py scans/pliers_long.pcd --no-downsample

  # Eigenen Ausgabepfad angeben:
  python3 preprocess_template.py scans/pliers_long.pcd -o templates/pliers_long_clean.pcd
        """
    )
    parser.add_argument("input", help="Pfad zur rohen .pcd Datei")
    parser.add_argument("-o", "--output", help="Ausgabepfad (Standard: templates/<name>_clean.pcd)")
    parser.add_argument("--ransac-threshold", type=float, default=0.004,
                        help="RANSAC Distanz-Schwellwert in Metern (Standard: 0.004 = 4mm)")
    parser.add_argument("--ransac-iterations", type=int, default=1000,
                        help="RANSAC Iterationen (Standard: 1000)")
    parser.add_argument("--cluster-eps", type=float, default=0.010,
                        help="Max Abstand zwischen Punkten im Cluster in Metern (Standard: 0.010 = 10mm)")
    parser.add_argument("--cluster-min-points", type=int, default=100,
                        help="Min. Punkte pro Cluster (Standard: 100)")
    parser.add_argument("--no-cluster", action="store_true",
                        help="Cluster-Extraktion überspringen")
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

    # 1. Tisch entfernen (RANSAC)
    print("\n[1/5] Tischebene entfernen (RANSAC)...")
    pcd = remove_table_ransac(pcd, args.ransac_threshold, args.ransac_iterations)

    if len(pcd.points) == 0:
        print("\nAbbruch: Keine Punkte nach Tisch-Entfernung übrig.")
        sys.exit(1)

    # 2. Größten Cluster extrahieren (= Zange)
    if not args.no_cluster:
        print("\n[2/5] Größten Cluster extrahieren...")
        pcd = extract_largest_cluster(pcd, args.cluster_eps, args.cluster_min_points)
    else:
        print("\n[2/5] Clustering übersprungen")

    if len(pcd.points) == 0:
        print("\nAbbruch: Keine Punkte nach Clustering übrig.")
        sys.exit(1)

    # 3. Outlier Removal
    print("\n[3/5] Statistical Outlier Removal...")
    pcd = remove_outliers(pcd, args.outlier_neighbors, args.outlier_std)

    # 4. Downsampling
    if not args.no_downsample:
        print("\n[4/5] Voxel Downsampling...")
        pcd = voxel_downsample(pcd, args.voxel_size)
    else:
        print("\n[4/5] Downsampling übersprungen")

    # 5. Normalen
    if args.normals:
        print("\n[5/5] Normalen berechnen...")
        pcd = estimate_normals(pcd)
    else:
        print("\n[5/5] Normalen übersprungen (--normals zum Aktivieren)")

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
