#!/usr/bin/env python3
"""
Preprocessing-Script fuer Punktwolken-Templates.

Liest einen rohen Scan (.pcd), entfernt den Tisch per RANSAC und Rauschen,
und speichert ein sauberes Template fuer das spaetere Matching.

Workflow:
  1. RANSAC Plane Detection: Findet und entfernt die Tischebene automatisch
  2. Cluster Extraction: Findet den groessten Cluster (= Zange)
  3. Statistical Outlier Removal: Entfernt isolierte Rauschpunkte
  4. Optional: Voxel-Downsampling fuer gleichmaessige Punktdichte
  5. Zentrierung auf den Centroid
  6. Rotation: Y-Achse zeigt entlang der Zange (PCA-Hauptachse)
"""
import argparse
import os
import sys

import numpy as np
import open3d as o3d


def load_pointcloud(path):
    """Laedt eine Punktwolke und zeigt Grundinfos."""
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
    print("  Wertebereiche:")
    print(f"    X: {mins[0]:.4f} bis {maxs[0]:.4f} m")
    print(f"    Y: {mins[1]:.4f} bis {maxs[1]:.4f} m")
    print(f"    Z: {mins[2]:.4f} bis {maxs[2]:.4f} m")
    return mins, maxs


def remove_table_ransac(pcd, distance_threshold=0.005, iterations=1000):
    """Entfernt die Tischebene automatisch per RANSAC."""
    if len(pcd.points) < 100:
        print("  Zu wenige Punkte fuer RANSAC")
        return pcd

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=iterations,
    )

    [a, b, c, d] = plane_model
    print(f"  Ebene gefunden: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
    print(f"  Tisch-Punkte (Inliers): {len(inliers)}")

    objects_pcd = pcd.select_by_index(inliers, invert=True)
    print(f"  Objekt-Punkte (uebrig): {len(objects_pcd.points)}")
    return objects_pcd


def extract_largest_cluster(pcd, eps=0.005, min_points=100):
    """Findet den groessten zusammenhaengenden Cluster (= die Zange)."""
    if len(pcd.points) < min_points:
        print(f"  Zu wenige Punkte ({len(pcd.points)}) fuer Clustering")
        return pcd

    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))

    if len(labels) == 0 or labels.max() < 0:
        print("  Keine Cluster gefunden")
        return pcd

    n_clusters = labels.max() + 1
    print(f"  {n_clusters} Cluster gefunden")

    cluster_sizes = []
    for i in range(n_clusters):
        size = np.sum(labels == i)
        cluster_sizes.append(size)
        print(f"    Cluster {i}: {size} Punkte")

    largest_idx = int(np.argmax(cluster_sizes))
    largest_mask = labels == largest_idx
    largest_pcd = pcd.select_by_index(np.where(largest_mask)[0].tolist())

    print(f"  Groesster Cluster ({largest_idx}): {len(largest_pcd.points)} Punkte")
    return largest_pcd


def remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """Entfernt statistische Ausreisser."""
    if len(pcd.points) < nb_neighbors:
        print("  Zu wenige Punkte fuer Outlier Removal")
        return pcd

    filtered, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    removed = len(pcd.points) - len(filtered.points)
    print(f"  Outlier Removal: {removed} Punkte entfernt, {len(filtered.points)} uebrig")
    return filtered


def voxel_downsample(pcd, voxel_size=0.001):
    """Voxel-Downsampling fuer gleichmaessige Punktdichte."""
    downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"  Nach Downsampling ({voxel_size*1000:.1f}mm): {len(downsampled.points)} Punkte")
    return downsampled


def estimate_normals(pcd):
    """Berechnet Normalen (nuetzlich fuer spaeteres ICP-Matching)."""
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
    )
    print("  Normalen berechnet")
    return pcd


def compute_centroid_and_translate(pcd):
    """
    Berechnet den Centroid (Mittelpunkt) und zentriert die Punktwolke.

    Returns:
        pcd: Zentrierte Punktwolke
        centroid: numpy array [x, y, z] des Original-Centroid im Kamera-Frame
    """
    points = np.asarray(pcd.points, dtype=np.float64)
    centroid = points.mean(axis=0)

    pcd.translate(-centroid, relative=True)

    print(f"  Centroid berechnet: [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}] m")
    print("  Zange zentriert um Origin (0, 0, 0)")
    return pcd, centroid


def align_y_axis_to_tool(pcd):
    """
    Richtet die Punktwolke so aus, dass die laengste PCA-Achse auf +Y liegt.

    Hinweis:
    - Die Richtung entlang der Zange ist damit konsistent auf der Y-Achse.
    - Bei symmetrischen Formen kann die Vorzeichenwahl ohne Zusatzinformation
      physikalisch nicht eindeutig sein; hier wird +Y bevorzugt.
    """
    pts = np.asarray(pcd.points, dtype=np.float64)
    if len(pts) < 10:
        print("  Zu wenige Punkte fuer PCA-Ausrichtung")
        return pcd

    centered = pts - pts.mean(axis=0)
    cov = centered.T @ centered / max(1, len(pts) - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    basis = eigvecs[:, order]

    if np.linalg.det(basis) < 0.0:
        basis[:, 2] *= -1.0

    longest = basis[:, 0]
    middle = basis[:, 1]

    if longest[1] < 0.0:
        longest = -longest

    z_axis = np.cross(middle, longest)
    z_norm = np.linalg.norm(z_axis)
    if z_norm < 1e-9:
        print("  Degenerierte PCA-Ausrichtung, Rotation uebersprungen")
        return pcd
    z_axis = z_axis / z_norm

    x_axis = np.cross(longest, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = longest / np.linalg.norm(longest)

    world_from_local = np.column_stack((x_axis, y_axis, z_axis))
    rot = world_from_local.T

    pcd.rotate(rot, center=np.array([0.0, 0.0, 0.0]))

    print("  Ausrichtung gesetzt: Y-Achse zeigt entlang der Zange")
    return pcd


def visualize(pcd, window_name="Preview"):
    """Zeigt die Punktwolke zur visuellen Kontrolle."""
    print("  Visualisierung geoeffnet. Fenster schliessen zum Fortfahren...")
    o3d.visualization.draw_geometries(
        [pcd],
        window_name=window_name,
        width=800,
        height=600,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Punktwolken-Template Preprocessing mit Y-Achsen-Ausrichtung",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Beispiele:
  # Standard-Preprocessing (inkl. Y-Achsen-Ausrichtung):
  python3 preprocess_template_ransac_y_axis.py scans/pliers_long.pcd --preview

  # Ohne Downsampling:
  python3 preprocess_template_ransac_y_axis.py scans/pliers_long.pcd --no-downsample

  # Eigenen Ausgabepfad:
  python3 preprocess_template_ransac_y_axis.py scans/pliers_long.pcd -o templates/pliers_long_clean.pcd
        """,
    )
    parser.add_argument("input", help="Pfad zur rohen .pcd Datei")
    parser.add_argument("-o", "--output", help="Ausgabepfad (Standard: templates/<name>_clean.pcd)")
    parser.add_argument(
        "--ransac-threshold",
        type=float,
        default=0.004,
        help="RANSAC Distanz-Schwellwert in Metern (Standard: 0.004 = 4mm)",
    )
    parser.add_argument(
        "--ransac-iterations",
        type=int,
        default=1000,
        help="RANSAC Iterationen (Standard: 1000)",
    )
    parser.add_argument(
        "--cluster-eps",
        type=float,
        default=0.010,
        help="Max Abstand zwischen Punkten im Cluster in Metern (Standard: 0.010 = 10mm)",
    )
    parser.add_argument(
        "--cluster-min-points",
        type=int,
        default=100,
        help="Min. Punkte pro Cluster (Standard: 100)",
    )
    parser.add_argument("--no-cluster", action="store_true", help="Cluster-Extraktion ueberspringen")
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.001,
        help="Voxelgroesse fuer Downsampling in Metern (Standard: 0.001 = 1mm)",
    )
    parser.add_argument("--no-downsample", action="store_true", help="Kein Voxel-Downsampling")
    parser.add_argument(
        "--outlier-neighbors",
        type=int,
        default=20,
        help="Nachbarn fuer Outlier Removal (Standard: 20)",
    )
    parser.add_argument(
        "--outlier-std",
        type=float,
        default=2.0,
        help="Standardabweichungs-Schwellwert (Standard: 2.0)",
    )
    parser.add_argument("--preview", action="store_true", help="Vorher/Nachher Visualisierung anzeigen")
    parser.add_argument("--normals", action="store_true", help="Normalen berechnen")

    args = parser.parse_args()

    print("=" * 50)
    print("Punktwolken-Template Preprocessing (Y-Achse entlang Zange)")
    print("=" * 50)

    pcd = load_pointcloud(args.input)
    show_bounds(pcd)

    if args.preview:
        visualize(pcd, "VORHER (Roher Scan)")

    print("\n[1/6] Tischebene entfernen (RANSAC)...")
    pcd = remove_table_ransac(pcd, args.ransac_threshold, args.ransac_iterations)
    if len(pcd.points) == 0:
        print("\nAbbruch: Keine Punkte nach Tisch-Entfernung uebrig.")
        sys.exit(1)

    if not args.no_cluster:
        print("\n[2/6] Groessten Cluster extrahieren...")
        pcd = extract_largest_cluster(pcd, args.cluster_eps, args.cluster_min_points)
    else:
        print("\n[2/6] Clustering uebersprungen")
    if len(pcd.points) == 0:
        print("\nAbbruch: Keine Punkte nach Clustering uebrig.")
        sys.exit(1)

    print("\n[3/6] Statistical Outlier Removal...")
    pcd = remove_outliers(pcd, args.outlier_neighbors, args.outlier_std)

    if not args.no_downsample:
        print("\n[4/6] Voxel Downsampling...")
        pcd = voxel_downsample(pcd, args.voxel_size)
    else:
        print("\n[4/6] Downsampling uebersprungen")

    if args.normals:
        print("\n[5/6] Normalen berechnen...")
        pcd = estimate_normals(pcd)
    else:
        print("\n[5/6] Normalen uebersprungen (--normals zum Aktivieren)")

    print("\n[6/6] Zentrieren + Y-Achse ausrichten...")
    pcd, _ = compute_centroid_and_translate(pcd)
    pcd = align_y_axis_to_tool(pcd)

    if args.preview:
        visualize(pcd, "NACHHER (zentriert + Y entlang Zange)")

    if args.output:
        output_path = args.output
    else:
        base_dir = os.path.dirname(args.input)
        template_dir = os.path.join(os.path.dirname(base_dir) if base_dir else ".", "templates")
        os.makedirs(template_dir, exist_ok=True)
        name = os.path.splitext(os.path.basename(args.input))[0]
        output_path = os.path.join(template_dir, f"{name}_clean_direction.pcd")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    o3d.io.write_point_cloud(output_path, pcd)

    print(f"\n{'=' * 50}")
    print(f"Template gespeichert: {output_path}")
    print(f"Endgueltige Punktanzahl: {len(pcd.points)}")
    print("Origin: Zangen-Mittelpunkt (Centroid)")
    print("Ausrichtung: Y-Achse zeigt in Zangenrichtung")
    print(f"{'=' * 50}")


if __name__ == "__main__":
    main()
