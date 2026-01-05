import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN

class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        # Subscriber für Point Cloud
        self.pc_sub = self.create_subscription(
            PointCloud2, '/camera/points', self.point_cloud_callback, 10)

        # Publisher für Detections
        self.detection_pub = self.create_publisher(Detection3DArray, '/detected_cubes', 10)

        # TF Broadcaster für Tisch-Frame
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_table_frame()

        self.get_logger().info('Cube Detector Node started')

    def broadcast_table_frame(self):
        # Beispiel: Tisch-Frame an Position (0,0,0) relativ zu Kamera (anpassen!)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_link'  # Oder dein Kamera-Frame
        transform.child_frame_id = 'table_frame'
        transform.transform.translation.x = 0.0  # Anpassen!
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def point_cloud_callback(self, msg):
        # Vereinfachte Konvertierung: Angenommen XYZ-Daten (erweitere mit ros2_numpy für echte Daten)
        # Dummy-Daten für Test: Zufällige Punkte
        points = np.random.rand(1000, 3) * 2  # Ersetze mit echter Konvertierung!

        if points.size == 0:
            return

        # Filter: Entferne Punkte außerhalb Arbeitsbereich (z.B. Höhe)
        valid = (points[:, 2] > 0.1) & (points[:, 2] < 1.0)
        points = points[valid]

        # Tisch-Segmentierung (vereinfacht: Annahme, Tisch ist bei z=0)
        # Erweitere mit RANSAC für echte Ebene-Erkennung
        table_z = 0.0  # Dummy: Tisch bei z=0
        object_points = points[points[:, 2] > table_z + 0.01]  # Punkte über Tisch

        # Clustering mit DBSCAN
        if object_points.shape[0] > 10:
            clustering = DBSCAN(eps=0.05, min_samples=10).fit(object_points)
            labels = clustering.labels_
            unique_labels = set(labels)
            if -1 in unique_labels:
                unique_labels.remove(-1)  # Rauschen ignorieren
        else:
            unique_labels = set()

        # Veröffentliche Detections
        detections = Detection3DArray()
        detections.header = msg.header
        detections.header.frame_id = 'table_frame'

        for label in unique_labels:
            cluster_points = object_points[labels == label]
            if cluster_points.shape[0] < 50:  # Min-Größe
                continue

            # Centroid
            centroid = np.mean(cluster_points, axis=0)

            # Ausrichtung (vereinfacht – erweitere mit PCA)
            orientation = [0.0, 0.0, 0.0, 1.0]  # Dummy

            # Farbe: Dummy
            detection = Detection3D()
            detection.header = msg.header
            detection.bbox.center.position.x = float(centroid[0])
            detection.bbox.center.position.y = float(centroid[1])
            detection.bbox.center.position.z = float(centroid[2])
            detection.bbox.center.orientation.x = orientation[0]
            detection.bbox.center.orientation.y = orientation[1]
            detection.bbox.center.orientation.z = orientation[2]
            detection.bbox.center.orientation.w = orientation[3]

            detections.detections.append(detection)

        self.detection_pub.publish(detections)
        self.get_logger().info(f'Published {len(detections.detections)} detections')

def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()