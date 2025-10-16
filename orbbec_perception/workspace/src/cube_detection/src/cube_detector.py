import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import open3d as o3d
import numpy as np

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
        # Konvertiere PointCloud2 zu Open3D (vereinfacht – nutze ros2_numpy für vollständige Konvertierung)
        # Hier Dummy: Angenommen, msg.data enthält XYZ-Punkte
        # Erweitere mit: import ros2_numpy as rnp; points = rnp.point_cloud2.point_cloud2_to_array(msg)
        points = np.random.rand(1000, 3) * 2  # Dummy-Daten – ersetze mit echter Konvertierung!
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        if cloud.is_empty():
            return

        # Filter: Entferne Punkte außerhalb Arbeitsbereich (z.B. Höhe)
        points = np.asarray(cloud.points)
        valid = (points[:, 2] > 0.1) & (points[:, 2] < 1.0)
        cloud = cloud.select_by_index(np.where(valid)[0])

        # Tisch-Segmentierung (RANSAC für Ebene)
        plane_model, inliers = cloud.segment_plane(distance_threshold=0.01,
                                                   ransac_n=3,
                                                   num_iterations=1000)
        outlier_cloud = cloud.select_by_index(inliers, invert=True)  # Objekte

        # Clustering für Würfel (DBSCAN)
        labels = np.array(outlier_cloud.cluster_dbscan(eps=0.02, min_points=10))
        max_label = labels.max() if len(labels) > 0 else -1

        # Veröffentliche Detections
        detections = Detection3DArray()
        detections.header = msg.header
        detections.header.frame_id = 'table_frame'

        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            if len(cluster_indices) < 100:  # Min-Cluster-Größe
                continue
            cluster_cloud = outlier_cloud.select_by_index(cluster_indices)

            # Centroid
            centroid = np.mean(np.asarray(cluster_cloud.points), axis=0)

            # Ausrichtung (vereinfacht – PCA mit Open3D)
            # Für Würfel: Annahme quaderförmig
            orientation = [0.0, 0.0, 0.0, 1.0]  # Quaternion (anpassen)

            # Farbe: Dummy (erweitere mit RGB-Bild)
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