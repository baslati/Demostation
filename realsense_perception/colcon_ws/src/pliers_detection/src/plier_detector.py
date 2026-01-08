import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray, Marker
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge

class PliersDetector(Node):
    def __init__(self):
        super().__init__('pliers_detector')

        # Parameter für Tisch-Höhe
        self.declare_parameter('table_z', 0.0)
        self.table_z = self.get_parameter('table_z').value

        # Subscriber für Point Cloud
        self.pc_sub = self.create_subscription(
            PointCloud2, '/camera/points', self.point_cloud_callback, 10)

        # Subscriber für RGB Image
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        # Publisher für Detections
        self.detection_pub = self.create_publisher(Detection3DArray, '/detected_pliers', 10)

        # Publisher für Marker
        self.marker_pub = self.create_publisher(MarkerArray, '/pliers_markers', 10)

        # Publisher für Overlay Image
        self.overlay_pub = self.create_publisher(Image, '/camera/color/image_overlay', 10)

        # TF Broadcaster für Tisch-Frame
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_table_frame()

        self.bridge = CvBridge()
        self.latest_image = None
        self.detections = []

        self.get_logger().info('Pliers Detector Node started')

    def broadcast_table_frame(self):
        # Tisch-Frame an Position relativ zu Kamera
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_link'
        transform.child_frame_id = 'table_frame'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = self.table_z
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def point_cloud_callback(self, msg):
        # Konvertierung von PointCloud2 zu numpy array (vereinfacht für XYZ)
        # Annahme: XYZ-Daten ohne Farbe
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)

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

            # PCA für Ausrichtung und Form
            pca = PCA(n_components=3)
            pca.fit(cluster_points)
            eigenvalues = pca.explained_variance_
            eigenvectors = pca.components_

            # Prüfe, ob es länglich ist (z.B. Verhältnis Eigenwerte)
            if len(eigenvalues) >= 2 and eigenvalues[0] / eigenvalues[1] > 2.0:  # Länglich
                # Hauptachse als Ausrichtung
                main_axis = eigenvectors[0]
                # Quaternion aus Vektor (vereinfacht)
                from scipy.spatial.transform import Rotation as R
                rotation = R.align_vectors([main_axis], [[1, 0, 0]])[0]
                orientation = rotation.as_quat()
            else:
                continue  # Keine Zange

            # Centroid
            centroid = np.mean(cluster_points, axis=0)

            # Detection erstellen
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
        self.detections = detections.detections  # Speichere für Overlay

        # Erstelle Marker für RViz
        marker_array = MarkerArray()
        for i, detection in enumerate(detections.detections):
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'pliers'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = detection.bbox.center
            marker.scale.x = 0.1  # Dummy-Größe, anpassen
            marker.scale.y = 0.05
            marker.scale.z = 0.02
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(detections.detections)} pliers detections')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.detections:
            overlay = self.latest_image.copy()
            # Vereinfacht: Zeichne Rechtecke an Dummy-Positionen (für echte Projection bräuchte man Kamera-Info)
            for detection in self.detections:
                # Dummy: Annahme, dass Detection in Pixel-Koordinaten ist (anpassen!)
                cv2.rectangle(overlay, (100, 100), (200, 150), (0, 255, 0), 2)
                cv2.putText(overlay, 'Plier', (100, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PliersDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()