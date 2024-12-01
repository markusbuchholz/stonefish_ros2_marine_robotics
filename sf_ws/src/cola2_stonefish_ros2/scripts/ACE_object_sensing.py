import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from cola2_stonefish_interfaces.srv import Cluster, ClusterResponse
from cola2_stonefish_interfaces.msg import ClusterObjInfo
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import xml.etree.ElementTree as ET
from xml.dom import minidom
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

octomap_data = None

class ACEClusteringServer(Node):
    def __init__(self):
        super().__init__('ACE_clustering_server')
        self.octomap_sub = self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self.octomap_callback, 10)
        self.srv = self.create_service(Cluster, 'cluster', self.handle_cluster)
        self.object_info_pub = self.create_publisher(ClusterObjInfo, 'object_info', 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.get_logger().info("Ready to cluster.")

    def octomap_callback(self, msg):
        global octomap_data
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        octomap_data = points
        self.get_logger().info("Received octomap data.")

    def handle_cluster(self, req, response):
        global octomap_data
        if octomap_data is None:
            response.success = False
            response.message = "Octomap data not available."
            return response
        
        points = octomap_data
        optimal_clusters = self.find_optimal_clusters(points)
        object_names = ['cone', 'sphere', 'box', 'cylinder', 'torus']
        kmeans = KMeans(n_clusters=optimal_clusters)
        kmeans.fit(points)
        cluster_labels = kmeans.labels_
        bounding_boxes = []
        cluster_centroids = []

        object_info_list = []  # List to store object information

        for i, obj_name in enumerate(object_names):
            cluster_points = points[cluster_labels == i]
            min_coords = np.min(cluster_points, axis=0)
            max_coords = np.max(cluster_points, axis=0)
            safety_margin = 0.5
            min_coords -= safety_margin
            max_coords += safety_margin

            min_coords[2] = max(min_coords[2], 0.0)
            bounding_box = np.concatenate((min_coords, max_coords))
            centroid = np.mean(cluster_points, axis=0)

            # Create surface point
            surface_point = self.calculate_surface_point(cluster_points)

            bounding_boxes.append(bounding_box)
            cluster_centroids.append(centroid.tolist())

            # Publish TF for centroid
            self.tf_broadcast(centroid, "cluster_centroid_" + obj_name)

            # Publish TF for surface point
            self.tf_broadcast(surface_point, "cluster_surface_point_" + obj_name)

            # Create and publish ObjectInfo message
            object_info = ClusterObjInfo()
            object_info.name = obj_name
            object_info.centroid.x, object_info.centroid.y, object_info.centroid.z = centroid
            object_info.surface_point.x, object_info.surface_point.y, object_info.surface_point.z = surface_point
            object_info_list.append(object_info)

        # Publish object information
        self.publish_object_info(object_info_list)

        # Save bounding boxes to XML
        self.save_bounding_boxes_to_xml(bounding_boxes)

        # Return ClusterResponse
        response.success = True
        response.message = "Clustering completed successfully."
        return response

    def publish_object_info(self, object_info_list):
        for object_info in object_info_list:
            self.object_info_pub.publish(object_info)
            self.get_logger().info(f"Published object info for {object_info.name}")

    def find_optimal_clusters(self, data):
        silhouette_scores = []
        max_clusters = 5

        for i in range(2, max_clusters + 1):
            kmeans = KMeans(n_clusters=i, random_state=42)
            cluster_labels = kmeans.fit_predict(data)
            silhouette_avg = silhouette_score(data, cluster_labels)
            silhouette_scores.append(silhouette_avg)

        plt.plot(range(2, max_clusters + 1), silhouette_scores, marker='o')
        plt.xlabel('Number of clusters')
        plt.ylabel('Silhouette Score')
        plt.title('Silhouette Score for Optimal Number of Clusters')
        plt.show()

        optimal_clusters = np.argmax(silhouette_scores) + 2  # Adding 2 because the range starts from 2
        self.get_logger().info(f"Optimal number of clusters: {optimal_clusters}")
        return optimal_clusters

    def save_bounding_boxes_to_xml(self, bounding_boxes):
        root = ET.Element("bounding_boxes")

        for i, box in enumerate(bounding_boxes):
            min_x, min_y, min_z, max_x, max_y, max_z = box

            bounding_box_elem = ET.SubElement(root, "bounding_box")
            bounding_box_elem.set("id", str(i + 1))

            min_coords_elem = ET.SubElement(bounding_box_elem, "min_coordinates")
            min_coords_elem.text = f"{min_x} {min_y} {min_z}"

            max_coords_elem = ET.SubElement(bounding_box_elem, "max_coordinates")
            max_coords_elem.text = f"{max_x} {max_y} {max_z}"

        tree = ET.ElementTree(root)
        xml_str = ET.tostring(root, encoding="unicode")
        xml_str_pretty = minidom.parseString(xml_str).toprettyxml(indent="    ")

        with open("bounding_boxes.xml", "w") as xml_file:
            xml_file.write(xml_str_pretty)

        self.get_logger().info("Bounding boxes saved to 'bounding_boxes.xml'.")

    def tf_broadcast(self, point, frame_id):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "world_ned"
        tf_msg.child_frame_id = frame_id
        tf_msg.transform.translation.x = point[0]
        tf_msg.transform.translation.y = point[1]
        tf_msg.transform.translation.z = point[2]
        tf_msg.transform.rotation.w = 1.0  # No rotation

        self.tf_broadcaster.sendTransform(tf_msg)

    def calculate_surface_point(self, cluster_points):
        centroid = np.mean(cluster_points, axis=0)
        surface_point = np.max(cluster_points[np.logical_and(cluster_points[:, :2] == centroid[:2], cluster_points[:, 2] == np.max(cluster_points[:, 2]))], axis=0)
        return surface_point

def main(args=None):
    rclpy.init(args=args)
    clustering_server = ACEClusteringServer()
    rclpy.spin(clustering_server)
    clustering_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

