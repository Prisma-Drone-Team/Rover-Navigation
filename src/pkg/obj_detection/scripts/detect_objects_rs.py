#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Polygon as ROSPolygon
from geometry_msgs.msg import Point32, Pose
from obj_msgs.msg import DetectedObject, DetectedObjectsList
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import message_filters
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from shapely.geometry import Polygon
from sklearn.cluster import DBSCAN
from ultralytics import YOLO
import open3d as o3d
import numpy as np
import ros2_numpy
import struct
import time
import json
import cv2

COCO_CATEGORIES = [
    {"color": [255, 179, 240], "id": 24, "name": "backpack"},
    {"color": [0, 125, 92], "id": 25, "name": "umbrella"},
    {"color": [209, 0, 151], "id": 26, "name": "handbag"},
    {"color": [188, 208, 182], "id": 27, "name": "tie"},
    {"color": [0, 220, 176], "id": 28, "name": "suitcase"},
    {"color": [78, 180, 255], "id": 32, "name": "sports ball"},
    {"color": [197, 226, 255], "id": 39, "name": "bottle"},
    {"color": [171, 134, 1], "id": 40, "name": "wine glass"},
    {"color": [109, 63, 54], "id": 41, "name": "cup"},
    {"color": [207, 138, 255], "id": 42, "name": "fork"},
    {"color": [151, 0, 95], "id": 43, "name": "knife"},
    {"color": [9, 80, 61], "id": 44, "name": "spoon"},
    {"color": [84, 105, 51], "id": 45, "name": "bowl"},
    {"color": [74, 65, 105], "id": 46, "name": "banana"},
    {"color": [166, 196, 102], "id": 47, "name": "apple"},
    {"color": [208, 195, 210], "id": 48, "name": "sandwich"},
    {"color": [255, 109, 65], "id": 49, "name": "orange"},
    {"color": [153, 69, 1], "id": 56, "name": "chair"},
    {"color": [3, 95, 161], "id": 57, "name": "couch"},
    {"color": [119, 0, 170], "id": 59, "name": "bed"},
    {"color": [0, 182, 199], "id": 60, "name": "dining table"},
    {"color": [0, 165, 120], "id": 61, "name": "toilet"},
    {"color": [183, 130, 88], "id": 62, "name": "tv"},
    {"color": [95, 32, 0], "id": 63, "name": "laptop"},
    {"color": [130, 114, 135], "id": 64, "name": "mouse"},
    {"color": [110, 129, 133], "id": 65, "name": "remote"},
    {"color": [166, 74, 118], "id": 66, "name": "keyboard"},
    {"color": [219, 142, 185], "id": 67, "name": "cell phone"},
    {"color": [79, 210, 114], "id": 68, "name": "microwave"},
    {"color": [178, 90, 62], "id": 69, "name": "oven"},
    {"color": [65, 70, 15], "id": 70, "name": "toaster"},
    {"color": [127, 167, 115], "id": 71, "name": "sink"},
    {"color": [59, 105, 106], "id": 72, "name": "refrigerator"},
    {"color": [142, 108, 45], "id": 73, "name": "book"},
    {"color": [196, 172, 0], "id": 74, "name": "clock"},
    {"color": [95, 54, 80], "id": 75, "name": "vase"},
    {"color": [128, 76, 255], "id": 76, "name": "scissors"},
    {"color": [201, 57, 1], "id": 77, "name": "teddy bear"},
    {"color": [246, 0, 122], "id": 78, "name": "hair drier"},
    {"color": [191, 162, 208], "id": 79, "name": "toothbrush"},
]

class Realsense(Node):

    def __init__(self):
        super().__init__('realsense')

        self.get_logger().info('Initializing node...')

        self.pub = self.create_publisher(DetectedObjectsList, 'objects_detected', 10)

        qos_profile_pc = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=3, reliability=ReliabilityPolicy.RELIABLE)
        qos_profile_image = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=6, reliability=ReliabilityPolicy.RELIABLE)

        # Realsense subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, '/rover_camera/color/image_raw',qos_profile=qos_profile_image)
        self.pointcloud_sub = message_filters.Subscriber(self, PointCloud2, '/rover_camera/depth/color/points',qos_profile=qos_profile_pc)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub],
            queue_size=5,
            slop=1.0
        )
        self.ts.registerCallback(self.sync_callback)

        # TF listener
        self.target_frame = self.declare_parameter('target_frame','rover_camera_color_optical_frame').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO setup
        self.model = YOLO("/home/user/ros2_ws/src/obj_detection/scripts/yolo11n-seg.pt")
        self.category_dict = {cat['id']: cat['name'] for cat in COCO_CATEGORIES}

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.8, self.camera)
        self.color_image = []
        self.pc = []
        self.img_labels = []
        self.img_sizes = []

        self.get_logger().info('Done!')

    def calculate_iou(self, box1, box2):
        poly1 = Polygon(box1)
        poly2 = Polygon(box2)
        intersection_area = poly1.intersection(poly2).area
        
        if intersection_area == 0:
            return 0.0

        return max(intersection_area/poly1.area, intersection_area/poly2.area)

    def sync_callback(self, image_msg, cloud_msg):
        print('Data received')
        pc = ros2_numpy.point_cloud2.point_cloud2_to_array(cloud_msg)
        self.pc = pc['xyz']
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        self.color_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    def camera(self):
        if np.array(self.pc).any() and np.array(self.color_image).any():

            from_frame_rel = self.target_frame
            to_frame_rel = 'rover/map'

            try:
                tf = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info('Could not transform frames.')
                return

            results = self.model.track(self.color_image, persist=True, device="cpu", classes=[39]) #classes=[24,25,26,27,28,32,39,40,41,42,43,44,45,46,47,48,49,56,57,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79])

            # Point cloud cropping using detected masks
            if results[0].boxes.id is not None:
                detected_objects = DetectedObjectsList()
                objetos = []

                _, width = self.color_image.shape[:2]
                masks = results[0].masks.xy
                classes = results[0].boxes.cls
                track_ids = results[0].boxes.id
                img_boxes = results[0].boxes.xyxy.tolist()

                for i, mask in enumerate(masks):
                    # Selecting points corresponding to object i
                    binary_mask = np.zeros(self.color_image.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(binary_mask, [np.array(mask, dtype=np.int32)], 1)
                    indices = np.argwhere(binary_mask == 1)
                    indices_1d = indices[:, 0] * width + indices[:, 1]
                    cropped_vtx = self.pc[indices_1d[::10]]
                    pcmask = (cropped_vtx[:,0] != 0) | (cropped_vtx[:,1] != 0) | (cropped_vtx[:,2] != 0) # Removing points placed in origin
                    cropped_vtx = cropped_vtx[pcmask,:]

                    # The cropped point cloud is evaluated only if it has enough points
                    if len(cropped_vtx)>10:
                        try:
                            # Create o3d point cloud for 3d OBB
                            cloudo3d = o3d.geometry.PointCloud()
                            pcd_points = np.array([(point[0], point[1], point[2]) for point in cropped_vtx], dtype=np.float32)
                            cloudo3d.points = o3d.utility.Vector3dVector(pcd_points)

                            # Filter point cloud
                            cloudo3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05,max_nn=30))
                            cloudo3d.orient_normals_consistent_tangent_plane(100)
                            normal_threshold = 0.4
                            normals = np.asarray(cloudo3d.normals)
                            indices = np.where(np.abs(normals[:,2])>normal_threshold)[0]
                            filt_pcd = cloudo3d.select_by_index(indices)

                            labels = np.array(filt_pcd.cluster_dbscan(eps=0.05,min_points=10, print_progress=False))
                            unique_labels, counts = np.unique(labels[labels !=-1], return_counts=True)
                            
                            if counts.size > 0:
                                largest_cluster_label = unique_labels[np.argmax(counts)]
                                object_indices = np.where(labels == largest_cluster_label)[0]
                                filtered_pcd = filt_pcd.select_by_index(object_indices)

                                filtered_pcd.translate((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
                                R = o3d.geometry.get_rotation_matrix_from_quaternion((tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z))
                                filtered_pcd.rotate(R, center=(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))
                                obb = filtered_pcd.get_oriented_bounding_box()
                                vertices = obb.get_box_points()
                                vertices = np.asarray(vertices)
                                
                                # Extracting geometric characteristics
                                # 2D oriented bounding box
                                vertices2d = np.array([[x, y] for x, y, z in vertices])
                                ca = np.cov(vertices2d,y = None,rowvar = 0,bias = 1)
                                v, vect = np.linalg.eig(ca)
                                tvect = np.transpose(vect)
                                ar = np.dot(vertices2d,np.linalg.inv(tvect))
                                mina = np.min(ar,axis=0)
                                maxa = np.max(ar,axis=0)
                                diff = (maxa - mina)*0.5
                                center = mina + diff
                                corners = np.array([center+[-diff[0],-diff[1]],center+[diff[0],-diff[1]],center+[diff[0],diff[1]],center+[-diff[0],diff[1]],center+[-diff[0],-diff[1]]])
                                corners = np.dot(corners,tvect)
                                corners = corners[0:4]

                                category = self.category_dict.get(int(classes[i].item()), 'None')
                                vertices = np.array(vertices)
                                objetos.append([corners,np.min(vertices[:, 0]),np.max(vertices[:, 0]),np.min(vertices[:, 1]),np.max(vertices[:, 1]),np.min(vertices[:, 2]),np.max(vertices[:, 2]),category])
                                
                                obj = DetectedObject()
                                obj.cls = category
                                obj.trackid = int(track_ids[i].item())
                                obj.pos2d = ROSPolygon(points=
                                    [Point32(x=corners[0][0], y=corners[0][1], z=0.0),
                                    Point32(x=corners[1][0], y=corners[1][1], z=0.0),
                                    Point32(x=corners[2][0], y=corners[2][1], z=0.0),
                                    Point32(x=corners[3][0], y=corners[3][1], z=0.0)
                                ])
                                obj.minh = np.min(vertices[:, 2])
                                obj.maxh = np.max(vertices[:, 2])
                                obj.rel_on = 65535 # Max. value if there is no relationship
                                obj.rel_under = []
                                obj.rel_nearby = []
                                detected_objects.objects.append(obj)

                                #Saving image crop
                                if int(track_ids[i].item()) in self.img_labels: # Already saved object but with bigger size
                                    index = self.img_labels.index(int(track_ids[i].item()))
                                    crop_size = self.img_sizes[index]
                                    x1, y1, x2, y2 = img_boxes[i]
                                    if crop_size < abs((x2-x1)*(y2-y1)):
                                        try:
                                            x1, y1, x2, y2 = img_boxes[i]
                                            cv_image = self.color_image[int(y1):int(y2), int(x1):int(x2)]
                                            filename = f"/home/user/ros2_ws/src/obj_detection/scripts/dataset/image_{int(track_ids[i].item())}.png"
                                            cv2.imwrite(filename, cv_image)
                                            self.img_sizes[index]=abs((x2-x1)*(y2-y1))
                                        except Exception as e:
                                            print(f"ERROR: {type(e).__name__} - {e}")
                                else: # New object
                                    try:
                                        x1, y1, x2, y2 = img_boxes[i]
                                        cv_image = self.color_image[int(y1):int(y2), int(x1):int(x2)]
                                        filename = f"/home/user/ros2_ws/src/obj_detection/scripts/dataset/image_{int(track_ids[i].item())}.png"
                                        cv2.imwrite(filename, cv_image)
                                        self.img_labels.append(int(track_ids[i].item()))
                                        self.img_sizes.append(abs((x2-x1)*(y2-y1)))
                                    except Exception as e:
                                        print(f"ERROR: {type(e).__name__} - {e}")

                        except Exception as e:
                            print(f"ERROR: {type(e).__name__} - {e}")

                if objetos:
                    # Extracting object relationships (support, adjacency)
                    boxes = [item[0] for item in objetos]
                    iou_pairs = []
                    nextto_pairs = []
                    for i in range(len(boxes)):
                        for j in range(i + 1, len(boxes)):
                            iou = self.calculate_iou(boxes[i], boxes[j])
                            if iou > 0.1:
                                iou_pairs.append((i, j))
                            # Adjacency relationship
                            x_interseccion = (objetos[i][2]>=objetos[j][1] and objetos[j][2]>=objetos[i][1]) or (objetos[j][1]-objetos[i][2]<=0.1) or (objetos[i][1]-objetos[j][2]<=0.1)
                            y_interseccion = (objetos[i][4]>=objetos[j][3] and objetos[j][4]>=objetos[i][3]) or (objetos[j][3]-objetos[i][4]<=0.1) or (objetos[i][3]-objetos[j][4]<=0.1)
                            z_interseccion = (objetos[i][6]>=objetos[j][5] and objetos[j][6]>=objetos[i][5]) or (objetos[j][5]-objetos[i][6]<=0.1) or (objetos[i][5]-objetos[j][6]<=0.1)
                            if x_interseccion and y_interseccion and z_interseccion:
                                nextto_pairs.append((i, j))

                    # Support relationship
                    supported_pairs=[]
                    for pair in iou_pairs:
                        if abs(objetos[pair[0]][6] - objetos[pair[1]][5])<0.1 and objetos[pair[0]][6]<objetos[pair[1]][6]: #Object 1 over object 0
                            supported_pairs.append((pair[1],pair[0]))
                        if abs(objetos[pair[0]][5] - objetos[pair[1]][6])<0.1 and objetos[pair[1]][6]<objetos[pair[0]][6]: #Object 0 over object 1
                            supported_pairs.append((pair[0],pair[1]))

                    # Removing repeated relationships (support prevails over adjacency)
                    supported_sets = [set(pair) for pair in supported_pairs]
                    nextto_pairs = [elem for elem in nextto_pairs if set(elem) not in supported_sets]

                    for pair in supported_pairs:
                        print(f"Object {pair[0]} ({objetos[pair[0]][7]}) ON object {pair[1]} ({objetos[pair[1]][7]})")
                        A, B = pair
                        detected_objects.objects[int(A)].rel_on=detected_objects.objects[int(B)].trackid
                        detected_objects.objects[int(B)].rel_under.append(detected_objects.objects[int(A)].trackid)

                    for pair in nextto_pairs:
                        print(f"Object {pair[0]} ({objetos[pair[0]][7]}) and object {pair[1]} ({objetos[pair[1]][7]}) are nearby")
                        A, B = pair
                        detected_objects.objects[int(A)].rel_nearby.append(detected_objects.objects[int(B)].trackid)
                        detected_objects.objects[int(B)].rel_nearby.append(detected_objects.objects[int(A)].trackid)

                    self.pub.publish(detected_objects)
                    print('PUBLISHING OBJECTS...')

        self.color_image = []
        self.pc = []

def main(args=None):
    rclpy.init(args=args)
    realsense = Realsense()

    try:
        rclpy.spin(realsense)
    except KeyboardInterrupt:
        pass
    finally:
        realsense.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()