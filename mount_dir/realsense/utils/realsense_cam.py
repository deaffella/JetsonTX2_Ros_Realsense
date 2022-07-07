import pyrealsense2 as rs
import numpy as np
from typing import List, Tuple
import cv2

from mount_dir.realsense.utils.translit import translit_ru_to_eng

def get_connected_realsense_ids() -> List[Tuple[str, str]]:
    realsense_ctx = rs.context()
    connected_devices = []
    for i in range(len(realsense_ctx.devices)):
        detected_camera_serial_number = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
        detected_camera_serial_name = realsense_ctx.devices[i].get_info(rs.camera_info.name)
        connected_devices.append((str(detected_camera_serial_number), str(detected_camera_serial_name)))
    return connected_devices





class DepthCamera:
    def __init__(self,
                 serial_number: str or None = None):
        self.serial_number = serial_number

        self.colorizer = rs.colorizer()
        self.colorizer.set_option(rs.option.visual_preset, 2) # 0=Dynamic, 1=Fixed, 2=Near, 3=Far


        self.align = rs.align(rs.stream.color)
        # self.align = rs.align(rs.stream.depth)

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        if type(self.serial_number) == str:
            self.config.enable_device(self.serial_number)

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        # device = pipeline_profile.get_device()
        # device_product_line = str(device.get_info(rs.camera_info.product_line))

        #self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth)
        self.config.enable_stream(rs.stream.color)


        # Start streaming
        self.pipeline.start(self.config)

    def get_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            frames = self.align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            if not depth_frame or not color_frame:
                return False, None, None
            return True, depth_image, color_image
        except:
            return False, None, None

    def release(self):
        self.pipeline.stop()


def draw_bboxes(frame: np.ndarray,
                bboxes: List[List[float]],
                color: List[int] = [0, 255, 0],
                ) -> np.ndarray:
    """
    Draw bounding boxes and face landmarks on image.

    :param frame:
    :param bboxes:
    :param color:
    :return:
    """
    if len(np.shape(frame)) == 2:
        color = 255

    for single_bbox in bboxes:
        x0, y0, x1, y1 = single_bbox[:4]
        cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
    return frame


def draw_annotations_to_bboxes(frame: np.ndarray,
                               bboxes: List[List[float]],
                               annotations: List[str],
                               color: List[int] = [0, 255, 0],
                               ) -> np.ndarray:
    if len(np.shape(frame)) == 2:
        color = 255

    for single_bbox, single_annotation in zip(bboxes, annotations):
        x0, y0, x1, y1 = single_bbox[:4]
        cv2.putText(frame,
                    str(single_annotation),
                    (x0 - 5, y0 - 10),
                    cv2.FONT_HERSHEY_PLAIN,
                    color=color,
                    lineType=2,
                    fontScale=1.2)
    return frame

def draw_face_data(frame: np.ndarray,
                   bboxes: List[List[float]],
                   depths: List[int],
                   recognized_labels: List[Tuple[int,str,float]],
                   color: List[int] = [0, 255, 0],
                   ):
    if len(np.shape(frame)) == 2:
        color = 255



    for single_bbox, \
        depth, \
        (person_id, person_name, feat_distance) in \
            zip(
                bboxes,
                depths,
                recognized_labels,
    ):
        x0, y0, x1, y1 = single_bbox[:4]
        if person_name =='GUEST':
            color = [0, 0, 255]

        text = [
            f'id: {person_id}',
            f'depth: {depth}mm',
            f'd: {round(feat_distance, 2)}',
            f'name:',
        ] + translit_ru_to_eng(person_name).split(' ')
        for line_idx, line in enumerate(text):
            cv2.putText(frame,
                        line,
                        (x1 + 5, y0 + 12 + line_idx*20),
                        cv2.FONT_HERSHEY_PLAIN,
                        color=color,
                        lineType=2,
                        fontScale=1)
    return frame


def process_bboxes_to_squares(bboxes: List[List[float]],
                              frame: np.ndarray,
                              crop_margin: int = 0,
                              ) -> List[List[int]]:
    frame_size = np.asarray(frame.shape)[0:2]
    processed = [
        [
            int(np.maximum(int(single_bbox[0]) - crop_margin / 2, 0)),
            int(np.maximum(int(single_bbox[1]) - crop_margin / 2, 0)),
            int(np.minimum(int(single_bbox[2]) + crop_margin / 2, frame_size[1])),
            int(np.minimum(int(single_bbox[3]) + crop_margin / 2, frame_size[0])),
         ]
        for single_bbox in bboxes
    ]

    # [ [x0, y0,  x1, y1],  ]
    return processed


def fixed_image_standardization(image_tensor):
    processed_tensor = (image_tensor - 127.5) / 128.0
    return processed_tensor



def crop_single_image_by_bbox(frame: np.ndarray,
                              single_bbox: List[int],
                              face_crop_size: int = 160,
                              ) -> np.ndarray:
    # [x0, y0,  x1, y1]
    x0, y0, x1, y1 = single_bbox[:4]

    # depth frame
    if len(np.shape(frame)) == 2:
        y_center = int((y1 - y0)/2 + y0)
        x_center = int((x1 - x0)/2 + x0)
        # cropped_frame = frame[y0:y1, x0:x1]
        cropped_frame = frame[y_center-2:y_center+2, x_center-2:x_center+2]

    # color frame
    else:
        cropped = frame[y0:y1, x0:x1, :]
        cropped_frame = cv2.resize(cropped, dsize=(face_crop_size, face_crop_size), interpolation=cv2.INTER_AREA)
        cropped_frame = fixed_image_standardization(cropped_frame)
    return cropped_frame


def extract_faces(frame: np.ndarray,
                  bboxes: List[List[int]]):
    return [crop_single_image_by_bbox(frame=frame, single_bbox=bbox) for bbox in bboxes]


def __calculate_mean_distance_in_cropped_depth(cropped_depth: np.ndarray) -> float:
    return int(np.mean(cropped_depth))


def calculate_face_depths(depth_frame: np.ndarray,
                          bboxes: List[List[int]]):
    depths_mean = [
        __calculate_mean_distance_in_cropped_depth(
            cropped_depth=crop_single_image_by_bbox(
                frame=depth_frame,
                single_bbox=bbox
            )
        ) for bbox in bboxes]
    return depths_mean







# def extract_cropped_images_by_bboxes(frame: np.ndarray,
#                                      squared_bboxes: List[List[float]],
#                                      ) -> List[ Tuple[ np.ndarray, np.ndarray ] ]:
#     return [crop_single_squared_image_by_bbox(frame=frame,
#                                               single_face_bbox=single_face_bbox) for single_face_bbox in squared_bboxes]


