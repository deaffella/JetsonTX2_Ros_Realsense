import cv2
from mount_dir.realsense.utils.realsense_cam import DepthCamera



def show_distance(event, x, y, args, params):
    global point
    point = (x, y)


if __name__ == '__main__':

    camera = DepthCamera(serial_number='f1230676')


    cv2.namedWindow("Color frame")
    cv2.setMouseCallback("Color frame", show_distance)
    point = (400, 300)
    while True:
        ret, depth_frame, color_frame = camera.get_frame()

        # Show distance for a specific point
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        distance = depth_frame[point[1], point[0]]

        cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

        cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Color frame", color_frame)
        key = cv2.waitKey(1)
        if key == 27:
            camera.release()
            break
