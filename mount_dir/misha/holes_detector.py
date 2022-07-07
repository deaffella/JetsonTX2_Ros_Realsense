import numpy as np
import torch
import cv2

import time

def to_tensor(img):
    img = np.float32(img/255)
    return torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0)

def get_contours(mask):
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


class PixelCoords2mmConverter:
    def __init__(self, overall_rectangle_size, working_frame_coords, img_shape):
        '''
        overall_rectangle_size:array-like
            размер рабочей области в миллиметрах в формате (ширина, высота)
        working_frame_coords:array-like
            координаты рабочей области детекции в миллиметрах в формате (x0, y0, x1, y1)
        img_shape:array-like
            размер изображения в формате (ширина, высота)
        '''
        #self.overall_rectangle_size = overall_rectangle_size

        assert len(working_frame_coords)==4 
        working_frame_coords = np.array(working_frame_coords).reshape(2, 2)
        # check if the working frame is inside the overall rectangle
        self.working_frame_coords = np.clip(working_frame_coords, (0, 0), overall_rectangle_size)
        self.img_shape = img_shape
        (x0, y0), (x1, y1) = self.working_frame_coords
        img_cols, img_rows = img_shape

        self.step = np.array([(x1-x0)/img_cols, (y1-y0)/img_rows])

    def __call__(self, pix_coords):
        '''
        pix_coords: array-like
            список или numpy.array с координатами центров детектированных нор
        '''
        x0y0_coord = self.working_frame_coords[0]
        #print(pix_coords)

        center_coords = np.multiply(pix_coords, self.step) + x0y0_coord

        #x, y = int(x_pix*self.x_step)+x0, int(y_pix*self.y_step)+y0
        return np.round(center_coords).astype(int)

class ContoursCentersGenerator:
    def __init__(
        self,
        path_to_saved_model,
        device='cpu',
        nn_input_size=(256, 256),
        img_norm_mean=(0, 0, 0),
        img_norm_std=(1, 1, 1),
        processing_type=np.float32
        ):
        self.model = torch.jit.load(path_to_saved_model).to(device).eval()
        #self.model = smp.UNet(classes=2).to(device).eval()
        #self.model = UNet2(in_channels=3, class_num=2).to(device).eval()
        self.device = device
        self.nn_input_size = nn_input_size
        self.img_norm_mean = img_norm_mean
        self.img_norm_std = img_norm_std
        self.processing_type = processing_type

    def to_tensor(self, img):
        '''
        Нормализация и преобразование изображения в torch.tensor
        '''
        if self.processing_type != np.uint8:
            img = img/255

        # normalize
        img = np.subtract(img, self.img_norm_mean)
        img = np.divide(img, self.img_norm_std)

        img = self.processing_type(img)
        return torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0)

    def filter_contours(self, contours):
        '''
        Дописать функционал фильтрации, когда будет тестирование в реальных условиях
        '''
        return contours

    def compute_centers(self, contours):
        centers = []
        for contour in contours:
            moments = cv2.moments(contour)
            try:
                cX = int(moments["m10"] / moments["m00"])
                cY = int(moments["m01"] / moments["m00"])
                centers.append([cX, cY]) 
            except ZeroDivisionError:
                continue

        return centers

    def __call__(self, img):
        
        img = self.to_tensor(cv2.resize(img, self.nn_input_size))

        with torch.no_grad():
            img = img.to(self.device)
            mask = torch.argmax(self.model(img), dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
            
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        centers = []
        contours = self.filter_contours(contours)
        #print(contours)
        centers = self.compute_centers(contours)
        #print(centers)

        return centers, contours


class HolesDetector:
    def __init__(self, video_device, overall_area_size, working_frame_coords, path_to_saved_model, nn_device):
        video_device = 'v4l2src device=/dev/video{} ! video/x-raw, width=(int){}, height=(int){}, framerate=(fraction){}/1 ! videoconvert !  video/x-raw, format=(string)BGR ! appsink'.format(video_device, 640, 480, 30)
        
        self.cap = cv2.VideoCapture(video_device, cv2.CAP_GSTREAMER)


        self.generate_contours_centers = ContoursCentersGenerator(
            path_to_saved_model=path_to_saved_model,
            device=nn_device
        )
        self.pixel_coords_converter = PixelCoords2mmConverter(
            overall_rectangle_size=overall_area_size,
            working_frame_coords=working_frame_coords,
            img_shape=(256, 256))

    def __call__(self):
        center_coords = ()
        ret, img = self.cap.read()
        if not ret:
            return center_coords

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        centers, contours = self.generate_contours_centers(img)
        #print(centers)
        # координаты в миллиметрах
        if len(centers) > 0:
            center_coords = self.pixel_coords_converter(centers)
            if len(center_coords) > 1:
                center_coords = np.array(center_coords)
                center_coords[1:] = center_coords[1:] - center_coords[:-1]
                center_coords = center_coords.tolist()
        else:
            center_coords = []

        return center_coords


if __name__ == '__main__':    
    detector = HolesDetector(
        video_device=1,#!!!!
        overall_area_size=(300, 140),
        working_frame_coords=(80, 100, 80, 280),
        path_to_saved_model='pretrained_custom_unet_scripted.pt',
        nn_device='cpu' # поменять на cuda
    )

    cnt = 0
    times_list = []
    while True:
        if cnt == 200:
            break
        
        t0 = time.time()

        center_coords_list = detector()
        print(center_coords_list)

        if len(center_coords_list) > 0:
            # запуск изничтожителя
            pass

        t1 = time.time()

        times_list.append(t1-t0)

        # Display the resulting frame
        '''
        #cv2.imshow('{}'.format(img.shape),img)

        # Press Q on keyboard to  exit
        
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
        '''
        cnt+=1



    mean_t = np.mean(times_list)
    std_t = np.std(times_list)

    print('mean time = {:.3f} s; std = {:.3f} s'.format(mean_t, std_t))
