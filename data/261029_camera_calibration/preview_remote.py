import cv2,sys,json,time
cap=cv2.VideoCapture('/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0',cv2.CAP_V4L2)
if not cap.isOpened():raise RuntimeError('Arducam 열기 실패')
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'));cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080);cap.set(cv2.CAP_PROP_FPS,30)
frame=None
for _ in range(12):
 ok,im=cap.read()
 if ok:frame=im
cap.release()
if frame is None:raise RuntimeError('영상 수신 실패')
print(json.dumps(dict(width=frame.shape[1],height=frame.shape[0],opencv=cv2.__version__,time=time.time())),file=sys.stderr)
ok,out=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,95]);assert ok;sys.stdout.buffer.write(out.tobytes())
