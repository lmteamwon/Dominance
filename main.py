import flight
import queue
from flight import Drone
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from threading import Thread
import sys, cv2, time, pathlib, datetime
# Installed via NVIDIA's website (special TF optimized for nano)
import tensorflow.compat.v1 as tf
# Installed via pip
import imutils, numpy as np
# Installed from TF models git, see readme.md
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

LABELS_PATH = "exported/label_map.pbtxt"
CHKPT_PATH = "exported/frozen_inference_graph.pb"
# Config variables for main()
# Use TF object detection API's box visualization
# (useful for debugging the HUD)
useBoxVisualization = False 

PYLON_CLASS = 1
RING_CLASS = 2

# Minimum confidence for a detection to be considered
# a real detection
MIN_CONFIDENCE = 0.2

# 'enums' for whichever mode we are in
M_MANUAL = 0
M_AUTONAV = 1
M_AUTOMANEUVER = 2
currentMode = M_AUTONAV
# Sent back from flight class
currentAltitude = None

# Causes the Tensorflow detection lists to be printed
# every frame
DEBUG_DUMP_DETECTIONS = False
DEBUG_DISABLE_FLIGHT = False

mainThreadRunning = True
# Camera focal length: 3.04mm
# f-number: 2.0
cam_w = 3280
cam_h = 2464
display_w = 720
display_h = 540

# width of the objects in mm
widthRing = 914.4
widthPylon = 76.2

# camera matrix based on calibration
# (used for distance detection)
cam_focal = 3.04
cam_fx = 2683
cam_fy = 2683 # 2493
cam_cx = 1641
cam_cy = 1214
cam_ppi = cam_fx / cam_focal


# Minimum score used for detected objects
minScore = 30
# currentFrameTime = 0
currentFps = 10
frameQueue = queue.Queue()
frame = None
annotatedFrame = None
xImage = None

# variables for tf object detection
# global so that HUD can access it
boxes = None
scores = None 
stats = None
# Index of currently selected object
selectedIdx = None
selectedClassName = None
objectDistance = 0

category_index = label_map_util.create_category_index_from_labelmap(LABELS_PATH, use_display_name=True)

def gstreamer_pipeline(
    capture_width=cam_w,
    capture_height=cam_h,
    display_width=display_w,
    display_height=display_h, 
    framerate=15,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# Overlay an image with top-left coordinate (x, y) over a background image
def overlayTransparent(background, overlay, x, y):
    background_width = background.shape[1]
    background_height = background.shape[0]
    if x >= background_width or y >= background_height:
        return background
    h, w = overlay.shape[0], overlay.shape[1]
    if x + w > background_width:
        w = background_width - x
        overlay = overlay[:, :w]
    if y + h > background_height:
        h = background_height - y
        overlay = overlay[:h]
    if overlay.shape[2] < 4:
        overlay = np.concatenate(
            [
                overlay,
                np.ones((overlay.shape[0], overlay.shape[1], 1), dtype = overlay.dtype) * 255
            ],
            axis = 2,
        )
    overlay_image = overlay[..., :3]
    mask = overlay[..., 3:] / 255.0
    background[y:y+h, x:x+w] = (1.0 - mask) * background[y:y+h, x:x+w] + mask * overlay_image
    return background

def updateFlightInfo(altitude, mode):
    global currentAltitude, currentMode
    currentAltitude = altitude
    currentMode = mode

# returns the global frame with the HUD overlaid on it
# Note: OpenCV images are by default BGR and *not* RGB
def applyHud():
    global frame, xImage, selectedIdx, selectedClassName, objectDistance, boxes, scores, stats
    global currentMode, currentMode, M_AUTOMANEUVER, M_AUTONAV, M_MANUAL
    if frame is None:
        return None
    if xImage is None:
        xImage = cv2.imread("x.png", cv2.IMREAD_UNCHANGED)
    alpha = 0.7
    text_color = (254, 205, 1)
    line_color = (206, 113, 255)
    bg_color = (30, 1, 10)
    overlay = frame.copy()
    output = frame.copy()
    # top left corner
    cv2.rectangle(overlay, (0, 0), (125, 65), bg_color, -1)
    cv2.putText(overlay, "Team 1", (5, 25), cv2.FONT_HERSHEY_PLAIN, 1.25, text_color, 1)
    cv2.putText(overlay, "%.1f FPS" % currentFps, (5, 55), cv2.FONT_HERSHEY_PLAIN, 1.25, text_color, 1)
    
    # top right corner
    x = display_w - 150
    y = 15
    modeName = "??" 
    if currentMode == M_MANUAL:
        modeName = "MANUAL"
    elif currentMode == M_AUTONAV:
        modeName = "AUTONAV"
    elif currentMode == M_AUTOMANEUVER:
        modeName = "AUTOMANEUVER"
    cv2.rectangle(overlay, (display_w, 0), (display_w - 160, 150), bg_color, -1)
    cv2.putText(overlay, "Mode: " + modeName, (x, y), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1)
    # 2px line
    cv2.rectangle(overlay, (display_w, y + 7), (display_w - 160, y + 8), line_color, -1)
    y = y + 25
    cv2.putText(overlay, "Target: " + (selectedClassName or "??"), (x, y), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1)
    
    # 2px line
    cv2.rectangle(overlay, (display_w, y + 7), (display_w - 160, y + 8), line_color, -1)
    y = y + 25

    conf = None 
    if selectedIdx is not None:
        conf = round(scores[selectedIdx], 2)
    cv2.putText(overlay, "Conf.: " + (str(conf) or "??"), (x, y), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1)

    cv2.rectangle(overlay, (display_w, y + 7), (display_w - 160, y + 8), line_color, -1)
    y = y + 25
    targDistance = None 
    if selectedIdx is not None and stats[selectedIdx] is not int:
        targDistance = round(stats[selectedIdx]['distance'], 1)
    cv2.putText(overlay, "Range: %s ft" % (str(targDistance) or "??"), (x, y), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1)

    cv2.rectangle(overlay, (display_w, y + 7), (display_w - 160, y + 8), line_color, -1)
    y = y + 25
    cv2.putText(overlay, "ETA: 0s", (x, y), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1)

    cv2.rectangle(overlay, (display_w, y + 7), (display_w - 160, y + 8), line_color, -1)
    y = y + 25
    cv2.putText(overlay, "ALT: %s ft" % (str(currentAltitude) or "??"), (x, y), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1)

    # bottom left - current time to see lag
    cv2.rectangle(overlay, (0, display_h), (150, display_h - 30), bg_color, -1)
    cv2.putText(overlay, str(datetime.datetime.now().time())[0:10], (5, display_h - 5), cv2.FONT_HERSHEY_PLAIN, 1.5, text_color, 1)

    # Add X over target
    if selectedIdx is not None:
        x = stats[selectedIdx]['centerX']
        y = stats[selectedIdx]['centerY']
        w, h, channels = xImage.shape
        # Adjust the top-left corner of the x,y such that
        # the X will be placed directly on the center
        x = int(x - w / 2)
        y = int(y - h / 2)
        try:
            overlay = overlayTransparent(overlay, xImage, x, y)
        except:
            # print("X is partially off screen")
            pass
    cv2.addWeighted(overlay, alpha, output, 1 - alpha, 0, output)
    return output

# Turns a class number (int) into a user-friendly string
def classToString(input):
    if input == PYLON_CLASS:
        return "Pylon"
    elif input == RING_CLASS:
        return "Ring"
    else:
        print("classToString() - invalid class " + input)
        return "invalid"
# Finds distance of an object of known width to camera, in mm
# @knownWidth physical width of object in mm
# @observedWidth width of the object in pixels
# returns estimated distance in ft
def distanceToCamera(knownWidth, observedWidth):
    # convert width in pixels to width in image sensor mm
    actualizedWidth = (display_w * cam_ppi) / cam_w
    sizeOnSensor = observedWidth / actualizedWidth
    # 1mm = 0.00328084 ft
    return (knownWidth * cam_focal / sizeOnSensor) * 0.00328084

# Gets the center of a bounding box from TF session
# @box - the individual bounding box
# returns dictionary of geometric info about box
def getBoxStats(box, boxClass):
    ymin = box[0] * display_h
    xmin = box[1] * display_w
    ymax = box[2] * display_h
    xmax = box[3] * display_w
    height = ymax - ymin
    width = xmax - xmin
    centerX = (xmin + xmax) / 2
    centerY = (ymin + ymax) / 2
    area = (xmax - xmin) * (ymax - ymin)
    knownWidth = widthRing
    if boxClass == PYLON_CLASS:
        knownWidth = widthPylon
    distance = distanceToCamera(knownWidth, width)
    data = dict()
    data['area'] = area
    data['height'] = height
    data['width'] = width 
    data['distance'] =  distance
    data['centerX'] = centerX
    data['centerY'] = centerY
    return data

# MJPEG Streaming class
class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print(self.path)
        if self.path.endswith('.ico'):
            # tell the browser we don't have a favicon
            self.send_response(404)
        else:
            # current camera frame
            global frameQueue
            self.send_response(20)
            self.send_header('Expires', '0')
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            #lastFrameTime = currentFrameTime + 3
            # "while true" is OK here cause when the browser closes the stream it will end this loop as well
            while True:
                try:
                    if frameQueue.empty():
                        time.sleep(0.05)
                        continue
                    newFrame = frameQueue.get_nowait()
                    # only send a frame when we have a new frame to send
                    if newFrame is None:
                        continue
                    r, buf = cv2.imencode(".jpg", newFrame)
                    self.wfile.write("--jpgboundary\r\n".encode())
                    byteData = bytearray(buf)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(byteData))
                    self.end_headers()
                    self.wfile.write(byteData)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.01)
                except KeyboardInterrupt:
                    sys.exit()
                except queue.Empty:
                    time.sleep(0.05)
                    continue
            return

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""
 
def main():
    # current camera frame
    global frame, annotatedFrame, frameQueue, currentFps, selectedIdx, selectedClassName, objectDistance, boxes, scores, stats
    global currentMode, M_AUTOMANEUVER, M_AUTONAV, M_MANUAL
    
    # print(cv2.getBuildInformation())
    print("Loading model")
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(CHKPT_PATH, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

    label_map = label_map_util.load_labelmap(LABELS_PATH)
    categories = label_map_util.convert_label_map_to_categories(
        label_map, max_num_classes=2, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    print("Starting main python module")
    if not DEBUG_DISABLE_FLIGHT:
        flightData = Drone(updateFlightInfo)
        process = Thread(target = flight.flightMain, args= (flightData,))
        process.start()
    ip = '0.0.0.0'
    server = ThreadedHTTPServer((ip, 9090), CamHandler)
    target = Thread(target=server.serve_forever,args=())
    i = 0

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    #print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    fpsSmoothing = 70
    lastUpdate = time.time()
    try:
        if cap.isOpened():
            print("CSI Camera opened")
            graph_options = tf.GraphOptions(
                    optimizer_options=tf.OptimizerOptions(
                        opt_level=tf.OptimizerOptions.L1,
                    )
                )
            OptConfig = tf.ConfigProto(
                graph_options=graph_options
            )
            with detection_graph.as_default():
                with tf.Session(graph=detection_graph, config=OptConfig) as sess:
                    # Definite input and output Tensors for detection_graph
                    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                    # Each box represents a part of the image where a particular object
                    # was detected.
                    detection_boxes = detection_graph.get_tensor_by_name(
                        'detection_boxes:0')
                    # Each score represent how level of confidence for each of the objects.
                    # Score is shown on the result image, together with the class
                    # label.
                    detection_scores = detection_graph.get_tensor_by_name(
                        'detection_scores:0')
                    detection_classes = detection_graph.get_tensor_by_name(
                        'detection_classes:0')
                    num_detections = detection_graph.get_tensor_by_name(
                        'num_detections:0')
                    i = 0
                    print("TensorFlow session loaded.")
                    while mainThreadRunning:
                        ret_val, img = cap.read()
                        frame = img
                        # convert OpenCV's BGR to RGB as the model
                        # was trained on RGB images
                        color_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        # resize image to model size of 360x270
                        color_frame = cv2.resize(color_frame, (360, 270), interpolation = cv2.INTER_CUBIC)
                        image_np_expanded = np.expand_dims(color_frame, axis=0)
                        # Actual detection
                        (boxes, scores, classes, num) = sess.run(
                            [detection_boxes, detection_scores,
                                detection_classes, num_detections],
                            feed_dict={image_tensor: image_np_expanded})

                        # Draw boxes using TF library, should be off during competition
                        if useBoxVisualization:
                            vis_util.visualize_boxes_and_labels_on_image_array(
                                frame, np.squeeze(boxes),
                                np.squeeze(classes).astype(np.int32),
                                np.squeeze(scores), category_index,
                                use_normalized_coordinates=True,
                                line_thickness=4, min_score_thresh=MIN_CONFIDENCE)

                        # Now that we have the detected BBoxes, it's time to determine our current obstacle
                        # First, gather stats about the bounding boxes
                        # squeezing makes it so you can do access box[i] directly instead of having to
                        # access box[0][i]
                        boxes = np.squeeze(boxes)
                        classes = np.squeeze(classes)
                        scores = np.squeeze(scores)
                        stats = []
                        j = 0
                        # This is 15ft, any object farther than that is a misidentification
                        lowestDistance = 15

                        if DEBUG_DUMP_DETECTIONS:
                            print("Boxes // Classes // Scores")
                            print(boxes)
                            print(classes)
                            print(scores)
                        # Reset selections
                        selectedIdx = None
                        if len(boxes) > 0:
                            for j in range(0, len(boxes)):
                                if scores[j] >= MIN_CONFIDENCE:
                                    stats.insert(j, getBoxStats(boxes[j], classes[j]))
                                    # print("box[%d] distance is %f" % (j, stats[j]['distance']))
                                    if stats[j]['distance'] < lowestDistance:
                                        selectedIdx = j
                                        selectedClassName = classToString(classes[j])
                                        objectDistance = stats[j]['distance']
                                        lowestDistance = objectDistance
                                        #print("Selected box[%d]: distance %f class %s conf %f" % (j, objectDistance, selectedClassName, scores[j]))
                                else:
                                    # Skip calculations on this box if it does not meet
                                    # confidence threshold
                                    stats.insert(j, 0)
                        if not DEBUG_DISABLE_FLIGHT:
                            if selectedIdx is not None:
                                flightData.upData(stats[selectedIdx], selectedClassName)
                            else:
                                flightData.upData(None, "None")
                            
                        # add the HUD to the current image
                        annotatedFrame = applyHud()
                        # currentFrameTime = time.time()
                        #if frameQueue.full():
                        #    with frameQueue.mutex:
                        #        frameQueue.queue.clear()
                        frameQueue.put(annotatedFrame.copy())
                        if i == 0:
                            target.start()
                            print("Starting MJPEG stream")
                        i += 1
                        # FPS smoothing algorithm
                        frameTime = time.time() - lastUpdate
                        frameFps = 1 / frameTime
                        currentFps += (frameFps - currentFps) / fpsSmoothing
                        lastUpdate = time.time()

                    cap.release()
        else:
            print("FATAL: Unable to open camera")
        
    except KeyboardInterrupt:
        sys.exit()

if __name__ == '__main__':
    main()
