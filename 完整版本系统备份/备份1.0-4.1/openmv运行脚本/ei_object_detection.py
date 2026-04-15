import sensor
import time
import ml
from ml.utils import NMS
import math
import image
from machine import UART


# UART link:
# STM32 -> OpenMV : TARGET:n
# OpenMV -> STM32 : X:123,Y:45,A:18 or X:-1,Y:-1,A:-1
uart = UART(3, baudrate=115200)


# Target labels must match model.labels.
target_labels = ["toilet", "exit", "office", "stairs", "elevator", "extinguisher", None]

# Default mode: toilet
target_mode = 0
target_label = target_labels[target_mode]

# Temporal confidence fusion parameters
fusion_alpha = 0.30
fusion_beta = 0.25
fusion_gamma = 0.15
fusion_delta = 0.15
fusion_epsilon = 0.15

fusion_streak_norm_frames = 5.0
fusion_position_delta_max = 40.0
fusion_area_delta_max_permille = 12.0
fusion_label_history_size = 5
fusion_confirm_threshold = 0.72
fusion_confirm_frames = 3
fusion_lost_threshold = 0.28
fusion_lost_frames = 4

# Detection / tracking state
target_confirmed = False
target_detect_streak = 0
fusion_confirm_count = 0
fusion_lost_count = 0
candidate_prev_center_x = -1
candidate_prev_center_y = -1
candidate_prev_area_permille = -1
last_confirmed_center_x = -1
last_confirmed_center_y = -1
last_confirmed_area_permille = -1
label_history = []
debug_frame_count = 0

# Detection threshold
min_confidence = 0.4
threshold_list = [(math.ceil(min_confidence * 255), 255)]
frame_area = 320.0 * 240.0


# Camera init
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240
sensor.skip_frames(time=2000)


# Load model
model = ml.Model("trained")
print("model labels:", model.labels)
print("OpenMV ready, default mode ->", target_mode, target_label)

colors = [
    (255, 0, 0),
    (0, 255, 0),
    (255, 255, 0),
    (0, 0, 255),
    (255, 0, 255),
    (0, 255, 255),
    (255, 255, 255),
]


def fomo_post_process(model, inputs, outputs):
    n, oh, ow, oc = model.output_shape[0]
    nms = NMS(ow, oh, inputs[0].roi)

    for i in range(oc):
        heatmap = image.Image(outputs[0][0, :, :, i] * 255)

        blobs = heatmap.find_blobs(
            threshold_list,
            x_stride=1,
            area_threshold=1,
            pixels_threshold=1
        )

        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect

            score = heatmap.get_statistics(
                thresholds=threshold_list,
                roi=rect
            ).l_mean() / 255.0

            nms.add_bounding_box(x, y, x + w, y + h, score, i)

    return nms.get_bounding_boxes()


def clamp01(value):
    if value < 0.0:
        return 0.0

    if value > 1.0:
        return 1.0

    return value


def reset_fusion_state(clear_history=True):
    global target_confirmed, target_detect_streak
    global fusion_confirm_count, fusion_lost_count
    global candidate_prev_center_x, candidate_prev_center_y, candidate_prev_area_permille
    global last_confirmed_center_x, last_confirmed_center_y, last_confirmed_area_permille
    global label_history, debug_frame_count

    target_confirmed = False
    target_detect_streak = 0
    fusion_confirm_count = 0
    fusion_lost_count = 0
    candidate_prev_center_x = -1
    candidate_prev_center_y = -1
    candidate_prev_area_permille = -1
    last_confirmed_center_x = -1
    last_confirmed_center_y = -1
    last_confirmed_area_permille = -1

    if clear_history:
        label_history = []
        debug_frame_count = 0


def append_label_history(label_name):
    global label_history

    label_history.append(label_name)

    while len(label_history) > fusion_label_history_size:
        label_history.pop(0)


def compute_confidence_score(score):
    if score < min_confidence:
        return 0.0

    return clamp01((score - min_confidence) / (1.0 - min_confidence))


def compute_streak_score(streak_count):
    return clamp01(streak_count / fusion_streak_norm_frames)


def compute_position_score(center_x, center_y):
    if (candidate_prev_center_x < 0) or (candidate_prev_center_y < 0):
        return 0.5

    dx = center_x - candidate_prev_center_x
    dy = center_y - candidate_prev_center_y
    delta_pos = math.sqrt((dx * dx) + (dy * dy))

    return clamp01(1.0 - (delta_pos / fusion_position_delta_max))


def compute_label_consistency(target_name):
    if not label_history:
        return 0.0

    match_count = 0

    for label_name in label_history:
        if label_name == target_name:
            match_count += 1

    return match_count / len(label_history)


def compute_area_score(area_permille):
    if candidate_prev_area_permille < 0:
        return 0.5

    delta_area = abs(area_permille - candidate_prev_area_permille)
    return clamp01(1.0 - (delta_area / fusion_area_delta_max_permille))


def compute_fusion_score(confidence_score, streak_score, position_score, label_score, area_score):
    return ((fusion_alpha * confidence_score) +
            (fusion_beta * streak_score) +
            (fusion_gamma * position_score) +
            (fusion_delta * label_score) +
            (fusion_epsilon * area_score))


def print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=False):
    global debug_frame_count

    debug_frame_count += 1

    if (not force) and ((debug_frame_count % 5) != 0):
        return

    print("mode:%s state:%s Ct:%.2f Kt:%.2f Pt:%.2f Mt:%.2f At:%.2f St:%.2f c:%d l:%d fps:%.2f" %
          (target_label, state_name, ct, kt, pt, mt, at, st,
           fusion_confirm_count, fusion_lost_count, clock.fps()))


def handle_target_mode_cmd():
    global target_mode, target_label

    while uart.any():
        cmd = uart.readline()
        if not cmd:
            return

        try:
            cmd = cmd.decode().strip()

            if cmd.startswith("TARGET:"):
                mode = int(cmd.split(":")[1])

                if 0 <= mode < len(target_labels):
                    target_mode = mode
                    target_label = target_labels[target_mode]
                    reset_fusion_state(clear_history=True)
                    print("Switch mode ->", target_mode, target_label)

        except Exception as exc:
            print("UART cmd error:", exc)


clock = time.clock()

while True:
    clock.tick()

    # Read mode command from STM32 first.
    handle_target_mode_cmd()

    # Capture one frame.
    img = sensor.snapshot()

    # None mode
    if target_label is None:
        reset_fusion_state(clear_history=True)
        img.draw_string(2, 2, "Mode: None", color=(255, 255, 0), scale=1)
        img.draw_string(2, 18, "No Target", color=(255, 0, 0), scale=1)
        uart.write("X:-1,Y:-1,A:-1\r\n")
        print_fusion_log("LOST", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, force=False)
        continue

    detections = model.predict([img], callback=fomo_post_process)

    best_detection = None
    best_score = -1.0
    best_class_index = -1
    global_best_label = None
    global_best_score = -1.0

    for i, detection_list in enumerate(detections):
        if i >= len(model.labels):
            continue

        label_name = model.labels[i]

        if isinstance(label_name, str) and label_name.lower() in ["background", "_background_"]:
            continue

        for (x, y, w, h), score in detection_list:
            if score > global_best_score:
                global_best_score = score
                global_best_label = label_name

            if (label_name == target_label) and (score >= min_confidence) and (score > best_score):
                best_score = score
                best_detection = (x, y, w, h)
                best_class_index = i

    append_label_history(global_best_label)

    ct = 0.0
    kt = 0.0
    pt = 0.0
    mt = compute_label_consistency(target_label)
    at = 0.0
    st = 0.0
    state_name = "LOST"

    if best_detection is not None:
        target_detect_streak += 1

        x, y, w, h = best_detection
        center_x = math.floor(x + (w / 2))
        center_y = math.floor(y + (h / 2))
        area_permille = int(((w * h) * 1000.0) / frame_area)
        color = colors[best_class_index % len(colors)]

        ct = compute_confidence_score(best_score)
        kt = compute_streak_score(target_detect_streak)
        pt = compute_position_score(center_x, center_y)
        at = compute_area_score(area_permille)
        st = compute_fusion_score(ct, kt, pt, mt, at)

        candidate_prev_center_x = center_x
        candidate_prev_center_y = center_y
        candidate_prev_area_permille = area_permille

        img.draw_rectangle(x, y, w, h, color=color, thickness=2)
        img.draw_cross(center_x, center_y, color=color, size=8)
        img.draw_string(2, 2, "Target: %s" % target_label, color=(255, 255, 0), scale=1)
        img.draw_string(
            x,
            max(0, y - 12),
            "%s %.2f" % (target_label, best_score),
            color=color,
            scale=1
        )

        if target_confirmed:
            if st <= fusion_lost_threshold:
                fusion_lost_count += 1

                if (fusion_lost_count >= fusion_lost_frames) or (last_confirmed_center_x < 0) or (last_confirmed_center_y < 0):
                    reset_fusion_state(clear_history=False)
                    state_name = "LOST"
                    img.draw_string(2, 18, "Lost Target", color=(255, 0, 0), scale=1)
                    uart.write("X:-1,Y:-1,A:-1\r\n")
                    print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=True)
                else:
                    state_name = "HOLD"
                    img.draw_string(2, 18, "Hold Target", color=(0, 255, 255), scale=1)
                    uart.write("X:%d,Y:%d,A:%d\r\n" % (last_confirmed_center_x, last_confirmed_center_y, last_confirmed_area_permille))
                    print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=False)
            else:
                fusion_lost_count = 0
                last_confirmed_center_x = center_x
                last_confirmed_center_y = center_y
                last_confirmed_area_permille = area_permille
                state_name = "CONFIRMED"
                img.draw_string(2, 18, "Confirmed", color=(0, 255, 0), scale=1)
                uart.write("X:%d,Y:%d,A:%d\r\n" % (center_x, center_y, area_permille))
                print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=False)
        else:
            fusion_lost_count = 0

            if st >= fusion_confirm_threshold:
                fusion_confirm_count += 1
            else:
                fusion_confirm_count = 0

            if fusion_confirm_count >= fusion_confirm_frames:
                target_confirmed = True
                fusion_lost_count = 0
                last_confirmed_center_x = center_x
                last_confirmed_center_y = center_y
                last_confirmed_area_permille = area_permille
                state_name = "CONFIRMED"
                img.draw_string(2, 18, "Confirmed", color=(0, 255, 0), scale=1)
                uart.write("X:%d,Y:%d,A:%d\r\n" % (center_x, center_y, area_permille))
                print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=True)
            else:
                state_name = "CANDIDATE"
                img.draw_string(2, 18, "Candidate", color=(255, 255, 0), scale=1)
                uart.write("X:-1,Y:-1,A:-1\r\n")
                print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=False)
    else:
        target_detect_streak = 0
        fusion_confirm_count = 0
        candidate_prev_center_x = -1
        candidate_prev_center_y = -1
        candidate_prev_area_permille = -1
        st = compute_fusion_score(ct, kt, pt, mt, at)

        img.draw_string(2, 2, "Target: %s" % target_label, color=(255, 255, 0), scale=1)

        if target_confirmed and (last_confirmed_center_x >= 0) and (last_confirmed_center_y >= 0):
            fusion_lost_count += 1

            if fusion_lost_count >= fusion_lost_frames:
                reset_fusion_state(clear_history=False)
                state_name = "LOST"
                img.draw_string(2, 18, "Lost Target", color=(255, 0, 0), scale=1)
                uart.write("X:-1,Y:-1,A:-1\r\n")
                print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=True)
            else:
                state_name = "HOLD"
                img.draw_string(2, 18, "Hold Target", color=(0, 255, 255), scale=1)
                uart.write("X:%d,Y:%d,A:%d\r\n" % (last_confirmed_center_x, last_confirmed_center_y, last_confirmed_area_permille))
                print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=False)
        else:
            fusion_lost_count = 0
            state_name = "LOST"
            img.draw_string(2, 2, "Target: %s" % target_label, color=(255, 255, 0), scale=1)
            img.draw_string(2, 18, "No Target", color=(255, 0, 0), scale=1)
            uart.write("X:-1,Y:-1,A:-1\r\n")
            print_fusion_log(state_name, ct, kt, pt, mt, at, st, force=False)
