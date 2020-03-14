import cv2

# acd999
COLOR = (172, 217, 153)


def draw_bbox(dst, bbox, score, label="object"):
    height, width = dst.shape[:2]
    # scale the bounding box from the range [0,1] to [0, width], [0, height]
    (min_y, min_x, max_y, max_x) = bbox
    min_x = int(min_x * width)
    min_y = int(min_y * height)
    max_x = int(max_x * width)
    max_y = int(max_y * height)
    # draw the prediction on the output image
    label = "{}: {:.2f}".format(label, score)
    cv2.rectangle(dst, (min_x, min_y), (max_x, max_y), COLOR, 2)
    y = min_y - 10 if min_y - 10 > 10 else min_y + 10
    cv2.putText(dst, label, (min_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, COLOR, 1)
