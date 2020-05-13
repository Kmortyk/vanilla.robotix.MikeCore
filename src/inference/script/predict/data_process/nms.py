import numpy as np

OVERLAP_THRESHOLD = 0.4


class NonMaximumSuppression:

    @staticmethod
    def process(bboxes, scores, labels):
        if len(bboxes) == 0:
            return [], [], []

        pick = []
        # get components
        x_mins = bboxes[:, 0]
        y_mins = bboxes[:, 1]
        x_maxs = bboxes[:, 2]
        y_maxs = bboxes[:, 3]
        # calculate areas and sort by max y
        area = (x_maxs - x_mins + 1) * (y_maxs - y_mins + 1)
        ids = np.argsort(y_maxs)

        while len(ids) > 0:

            last = len(ids) - 1
            i = ids[last]
            pick.append(i)

            xx1 = np.maximum(x_mins[i], x_mins[ids[:last]])
            yy1 = np.maximum(y_mins[i], y_mins[ids[:last]])
            xx2 = np.minimum(x_maxs[i], x_maxs[ids[:last]])
            yy2 = np.minimum(y_maxs[i], y_maxs[ids[:last]])

            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)
            # calculate overlap area
            overlap = (w * h) / area[ids[:last]]

            ids = np.delete(ids, np.concatenate(([last], np.where(overlap > OVERLAP_THRESHOLD)[0])))

        return bboxes[pick], scores[pick], labels[pick]

