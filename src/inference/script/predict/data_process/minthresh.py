SCORE_MIN_THRESHOLD = 0.5


class MinThreshold:

    @staticmethod
    def process(bboxes, scores, labels):
        # remove with low score
        pick = []
        for i in range(0, len(bboxes)):
            if scores[i] >= SCORE_MIN_THRESHOLD:
                pick.append(i)
        # return stuff
        return bboxes[pick], scores[pick], labels[pick]
