SCORE_MIN_THRESHOLD = 0.01


class MinThreshold:

    @staticmethod
    def process(bboxes, scores, labels):
        # remove with low score
        pick = []
        for i in range(0, len(bboxes)):
            if scores[i] >= SCORE_MIN_THRESHOLD:
                pick.append(i)
        # return stuff
        #print(scores)
        #print(bboxes)
        #print(labels)
        if len(pick) == 0:
            return [], [], []
        #print(pick)
        picked_bboxes = [bboxes[index] for index in pick]
        picked_scores = [scores[index] for index in pick]
        picked_labels = [labels[index] for index in pick]
        return picked_bboxes, picked_scores, picked_labels

