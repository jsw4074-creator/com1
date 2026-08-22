import cv2


def is_intersection(mask, threshold=0.15):
    """마스크에서 교차점 여부를 판단."""
    ratio = cv2.countNonZero(mask) / mask.size
    return ratio > threshold


def filter_contours_by_mode(contours, is_vertical):
    """모드에 맞는 컨투어만 반환."""
    valid = []
    for cnt in contours:
        x_, y_, w_, h_ = cv2.boundingRect(cnt)
        if w_ == 0 or h_ == 0:
            continue
        ratio = w_ / h_
        if is_vertical and ratio < 0.3:
            valid.append(cnt)
        elif not is_vertical and ratio > 3.0:
            valid.append(cnt)
    return valid
