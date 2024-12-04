def patch() -> None:
    """
    Patch Ultralytics
    Code from https://github.com/ultralytics/ultralytics/issues/2863#issuecomment-2259399447

    Paste this code (control + A the entire file) at the bottom of `/yolovenv/lib/python3.8/site-packages/ultralytics/utils/patches.py`.

    Add the following to the bottom of `/yolovenv/lib/python3.8/site-packages/ultralytics/utils/__init__.py`:
    ```from .patches import patch
    patch()```
    """
    from ultralytics.utils import ops
    import ultralytics.engine.results
    import ultralytics.utils.ops
    def init(self, boxes, orig_shape) -> None:
        """
        Initialize the Boxes class with detection box data and the original image shape.

        Args:
            boxes (torch.Tensor | np.ndarray): A tensor or numpy array with detection boxes.
                Shape can be (num_boxes, 6), (num_boxes, 7), or (num_boxes, 6 + num_classes).
                Columns should contain [x1, y1, x2, y2, confidence, class, (optional) track_id, (optional) class_conf_1, class_conf_2, ...].
            orig_shape (tuple): The original image shape as (height, width). Used for normalization.

        Returns:
            (None)
        """

        if boxes.ndim == 1:
            boxes = boxes[None, :]
        n = boxes.shape[-1]
        super(ultralytics.engine.results.Boxes, self).__init__(boxes, orig_shape)
        self.orig_shape = orig_shape
        self.is_track = False
        self.num_classes = 0

        if n == 6:
            self.format = 'xyxy_conf_cls'
        elif n == 7:
            self.format = 'xyxy_conf_cls_track'
            self.is_track = True
            # self.track_id = boxes[:, -3]
        else:
            self.format = 'xyxy_conf_cls_classconf'
            self.num_classes = n - 6

    ultralytics.engine.results.Boxes.__init__ = init

    from .ops import xywh2xyxy, nms_rotated
    from ultralytics.utils import LOGGER
    import torch
    import time

    def non_max_suppression(
        prediction,
        conf_thres=0.25,
        iou_thres=0.45,
        classes=None,
        agnostic=False,
        multi_label=False,
        labels=(),
        max_det=300,
        nc=0,  # number of classes (optional)
        max_time_img=0.05,
        max_nms=30000,
        max_wh=7680,
        in_place=True,
        rotated=False,
    ):
        """
        Perform non-maximum suppression (NMS) on a set of boxes, with support for masks and multiple labels per box.
        This version returns confidences for all classes.

        Args:
            (... same as before ...)

        Returns:
            (List[torch.Tensor]): A list of length batch_size, where each element is a tensor of
                shape (num_boxes, 6 + num_classes + num_masks) containing the kept boxes, with columns
                (x1, y1, x2, y2, confidence, class, class_conf_1, class_conf_2, ..., mask1, mask2, ...).
        """
        import torchvision

        # Checks and initialization (same as before)
        assert 0 <= conf_thres <= 1, f"Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0"
        assert 0 <= iou_thres <= 1, f"Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0"
        if isinstance(prediction, (list, tuple)):
            prediction = prediction[0]
        if classes is not None:
            classes = torch.tensor(classes, device=prediction.device)

        bs = prediction.shape[0]  # batch size
        nc = nc or (prediction.shape[1] - 4)  # number of classes
        nm = prediction.shape[1] - nc - 4  # number of masks
        mi = 4 + nc  # mask start index
        xc = prediction[:, 4:mi].amax(1) > conf_thres  # candidates

        # Settings
        time_limit = 2.0 + max_time_img * bs  # seconds to quit after
        multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)

        prediction = prediction.transpose(-1, -2)  # shape(1,84,6300) to shape(1,6300,84)
        if not rotated:
            if in_place:
                prediction[..., :4] = xywh2xyxy(prediction[..., :4])  # xywh to xyxy
            else:
                prediction = torch.cat((xywh2xyxy(prediction[..., :4]), prediction[..., 4:]), dim=-1)  # xywh to xyxy

        t = time.time()
        output = [torch.zeros((0, 6 + nc + nm), device=prediction.device)] * bs
        for xi, x in enumerate(prediction):  # image index, image inference
            x = x[xc[xi]]  # confidence

            # Cat apriori labels if autolabelling
            if labels and len(labels[xi]) and not rotated:
                lb = labels[xi]
                v = torch.zeros((len(lb), nc + nm + 4), device=x.device)
                v[:, :4] = xywh2xyxy(lb[:, 1:5])  # box
                v[range(len(lb)), lb[:, 0].long() + 4] = 1.0  # cls
                x = torch.cat((x, v), 0)

            # If none remain process next image
            if not x.shape[0]:
                continue

            # Detections matrix nx(4 + nc + nm) (xyxy, class_conf, cls, masks)
            box, cls_conf, mask = x.split((4, nc, nm), 1)

            # Confidence thresholding
            conf, j = cls_conf.max(1, keepdim=True)
            x = torch.cat((box, conf, j.float(), cls_conf, mask), 1)[conf.view(-1) > conf_thres]

            # Filter by class
            if classes is not None:
                x = x[(x[:, 5:6] == classes).any(1)]

            # Check shape
            n = x.shape[0]  # number of boxes
            if not n:  # no boxes
                continue
            if n > max_nms:  # excess boxes
                x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence and remove excess boxes

            # Batched NMS
            c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
            scores = x[:, 4]  # scores
            if rotated:
                boxes = torch.cat((x[:, :2] + c, x[:, 2:4], x[:, -1:]), dim=-1)  # xywhr
                i = nms_rotated(boxes, scores, iou_thres)
            else:
                boxes = x[:, :4] + c  # boxes (offset by class)
                i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
            i = i[:max_det]  # limit detections

            output[xi] = x[i]
            if (time.time() - t) > time_limit:
                LOGGER.warning(f"WARNING ⚠️ NMS time limit {time_limit:.3f}s exceeded")
                break  # time limit exceeded

        return output

    ops.non_max_suppression = non_max_suppression
