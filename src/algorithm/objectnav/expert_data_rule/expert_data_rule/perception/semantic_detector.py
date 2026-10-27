"""Frozen GFL/ResNet-50 detector adapter."""

import json
from pathlib import Path

from expert_data_rule.types import Detection


class MMDetSemanticDetector:
    def __init__(self, model_config, checkpoint, classes=("echinus", "rock"),
                 thresholds=None, threshold_file=None,
                 candidate_threshold=0.001, device="cuda:0"):
        from mmdet.apis import DetInferencer

        self.classes = tuple(classes)
        self.thresholds = {name: 0.5 for name in self.classes}
        self.thresholds.update(thresholds or {})
        if threshold_file:
            path = Path(threshold_file)
            if path.is_file():
                record = json.loads(path.read_text(encoding="utf-8"))
                target = str(record.get("target_class", "echinus"))
                if "recommended_threshold" in record and target in self.thresholds:
                    calibrated = record.get("checkpoint")
                    if calibrated and Path(str(calibrated)).name != Path(checkpoint).name:
                        raise ValueError("detector threshold/checkpoint mismatch")
                    self.thresholds[target] = float(record["recommended_threshold"])
        self.inferencer = DetInferencer(
            model=str(model_config), weights=str(checkpoint), device=device)
        model_classes = tuple(
            self.inferencer.model.dataset_meta.get("classes", ()))
        if model_classes and model_classes != self.classes:
            raise ValueError("detector class order mismatch: %r != %r" %
                             (model_classes, self.classes))
        self.candidate_threshold = float(candidate_threshold)
        self.inferencer.model.eval()
        for parameter in self.inferencer.model.parameters():
            parameter.requires_grad_(False)

    def detect(self, image):
        result = self.inferencer(
            image, pred_score_thr=self.candidate_threshold,
            no_save_pred=True, return_datasamples=True)
        prediction = result["predictions"][0].pred_instances.cpu()
        detections = []
        for box, score, label in zip(prediction.bboxes.numpy(),
                                     prediction.scores.numpy(),
                                     prediction.labels.numpy()):
            index = int(label)
            if index < 0 or index >= len(self.classes):
                continue
            name = self.classes[index]
            score = float(score)
            if score >= float(self.thresholds[name]):
                detections.append(Detection(
                    tuple(map(float, box)), score, name))
        return detections
