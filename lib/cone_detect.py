# encoding: utf-8
import os
import time
from collections import deque

import cv2
import numpy as np
from picamera2 import Picamera2

from lib.cone_diagnostics import CONE_DIAGNOSTIC_SCHEMA_VERSION


class detector:
    def __init__(self):
        # Cone-like shape preference. Keep fairly close to the proven detector so
        # Phase4 can lock onto a distant cone before stricter scoring takes over.
        self.cone_ratio = 33 / 70
        self.ratio_thresh = 0.12
        self.min_component_occupancy = 0.001
        self.proposal_min_component_occupancy = 0.00018
        self.max_candidates_per_mode = 5
        self.backproj_rescue_min_occupancy = 0.006

        # Goal (close contact) judgment: screen should be mostly red and touching edges
        # Goal判定はやや厳しめにする（近距離で画面を大きく占有していること）
        self.reached_occupancy_thresh = 0.26
        self.reached_edge_touch_min = 2

        # Runtime state
        self.input_img = None
        self.projected_img = None
        self.binarized_img = None
        self.detected = None
        self.probability = 0.0
        self.centroids = None
        self.cone_direction = None
        self.occupancy = 0.0
        self.frame_red_occupancy = 0.0
        self.is_detected = False
        self.is_reached = False
        self.debug_method = "init"
        self.debug_scores = {}
        self.capture_reached_path = "./log/capture_reached.png"
        self.evidence_dir = None
        self.evidence_interval_sec = 2.0
        self._last_evidence_save_at = 0.0
        self._evidence_sequence = 0
        self._evidence_frame_sequence = 0
        self._evidence_burst_until = 0.0
        self._evidence_last_trigger_at = 0.0
        self._evidence_saved_sequences = set()
        self._evidence_recent_frames = deque(maxlen=6)
        self.evidence_burst_duration_sec = 1.0
        self.evidence_burst_cooldown_sec = 1.5

        # Camera
        self.picam2 = None
        self.camera_width = 640
        self.camera_height = 480
        self.camera_warmup_sec = 0.8
        self.capture_retry_count = 3
        self.capture_retry_sleep = 0.2
        self.last_capture_error = None
        self.roi_backproj_scale = 0.5
        self.backproj_dilate_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (7, 7),
        )

        # ROI histogram (None = default red mode)
        self.__roi_hist = None
        self.__roi_hist_negative = None
        self.roi_positive_count = 0
        self.roi_negative_count = 0
        self.roi_positive_weight = 0.0
        self.roi_negative_weight = 0.0
        self.negative_backproj_scale = 0.85
        self.roi_hist_min_support_ratio = 0.18
        self.strict_red_min_hue_score = 0.58
        self.strict_red_min_sv_score = 0.28
        self.strict_red_min_shape_score = 0.32
        self.strict_red_min_prob = 0.18
        self.relaxed_red_min_hue_score = 0.45
        self.relaxed_red_min_sv_score = 0.20
        # Keep relaxed candidates below both Phase4 (0.20) and Phase5 (0.18)
        # drive thresholds; temporal confirmation promotes them separately.
        self.relaxed_red_probability_cap = 0.17
        self.small_lower_probability_cap = 0.24

        # Default red HSV ranges (OpenCV H: 0-179)
        self.default_hsv_ranges = [
            (np.array([0, 100, 70], dtype=np.uint8), np.array([16, 255, 255], dtype=np.uint8)),
            (np.array([165, 100, 70], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8)),
        ]
        # Stricter candidate quality floors (user reported cones are visually salient).
        self.min_shape_score_strict = 0.26
        self.min_sv_score_strict = 0.18
        self.min_hue_redness_strict = 0.34
        self.min_detect_probability = 0.16
        self.min_detect_quality_floor = 0.16
        self.temporal_prob_alpha = 0.62
        self.temporal_prob_drop_floor = 0.68
        self.temporal_prob_quality_shape = 0.64
        self.temporal_prob_quality_hue = 0.58
        self.temporal_prob_quality_sv = 0.55
        self.temporal_prob_quality_occ = 0.014
        self._prev_probability = 0.0
        self.variant_selection_margin = 0.20
        self.allow_swap_rb_rescue = False
        self.swap_rb_min_probability = 0.18
        self.swap_rb_min_shape = 0.22
        self.swap_rb_min_hue = 0.26
        self.swap_rb_min_sv = 0.16
        self.swap_rb_force_margin = 0.24
        self.legacy_backproj_min_occupancy = 0.018
        self.legacy_backproj_min_shape = 0.58
        self.legacy_backproj_min_hue = 0.62
        self.legacy_backproj_min_sv = 0.30
        self.legacy_backproj_min_roi_support = 0.28
        self.ground_band_width_frac = 0.62
        self.ground_band_height_frac = 0.42
        self.ground_band_bottom_frac = 0.58
        self.close_region_min_hue = 0.56
        self.close_region_min_sv = 0.26
        self.close_region_min_shape = 0.22
        self.close_region_min_roi_support = 0.12
        self.close_reached_min_prob = 0.28
        self.close_reached_min_shape = 0.34
        self.close_reached_min_sv = 0.34
        self.close_reached_max_aspect = 1.35
        self.close_reached_max_width_frac = 0.86
        self.upper_sky_reject_top_frac = 0.38
        self.upper_sky_reject_width_frac = 0.18
        self.lower_frame_bonus_start = 0.52
        self.clahe_clip_limit = 2.4
        self.clahe_tile_grid = (8, 8)
        self.mask_min_component_area_ratio = 0.00008
        self.mask_open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.mask_close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self._candidate_color_masks = {}
        self._candidate_track_direction = None
        self._candidate_track_height = None
        self._candidate_track_mode = None
        self._candidate_track_at = 0.0

    def __roi_focus_mask(self, bgr_img, label):
        alpha_mask = None
        if bgr_img is not None and getattr(bgr_img, "ndim", 0) == 3 and bgr_img.shape[2] == 4:
            alpha = bgr_img[:, :, 3]
            bgr = cv2.cvtColor(bgr_img, cv2.COLOR_BGRA2BGR)
            alpha_mask = ((alpha.astype(np.uint8) >= 200).astype(np.uint8) * 255)
        else:
            bgr = bgr_img

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if label == "negative":
            return hsv, alpha_mask

        red_mask = None
        for lo, hi in self.default_hsv_ranges:
            part = cv2.inRange(hsv, lo, hi)
            red_mask = part if red_mask is None else cv2.bitwise_or(red_mask, part)

        if red_mask is None:
            return hsv, None

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        coverage = float(np.count_nonzero(red_mask)) / float(red_mask.size) if red_mask.size > 0 else 0.0
        if coverage < 0.005:
            return hsv, alpha_mask

        if alpha_mask is not None:
            red_mask = cv2.bitwise_and(red_mask, alpha_mask)
            if np.count_nonzero(red_mask) == 0:
                return hsv, alpha_mask
        return hsv, red_mask

    def set_roi_img(self, roi):
        if roi is None:
            print("[Detector] Warning: ROI image is None. Using default color range.")
            self.__roi_hist = None
            self.__roi_hist_negative = None
            return
        try:
            roi_list = roi if isinstance(roi, (list, tuple)) else [roi]
            refs = []
            for item in roi_list:
                if item is None:
                    continue
                if isinstance(item, dict):
                    img = item.get("image")
                    if img is None:
                        continue
                    refs.append({
                        "image": img,
                        "label": str(item.get("label", "positive")),
                        "weight": float(item.get("weight", 1.0)),
                    })
                else:
                    refs.append({
                        "image": item,
                        "label": "positive",
                        "weight": 1.0,
                    })
            if not refs:
                print("[Detector] Warning: ROI image list is empty. Using default color range.")
                self.__roi_hist = None
                self.__roi_hist_negative = None
                return
            hist_pos = np.zeros((180, 256), dtype=np.float32)
            hist_neg = np.zeros((180, 256), dtype=np.float32)
            pos_count = 0
            neg_count = 0
            pos_weight = 0.0
            neg_weight = 0.0
            for ref in refs:
                roi_hsv, roi_mask = self.__roi_focus_mask(ref["image"], ref["label"])
                hist = cv2.calcHist([roi_hsv], [0, 1], roi_mask, [180, 256], [0, 180, 0, 256])
                if np.max(hist) <= 0:
                    hist = cv2.calcHist([roi_hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
                hist_sum = float(hist.sum())
                if hist_sum > 0.0:
                    hist = hist / hist_sum
                weight = max(0.05, float(ref["weight"]))
                if ref["label"] == "negative":
                    hist_neg += hist * weight
                    neg_count += 1
                    neg_weight += weight
                else:
                    hist_pos += hist * weight
                    pos_count += 1
                    pos_weight += weight
            if pos_count <= 0:
                print("[Detector] Warning: No positive ROI images. Using default color range.")
                self.__roi_hist = None
                self.__roi_hist_negative = None
                return
            cv2.normalize(hist_pos, hist_pos, 0, 255, cv2.NORM_MINMAX)
            self.__roi_hist = hist_pos
            self.__roi_hist_negative = None
            if neg_count > 0 and np.max(hist_neg) > 0:
                cv2.normalize(hist_neg, hist_neg, 0, 255, cv2.NORM_MINMAX)
                self.__roi_hist_negative = hist_neg
            self.roi_positive_count = pos_count
            self.roi_negative_count = neg_count
            self.roi_positive_weight = pos_weight
            self.roi_negative_weight = neg_weight
            print(
                "[Detector] ROI Histogram set successfully "
                f"(positive={pos_count}, negative={neg_count}, "
                f"pos_w={pos_weight:.2f}, neg_w={neg_weight:.2f})."
            )
        except Exception as exc:
            print(f"[Detector] Error setting ROI: {exc}. Using default color.")
            self.__roi_hist = None
            self.__roi_hist_negative = None

    def __init_camera(self):
        try:
            self.close()

            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                # Picamera2/libcamera names formats by packed word order.  For
                # OpenCV B,G,R array order the documented format is RGB888.
                main={"size": (self.camera_width, self.camera_height), "format": "RGB888"}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(self.camera_warmup_sec)
            print("[Detector] Camera Initialized.")
            return True
        except Exception as exc:
            print(f"[Detector] Camera Init Failed: {exc}")
            self.picam2 = None
            return False

    def close(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
            try:
                self.picam2.close()
            except Exception:
                pass
            self.picam2 = None

    def set_evidence_dir(self, path):
        self.evidence_dir = str(path) if path else None
        if self.evidence_dir:
            os.makedirs(self.evidence_dir, exist_ok=True)

    def __write_evidence_snapshot(self, snapshot, stem):
        try:
            raw = snapshot["input"]
            annotated = raw.copy()
            bbox = snapshot.get("bbox")
            if bbox is not None and len(bbox) >= 4:
                x, y, width, height = (int(value) for value in bbox[:4])
                cv2.rectangle(
                    annotated,
                    (x, y),
                    (x + width, y + height),
                    (0, 255, 255),
                    2,
                )
            cv2.imwrite(
                os.path.join(self.evidence_dir, stem + "_input.jpg"),
                annotated,
                [int(cv2.IMWRITE_JPEG_QUALITY), 88],
            )
            cv2.imwrite(
                os.path.join(self.evidence_dir, stem + "_raw.jpg"),
                raw,
                [int(cv2.IMWRITE_JPEG_QUALITY), 88],
            )
            if snapshot.get("mask") is not None:
                cv2.imwrite(
                    os.path.join(self.evidence_dir, stem + "_mask.png"),
                    snapshot["mask"].astype(np.uint8),
                )
            if snapshot.get("roi") is not None:
                cv2.imwrite(
                    os.path.join(self.evidence_dir, stem + "_roi.jpg"),
                    snapshot["roi"].astype(np.uint8),
                    [int(cv2.IMWRITE_JPEG_QUALITY), 88],
                )
        except Exception as exc:
            print(f"[Detector] Evidence save failed: {exc}")

    def __save_periodic_evidence(self):
        if not self.evidence_dir or self.input_img is None:
            return
        now = time.monotonic()
        self._evidence_frame_sequence += 1
        snapshot = {
            "sequence": self._evidence_frame_sequence,
            "input": self.input_img.copy(),
            "mask": self.binarized_img.copy() if self.binarized_img is not None else None,
            "roi": self.projected_img.copy() if self.projected_img is not None else None,
            "bbox": list(self.detected) if self.detected is not None else None,
            "probability": float(self.probability),
        }
        self._evidence_recent_frames.append(snapshot)

        scores = dict(self.debug_scores or {})
        candidate_event = bool(
            int(scores.get("strict_red_ok", 0))
            or (
                float(scores.get("candidate_probability", 0.0)) >= self.strict_red_min_prob
                and float(scores.get("cone_shape_score", 0.0)) >= self.strict_red_min_shape_score
                and float(scores.get("hue_redness_score", 0.0)) >= self.relaxed_red_min_hue_score
                and float(scores.get("sv_score", 0.0)) >= self.relaxed_red_min_sv_score
            )
        )
        if (
            candidate_event
            and now - self._evidence_last_trigger_at >= self.evidence_burst_cooldown_sec
        ):
            self._evidence_last_trigger_at = now
            self._evidence_burst_until = now + self.evidence_burst_duration_sec
            for buffered in self._evidence_recent_frames:
                sequence = int(buffered["sequence"])
                if sequence in self._evidence_saved_sequences:
                    continue
                stem = (
                    f"event_{sequence:06d}_"
                    f"p{float(buffered['probability']):.3f}"
                )
                self.__write_evidence_snapshot(buffered, stem)
                self._evidence_saved_sequences.add(sequence)

        if now <= self._evidence_burst_until:
            sequence = int(snapshot["sequence"])
            if sequence not in self._evidence_saved_sequences:
                stem = f"event_{sequence:06d}_p{self.probability:.3f}"
                self.__write_evidence_snapshot(snapshot, stem)
                self._evidence_saved_sequences.add(sequence)

        if now - self._last_evidence_save_at >= float(self.evidence_interval_sec):
            self._last_evidence_save_at = now
            self._evidence_sequence += 1
            stem = f"frame_{self._evidence_sequence:04d}_p{self.probability:.3f}"
            self.__write_evidence_snapshot(snapshot, stem)

    def __get_camera_img(self):
        if self.picam2 is None and not self.__init_camera():
            self.last_capture_error = "camera init failed"
            return None

        last_exc = None
        for attempt in range(self.capture_retry_count):
            try:
                raw = self.picam2.capture_array()
                self.last_capture_error = None
                # Standardize detector input orientation for this vehicle build (camera is mounted upside down).
                raw = cv2.rotate(raw, cv2.ROTATE_180)
                # Red-mask preprocessing already applies a 3x3 Gaussian blur.
                # An additional 8x8 box blur erased narrow distant cones.
                return raw
            except Exception as exc:
                last_exc = exc
                print(f"[Detector] Capture Error (attempt {attempt + 1}/{self.capture_retry_count}): {exc}")
                time.sleep(self.capture_retry_sleep)

        self.__init_camera()
        self.last_capture_error = str(last_exc) if last_exc is not None else "camera capture failed"
        return None

    def __red_mask(self, bgr_img):
        pre = self.__preprocess_for_red_mask(bgr_img)
        hsv = cv2.cvtColor(pre, cv2.COLOR_BGR2HSV)
        ycrcb = cv2.cvtColor(pre, cv2.COLOR_BGR2YCrCb)
        lab = cv2.cvtColor(pre, cv2.COLOR_BGR2LAB)

        sat_min, val_min = self.__adaptive_sv_threshold(hsv)
        lo1 = np.array([0, sat_min, val_min], dtype=np.uint8)
        hi1 = np.array([16, 255, 255], dtype=np.uint8)
        lo2 = np.array([165, sat_min, val_min], dtype=np.uint8)
        hi2 = np.array([179, 255, 255], dtype=np.uint8)
        hsv_mask = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1), cv2.inRange(hsv, lo2, hi2))

        cr_min = int(max(138, sat_min + 70))
        ycrcb_mask = cv2.inRange(
            ycrcb,
            np.array([0, cr_min, 60], dtype=np.uint8),
            np.array([255, 255, 175], dtype=np.uint8),
        )

        strict_hsv = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 115, 60], dtype=np.uint8), np.array([12, 255, 255], dtype=np.uint8)),
            cv2.inRange(hsv, np.array([168, 115, 60], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8)),
        )
        lab_a = lab[:, :, 1]
        lab_a_min = int(max(142, np.percentile(lab_a, 82.0)))
        lab_mask = cv2.inRange(lab_a, lab_a_min, 255)
        chromatic_gate = cv2.inRange(
            hsv,
            np.array([0, max(45, sat_min - 30), val_min], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        lab_mask = cv2.bitwise_and(lab_mask, chromatic_gate)
        color_consensus = cv2.bitwise_and(ycrcb_mask, lab_mask)
        fused = cv2.bitwise_or(
            cv2.bitwise_or(cv2.bitwise_and(hsv_mask, ycrcb_mask), strict_hsv),
            color_consensus,
        )
        self._candidate_color_masks = {
            "hsv": hsv_mask,
            "ycrcb": ycrcb_mask,
            "lab": lab_mask,
            "color_fused": fused,
        }
        return hsv, fused

    def __adaptive_sv_threshold(self, hsv_img):
        if hsv_img is None or hsv_img.size == 0:
            return 100, 70
        v_median = float(np.median(hsv_img[:, :, 2]))
        sat_min = int(np.clip(90.0 - 0.20 * (v_median - 120.0), 55.0, 130.0))
        val_min = int(np.clip(65.0 - 0.25 * (v_median - 120.0), 35.0, 95.0))
        return sat_min, val_min

    def __gray_world_balance(self, bgr_img):
        if bgr_img is None:
            return bgr_img
        img = bgr_img.astype(np.float32)
        mean_b, mean_g, mean_r = np.mean(img[:, :, 0]), np.mean(img[:, :, 1]), np.mean(img[:, :, 2])
        mean_gray = max((mean_b + mean_g + mean_r) / 3.0, 1.0)
        scale_b = mean_gray / max(mean_b, 1.0)
        scale_g = mean_gray / max(mean_g, 1.0)
        scale_r = mean_gray / max(mean_r, 1.0)
        img[:, :, 0] *= scale_b
        img[:, :, 1] *= scale_g
        img[:, :, 2] *= scale_r
        return np.clip(img, 0, 255).astype(np.uint8)

    def __preprocess_for_red_mask(self, bgr_img):
        if bgr_img is None:
            return bgr_img
        wb = self.__gray_world_balance(bgr_img)
        lab = cv2.cvtColor(wb, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=self.clahe_clip_limit, tileGridSize=self.clahe_tile_grid)
        l_eq = clahe.apply(l)
        lab_eq = cv2.merge((l_eq, a, b))
        out = cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)
        return cv2.GaussianBlur(out, (3, 3), 0)

    def __remove_small_components(self, mask):
        if mask is None or mask.size == 0:
            return mask
        h, w = mask.shape[:2]
        min_area = max(6, int(float(h * w) * self.mask_min_component_area_ratio))
        nlabels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if nlabels <= 1:
            return mask
        filtered = np.zeros_like(mask)
        for idx in range(1, nlabels):
            area = int(stats[idx, cv2.CC_STAT_AREA])
            if area >= min_area:
                filtered[labels == idx] = 255
        return filtered

    def __back_projection_mask(self, hsv_img):
        if self.__roi_hist is None:
            return None, None
        scale = max(0.25, min(float(self.roi_backproj_scale), 1.0))
        if scale < 1.0:
            work_hsv = cv2.resize(
                hsv_img,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            work_hsv = hsv_img
        proj_pos = cv2.calcBackProject(
            [work_hsv],
            [0, 1],
            self.__roi_hist,
            [0, 180, 0, 256],
            1,
        ).astype(np.float32)
        proj = proj_pos
        if self.__roi_hist_negative is not None:
            proj_neg = cv2.calcBackProject(
                [work_hsv],
                [0, 1],
                self.__roi_hist_negative,
                [0, 180, 0, 256],
                1,
            ).astype(np.float32)
            proj = cv2.max(proj_pos - (self.negative_backproj_scale * proj_neg), 0.0)
        proj = cv2.GaussianBlur(proj, (5, 5), 0)
        proj_u8 = cv2.normalize(proj, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # If ROI backprojection is too weak, inject a conservative hue prior.
        p99 = float(np.percentile(proj_u8, 99.0)) if proj_u8.size > 0 else 0.0
        if p99 < 30.0:
            hue_prior = None
            for lo, hi in self.default_hsv_ranges:
                part = cv2.inRange(work_hsv, lo, hi)
                hue_prior = part if hue_prior is None else cv2.bitwise_or(hue_prior, part)
            if hue_prior is not None:
                hue_prior = cv2.GaussianBlur(hue_prior, (7, 7), 0)
                proj_u8 = cv2.max(proj_u8, (hue_prior.astype(np.float32) * 0.25).astype(np.uint8))

        _, bp_bin = cv2.threshold(proj_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        bp_bin = cv2.morphologyEx(bp_bin, cv2.MORPH_DILATE, self.backproj_dilate_kernel)
        if work_hsv is not hsv_img:
            output_size = (int(hsv_img.shape[1]), int(hsv_img.shape[0]))
            proj_u8 = cv2.resize(
                proj_u8,
                output_size,
                interpolation=cv2.INTER_LINEAR,
            )
            bp_bin = cv2.resize(
                bp_bin,
                output_size,
                interpolation=cv2.INTER_NEAREST,
            )
        return proj_u8, bp_bin

    @staticmethod
    def __bounded_support_ratio(overlap_count, denominator):
        if int(denominator) <= 0:
            return 0.0
        ratio = float(overlap_count) / float(denominator)
        return float(max(0.0, min(ratio, 1.0)))

    @staticmethod
    def __component_roi_support(red_mask, bp_mask, component):
        if red_mask is None or bp_mask is None or component is None:
            return 0.0
        bbox = component.get("bbox") or [0, 0, 0, 0]
        if len(bbox) < 4:
            return 0.0
        x, y, width, height = (int(value) for value in bbox[:4])
        if width <= 0 or height <= 0:
            return 0.0
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(red_mask.shape[1], x + width)
        y1 = min(red_mask.shape[0], y + height)
        if x1 <= x0 or y1 <= y0:
            return 0.0
        labels = component.get("labels")
        label_idx = component.get("label_idx")
        if (
            labels is not None
            and label_idx is not None
            and getattr(labels, "shape", None) == getattr(red_mask, "shape", None)
        ):
            red_crop = (
                (labels[y0:y1, x0:x1] == int(label_idx)).astype(np.uint8)
                * 255
            )
        else:
            red_crop = red_mask[y0:y1, x0:x1]
        bp_crop = bp_mask[y0:y1, x0:x1]
        denominator = int(np.count_nonzero(red_crop))
        if denominator <= 0:
            return 0.0
        overlap = cv2.bitwise_and(red_crop, bp_crop)
        return detector.__bounded_support_ratio(
            np.count_nonzero(overlap),
            denominator,
        )

    @staticmethod
    def __component_projection_support(projected_img, component):
        if projected_img is None or component is None:
            return 0.0
        labels = component.get("labels")
        label_idx = component.get("label_idx")
        if labels is None or label_idx is None:
            return 0.0
        pixels = projected_img[labels == int(label_idx)]
        if pixels.size <= 0:
            return 0.0
        # A high percentile is robust to grass cutting holes through an
        # otherwise reference-like cone, while the mean prevents a single hot
        # pixel from claiming full support.
        mean_score = float(np.mean(pixels)) / 255.0
        high_score = float(np.percentile(pixels, 75.0)) / 255.0
        return float(max(0.0, min(0.55 * mean_score + 0.45 * high_score, 1.0)))

    def __postprocess_mask(self, mask):
        if mask is None:
            return None
        out = cv2.medianBlur(mask, 3)
        out = cv2.morphologyEx(out, cv2.MORPH_OPEN, self.mask_open_kernel, iterations=1)
        out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, self.mask_close_kernel, iterations=2)
        out = self.__remove_small_components(out)
        out = cv2.morphologyEx(out, cv2.MORPH_DILATE, self.mask_open_kernel, iterations=1)
        return out.astype(np.uint8)

    def __edge_touch_count(self, mask):
        if mask is None or mask.size == 0:
            return 0
        m = mask > 0
        count = 0
        if np.any(m[0, :]):
            count += 1
        if np.any(m[-1, :]):
            count += 1
        if np.any(m[:, 0]):
            count += 1
        if np.any(m[:, -1]):
            count += 1
        return count

    def __largest_component_bbox(self, mask):
        if mask is None or mask.size == 0:
            return None
        mask_u8 = mask.astype(np.uint8)
        nlabels, _, stats, centroids = cv2.connectedComponentsWithStats(mask_u8)
        if nlabels <= 1:
            return None
        best_idx = None
        best_area = 0
        for idx in range(1, nlabels):
            area = int(stats[idx, cv2.CC_STAT_AREA])
            if area > best_area:
                best_area = area
                best_idx = idx
        if best_idx is None:
            return None
        s = stats[best_idx]
        c = centroids[best_idx]
        return {
            "bbox": [
                int(s[cv2.CC_STAT_LEFT]),
                int(s[cv2.CC_STAT_TOP]),
                int(s[cv2.CC_STAT_WIDTH]),
                int(s[cv2.CC_STAT_HEIGHT]),
            ],
            "centroid": [int(c[0]), int(c[1])],
            "area": float(s[cv2.CC_STAT_AREA]),
        }

    def __row_span_width(self, mask_crop, row_start, row_end):
        if mask_crop is None or mask_crop.size == 0:
            return 0.0
        h, _ = mask_crop.shape[:2]
        row_start = max(0, min(h, int(row_start)))
        row_end = max(row_start + 1, min(h, int(row_end)))
        band = mask_crop[row_start:row_end, :] > 0
        if not np.any(band):
            return 0.0
        cols = np.where(np.any(band, axis=0))[0]
        if cols.size == 0:
            return 0.0
        return float(cols[-1] - cols[0] + 1)

    def __contour_shape_metrics(self, labels, label_idx, bbox, area):
        if labels is None or bbox is None:
            return {
                "cone_shape_score": 0.0,
                "taper_score": 0.0,
                "solidity": 0.0,
                "extent": 0.0,
                "bottom_heavy_score": 0.0,
                "vertex_score": 0.0,
            }

        x, y, w, h = bbox
        comp_crop = (labels[y : y + h, x : x + w] == label_idx).astype(np.uint8) * 255
        contours, _ = cv2.findContours(comp_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {
                "cone_shape_score": 0.0,
                "taper_score": 0.0,
                "solidity": 0.0,
                "extent": 0.0,
                "bottom_heavy_score": 0.0,
                "vertex_score": 0.0,
            }

        cnt = max(contours, key=cv2.contourArea)
        cnt_area = max(float(cv2.contourArea(cnt)), 1.0)
        hull = cv2.convexHull(cnt)
        hull_area = max(float(cv2.contourArea(hull)), 1.0)
        solidity = min(cnt_area / hull_area, 1.0)
        extent = min(float(area) / max(float(w * h), 1.0), 1.0)

        top_width = self.__row_span_width(comp_crop, h * 0.05, h * 0.25)
        bottom_width = self.__row_span_width(comp_crop, h * 0.70, h * 0.95)
        taper_raw = (bottom_width - top_width) / max(float(w), 1.0)
        taper_score_down = max(0.0, min((taper_raw + 0.05) / 0.45, 1.0))
        # Grass can hide the base and close cones can be clipped by the top
        # edge, so keep both taper directions as soft evidence rather than a
        # hard orientation gate.
        taper_score_up = max(0.0, min(((-taper_raw) + 0.05) / 0.45, 1.0))
        taper_score = max(taper_score_down, taper_score_up)

        half = max(1, h // 2)
        upper_pixels = float(np.count_nonzero(comp_crop[:half, :]))
        lower_pixels = float(np.count_nonzero(comp_crop[half:, :]))
        total_pixels = upper_pixels + lower_pixels
        lower_ratio = (lower_pixels / total_pixels) if total_pixels > 0 else 0.0
        bottom_heavy_score_down = max(0.0, min((lower_ratio - 0.48) / 0.22, 1.0))
        bottom_heavy_score_up = max(0.0, min(((1.0 - lower_ratio) - 0.48) / 0.22, 1.0))
        bottom_heavy_score = max(bottom_heavy_score_down, bottom_heavy_score_up)

        peri = float(cv2.arcLength(cnt, True))
        vertex_score = 0.0
        if peri > 0.0:
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            # Cone silhouettes often appear as 3-6 vertices depending on blur and truncation.
            v = len(approx)
            if 3 <= v <= 6:
                vertex_score = 1.0
            elif v in (2, 7):
                vertex_score = 0.5

        # Penalize thin/irregular blobs (grass) while keeping tolerance for far blurry cones.
        slenderness = float(h) / max(float(w), 1.0)
        slender_tall_score = max(0.0, min((slenderness - 1.4) / 2.8, 1.0))
        taper_contrast_score = max(0.0, min(abs(taper_raw) / 0.45, 1.0))
        partial_edge_score = 0.0
        if (
            y <= 1
            and float(h) / max(float(self.camera_height), 1.0) >= 0.16
            and float(w) / max(float(h), 1.0) <= 1.15
            and extent >= 0.30
        ):
            partial_edge_score = 1.0
        cone_shape_score = (
            0.22 * taper_score
            + 0.20 * solidity
            + 0.15 * max(0.0, min((extent - 0.16) / 0.46, 1.0))
            + 0.10 * bottom_heavy_score
            + 0.10 * vertex_score
            + 0.08 * slender_tall_score
            + 0.05 * taper_contrast_score
            + 0.10 * partial_edge_score
        )
        return {
            "cone_shape_score": float(max(0.0, min(cone_shape_score, 1.0))),
            "taper_score": float(taper_score),
            "taper_score_down": float(taper_score_down),
            "taper_score_up": float(taper_score_up),
            "solidity": float(solidity),
            "extent": float(extent),
            "bottom_heavy_score": float(bottom_heavy_score),
            "bottom_heavy_score_down": float(bottom_heavy_score_down),
            "bottom_heavy_score_up": float(bottom_heavy_score_up),
            "vertex_score": float(vertex_score),
            "slender_tall_score": float(slender_tall_score),
            "taper_contrast_score": float(taper_contrast_score),
            "partial_edge_score": float(partial_edge_score),
        }

    def __component_color_metrics(self, hsv_img, labels, label_idx):
        if hsv_img is None or labels is None:
            return {"sv_score": 0.0, "sat_mean": 0.0, "val_mean": 0.0, "hue_redness_score": 0.0}
        pix = hsv_img[labels == label_idx]
        if pix.size == 0:
            return {"sv_score": 0.0, "sat_mean": 0.0, "val_mean": 0.0, "hue_redness_score": 0.0}
        hue = pix[:, 0].astype(np.float32)
        sat = pix[:, 1].astype(np.float32)
        val = pix[:, 2].astype(np.float32)
        sat_mean = float(np.mean(sat))
        val_mean = float(np.mean(val))
        # Circular distance on OpenCV hue scale [0,179]; red is near 0/179.
        hue_dist = np.minimum(hue, 180.0 - hue)
        hue_redness = 1.0 - np.clip(hue_dist / 18.0, 0.0, 1.0)

        # Score hue mainly on strongly chromatic pixels to suppress grass/ground false positives.
        color_valid = np.logical_and(sat >= 90.0, val >= 50.0)
        if np.count_nonzero(color_valid) >= max(12, int(0.05 * hue_redness.size)):
            hue_redness_valid = hue_redness[color_valid]
            sat_valid = sat[color_valid]
            sat_weight = np.clip((sat_valid - 90.0) / 100.0, 0.20, 1.0)
            hue_redness_score = float(np.sum(hue_redness_valid * sat_weight) / max(np.sum(sat_weight), 1e-6))
        else:
            sat_weight = np.clip((sat - 70.0) / 120.0, 0.05, 1.0)
            hue_redness_score = float(np.sum(hue_redness * sat_weight) / max(np.sum(sat_weight), 1e-6))

        sat_score = max(0.0, min((sat_mean - 110.0) / 90.0, 1.0))
        val_score = max(0.0, min((val_mean - 70.0) / 90.0, 1.0))
        sv_score = 0.6 * sat_score + 0.4 * val_score
        return {
            "sv_score": float(max(0.0, min(sv_score, 1.0))),
            "sat_mean": sat_mean,
            "val_mean": val_mean,
            "hue_redness_score": hue_redness_score,
        }

    def __component_metrics(
        self,
        mask,
        hsv_img=None,
        include_largest=False,
        return_candidates=False,
        min_occupancy=None,
    ):
        if mask is None:
            if return_candidates:
                return ([], None) if include_largest else []
            return (None, None) if include_largest else None
        mask_u8 = mask.astype(np.uint8)
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_u8)
        if nlabels <= 1:
            if return_candidates:
                return ([], None) if include_largest else []
            return (None, None) if include_largest else None

        img_size = float(self.camera_width * self.camera_height)
        occupancy_floor = float(
            self.min_component_occupancy
            if min_occupancy is None
            else min_occupancy
        )
        best = None
        largest = None
        ranked = []
        for idx in range(1, nlabels):
            s = stats[idx]
            area = float(s[cv2.CC_STAT_AREA])
            if area / img_size < occupancy_floor:
                continue
            w = max(1, int(s[cv2.CC_STAT_WIDTH]))
            h = max(1, int(s[cv2.CC_STAT_HEIGHT]))
            cx, cy = centroids[idx]
            left = int(s[cv2.CC_STAT_LEFT])
            top = int(s[cv2.CC_STAT_TOP])
            right = left + w
            bottom = top + h
            aspect = float(w) / float(h)
            aspect_diff = abs(aspect - self.cone_ratio)
            aspect_score_std = 1.0 - min(aspect_diff / max(self.ratio_thresh, 1e-6), 1.0)
            # Distant cone often looks more slender in this camera geometry.
            aspect_score_far = 1.0 - min(abs(aspect - 0.24) / 0.18, 1.0)
            aspect_score = max(aspect_score_std, 0.95 * aspect_score_far)
            bbox = [
                left,
                top,
                w,
                h,
            ]
            contour_metrics = self.__contour_shape_metrics(labels, idx, bbox, area)
            cone_shape_score = contour_metrics["cone_shape_score"]
            color_metrics = self.__component_color_metrics(hsv_img, labels, idx)
            sv_score = color_metrics["sv_score"]
            hue_redness_score = color_metrics["hue_redness_score"]
            shape_score = 0.42 * aspect_score + 0.58 * cone_shape_score
            area_score = min((area / img_size) / 0.10, 1.0)
            center_score = 1.0 - min(abs((cx / self.camera_width) - 0.5) / 0.5, 1.0)
            cy_norm = float(cy) / float(self.camera_height)
            vertical_center_score = 1.0 - min(abs(cy_norm - 0.5) / 0.5, 1.0)
            lower_frame_bonus = max(0.0, min((cy_norm - self.lower_frame_bonus_start) / 0.30, 1.0))
            score = (
                0.24 * area_score
                + 0.41 * shape_score
                + 0.08 * center_score
                + 0.12 * sv_score
                + 0.08 * hue_redness_score
                + 0.02 * vertical_center_score
                + 0.10 * lower_frame_bonus
            )
            occ = area / img_size
            edge_touch_count = 0
            if left <= 1:
                edge_touch_count += 1
            if top <= 1:
                edge_touch_count += 1
            if right >= (self.camera_width - 1):
                edge_touch_count += 1
            if bottom >= (self.camera_height - 1):
                edge_touch_count += 1
            if occ < 0.08:
                score *= 0.60 + 0.40 * cone_shape_score
            if hue_redness_score < 0.45:
                score *= 0.70 + 0.30 * hue_redness_score
            quality_floor = min(cone_shape_score, sv_score, hue_redness_score)
            score *= 0.52 + 0.48 * quality_floor
            width_frac = float(w) / float(self.camera_width)
            height_frac = float(h) / float(self.camera_height)
            if occ < self.reached_occupancy_thresh:
                if edge_touch_count >= 3:
                    score *= 0.25
                elif edge_touch_count == 2:
                    score *= 0.45 if aspect > 1.20 else 0.62
                elif edge_touch_count == 1 and aspect > 1.80 and cone_shape_score < 0.55:
                    score *= 0.78
            if aspect > 1.55 and cone_shape_score < 0.55:
                # Wide edge-connected blobs tend to be sky/horizon after channel swap.
                score *= 0.72
            if edge_touch_count >= 2 and width_frac > 0.45 and cone_shape_score < 0.60:
                score *= 0.45 if width_frac > 0.65 else 0.62
            if width_frac > 0.55 and aspect > 1.25 and contour_metrics["taper_contrast_score"] < 0.25:
                score *= 0.58
            if (
                cone_shape_score < self.min_shape_score_strict
                or sv_score < self.min_sv_score_strict
                or hue_redness_score < self.min_hue_redness_strict
            ):
                # Strongly suppress grass/branches and brownish blobs that pass hue threshold loosely.
                score *= 0.72
            # Far cone rescue: allow small but very cone-like red silhouettes to survive strict penalties.
            if (
                occ < 0.035
                and cone_shape_score >= 0.55
                and hue_redness_score >= 0.68
                and sv_score >= 0.24
                and cy_norm >= 0.48
            ):
                score = max(score, 0.34 + 0.36 * cone_shape_score + 0.18 * hue_redness_score)
            item = {
                "label_idx": idx,
                "score": score,
                "area": area,
                "occupancy": area / img_size,
                "bbox": bbox,
                "centroid": [int(cx), int(cy)],
                "shape_score": shape_score,
                "aspect_score": aspect_score,
                "aspect_score_std": aspect_score_std,
                "aspect_score_far": aspect_score_far,
                "cone_shape_score": cone_shape_score,
                "sv_score": sv_score,
                "hue_redness_score": hue_redness_score,
                "vertical_center_score": vertical_center_score,
                "lower_frame_bonus": lower_frame_bonus,
                "edge_touch_count": edge_touch_count,
                "width_frac": width_frac,
                "height_frac": height_frac,
                "aspect": aspect,
                "labels": labels,
            }
            item.update(contour_metrics)
            item.update(color_metrics)
            ranked.append(item)
            if best is None or item["score"] > best["score"]:
                best = item
            if largest is None or item["area"] > largest["area"]:
                largest = item
        ranked.sort(key=lambda item: float(item.get("score", 0.0)), reverse=True)
        if return_candidates:
            limited = ranked[: max(1, int(self.max_candidates_per_mode))]
            return (limited, largest) if include_largest else limited
        if include_largest:
            return best, largest
        return best

    def __proposal_is_eligible(self, component, support_ratio, mode_name):
        if component is None:
            return False
        occupancy = float(component.get("occupancy", 0.0))
        shape = float(component.get("cone_shape_score", 0.0))
        hue = float(component.get("hue_redness_score", 0.0))
        sv = float(component.get("sv_score", 0.0))
        if mode_name in ("lab", "ycrcb"):
            return bool(
                float(support_ratio) >= 0.06
                or (shape >= 0.32 and hue >= 0.45 and sv >= 0.20)
            )
        if mode_name == "backproj":
            return bool(
                float(support_ratio) >= 0.16
                and shape >= 0.26
                and hue >= 0.34
                and sv >= 0.18
            )
        if occupancy >= self.min_component_occupancy:
            return True
        return bool(
            mode_name == "hybrid_overlap"
            or float(support_ratio) >= 0.10
            or (shape >= 0.45 and hue >= 0.62 and sv >= 0.20)
        )

    @staticmethod
    def __candidate_mode_score(component, support_ratio, mode_name):
        if component is None:
            return -1.0
        score = float(component.get("score", 0.0))
        score += 0.20 * max(0.0, min(float(support_ratio), 1.0))
        if mode_name == "hybrid_overlap":
            score += 0.12
        elif mode_name == "hue":
            score += 0.06
        elif mode_name in ("lab", "ycrcb"):
            score -= 0.03
        elif mode_name == "backproj":
            score -= 0.08
        if float(component.get("occupancy", 0.0)) < 0.001:
            score -= 0.04
        return float(score)

    def __candidate_temporal_bonus(self, component, mode_name, now):
        last_direction = self._candidate_track_direction
        if last_direction is None or now - float(self._candidate_track_at) > 1.0:
            return 0.0
        centroid = component.get("centroid") or [self.camera_width * 0.5, 0]
        direction = float(centroid[0]) / max(float(self.camera_width), 1.0)
        direction_error = abs(direction - float(last_direction))
        bonus = 0.12 * max(0.0, 1.0 - direction_error / 0.48)
        last_height = self._candidate_track_height
        current_height = float((component.get("bbox") or [0, 0, 0, 0])[3])
        if last_height and current_height > 0.0:
            ratio = max(
                float(last_height) / current_height,
                current_height / float(last_height),
            )
            if ratio <= 2.5:
                bonus += 0.04 * max(0.0, 1.0 - (ratio - 1.0) / 1.5)
        if mode_name == self._candidate_track_mode:
            bonus += 0.02
        return float(bonus)

    def __largest_component_metrics(self, mask, hsv_img=None):
        if mask is None:
            return None
        mask_u8 = mask.astype(np.uint8)
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_u8)
        if nlabels <= 1:
            return None

        img_size = float(self.camera_width * self.camera_height)
        best_idx = None
        best_occ = 0.0
        for idx in range(1, nlabels):
            area = float(stats[idx, cv2.CC_STAT_AREA])
            occ = area / img_size
            if occ < self.min_component_occupancy:
                continue
            if best_idx is None or occ > best_occ:
                best_idx = idx
                best_occ = occ
        if best_idx is None:
            return None

        s = stats[best_idx]
        cx, cy = centroids[best_idx]
        bbox = [
            int(s[cv2.CC_STAT_LEFT]),
            int(s[cv2.CC_STAT_TOP]),
            int(s[cv2.CC_STAT_WIDTH]),
            int(s[cv2.CC_STAT_HEIGHT]),
        ]
        contour_metrics = self.__contour_shape_metrics(labels, best_idx, bbox, float(s[cv2.CC_STAT_AREA]))
        color_metrics = self.__component_color_metrics(hsv_img, labels, best_idx)
        w = max(1, int(s[cv2.CC_STAT_WIDTH]))
        h = max(1, int(s[cv2.CC_STAT_HEIGHT]))
        aspect = float(w) / float(h)
        aspect_diff = abs(aspect - self.cone_ratio)
        aspect_score_std = 1.0 - min(aspect_diff / max(self.ratio_thresh, 1e-6), 1.0)
        aspect_score_far = 1.0 - min(abs(aspect - 0.24) / 0.18, 1.0)
        aspect_score = max(aspect_score_std, 0.95 * aspect_score_far)
        occ = float(s[cv2.CC_STAT_AREA]) / img_size
        legacy_score = min(0.95, 0.18 + 8.0 * occ)
        item = {
            "label_idx": best_idx,
            "score": legacy_score,
            "legacy_score": legacy_score,
            "area": float(s[cv2.CC_STAT_AREA]),
            "occupancy": occ,
            "bbox": bbox,
            "centroid": [int(cx), int(cy)],
            "aspect": aspect,
            "aspect_score": aspect_score,
            "aspect_score_std": aspect_score_std,
            "aspect_score_far": aspect_score_far,
            "shape_score": 0.42 * aspect_score + 0.58 * contour_metrics["cone_shape_score"],
            "labels": labels,
        }
        item.update(contour_metrics)
        item.update(color_metrics)
        return item

    def __variant_selection_score(self, candidate):
        if candidate is None:
            return -1.0
        prob = float(candidate.get("probability", 0.0))
        shape = float(candidate.get("cone_shape_score", 0.0))
        hue = float(candidate.get("hue_redness_score", 0.0))
        sv = float(candidate.get("sv_score", 0.0))
        edge_touch = float(candidate.get("edge_touch_count", 0.0))
        width_frac = float(candidate.get("width_frac", 0.0))
        aspect = float(candidate.get("aspect", 0.0))
        reached = bool(candidate.get("is_reached", False))

        score = prob + 0.34 * shape + 0.18 * hue + 0.12 * sv
        if reached:
            score += 1.5
        if edge_touch >= 2.0 and width_frac > 0.45 and not reached:
            score -= 0.38 + 0.35 * max(0.0, width_frac - 0.45)
        if aspect > 1.25 and shape < 0.60 and not reached:
            score -= 0.14
        if candidate.get("debug_method", "").endswith(":backproj"):
            score += 0.10 * min(float(candidate.get("occupancy", 0.0)) / 0.05, 1.0)
        return score

    def __ground_band_penalty(self, width_frac, height_frac, bbox_bottom_frac, aspect, cone_shape_score):
        penalty = 1.0
        if (
            width_frac >= self.ground_band_width_frac
            and height_frac <= self.ground_band_height_frac
            and bbox_bottom_frac >= self.ground_band_bottom_frac
        ):
            penalty *= 0.04
        elif width_frac >= 0.52 and height_frac <= 0.36 and bbox_bottom_frac >= 0.52:
            penalty *= 0.16
        if aspect >= 2.2 and cone_shape_score < 0.58:
            penalty *= 0.22
        if width_frac >= 0.40 and aspect >= 1.60 and cone_shape_score < 0.62:
            penalty *= 0.30
        return penalty

    def __strict_red_candidate_reject_reason(self, component, roi_support_ratio):
        if component is None:
            return "no_component"
        hue = float(component.get("hue_redness_score", 0.0))
        sv = float(component.get("sv_score", 0.0))
        shape = float(component.get("cone_shape_score", 0.0))
        prob = float(component.get("score", 0.0))
        width_frac = float(component.get("width_frac", 0.0))
        height_frac = float(component.get("height_frac", 0.0))
        bbox = component.get("bbox") or [0, 0, 0, 0]
        bbox_bottom_frac = float((bbox[1] + bbox[3]) / float(self.camera_height)) if self.camera_height > 0 else 0.0
        aspect = float(component.get("aspect", 0.0))
        if hue < self.strict_red_min_hue_score:
            return "hue_below_min"
        if sv < self.strict_red_min_sv_score:
            return "sv_below_min"
        if shape < self.strict_red_min_shape_score:
            return "shape_below_min"
        if prob < self.strict_red_min_prob:
            return "candidate_prob_below_min"
        if self.__ground_band_penalty(width_frac, height_frac, bbox_bottom_frac, aspect, shape) < 0.5:
            return "ground_band_rejected"
        if roi_support_ratio < 0.06 and width_frac > 0.26:
            return "wide_without_roi_support"
        return ""

    def __relaxed_red_candidate_ok(self, component, reject_reason):
        """Keep plausible outdoor red cones as weak temporal candidates."""
        if component is None:
            return False
        hue = float(component.get("hue_redness_score", 0.0))
        sv = float(component.get("sv_score", 0.0))
        shape = float(component.get("cone_shape_score", 0.0))
        prob = float(component.get("score", 0.0))
        if prob < self.strict_red_min_prob or shape < self.strict_red_min_shape_score:
            return False
        if reject_reason == "hue_below_min":
            return hue >= self.relaxed_red_min_hue_score and sv >= self.strict_red_min_sv_score
        if reject_reason == "sv_below_min":
            return sv >= self.relaxed_red_min_sv_score and hue >= self.strict_red_min_hue_score
        return False

    def __relaxed_red_penalty(self, component, reject_reason):
        hue = float(component.get("hue_redness_score", 0.0))
        sv = float(component.get("sv_score", 0.0))
        if reject_reason == "hue_below_min":
            span = max(self.strict_red_min_hue_score - self.relaxed_red_min_hue_score, 1e-6)
            ratio = np.clip((hue - self.relaxed_red_min_hue_score) / span, 0.0, 1.0)
        else:
            span = max(self.strict_red_min_sv_score - self.relaxed_red_min_sv_score, 1e-6)
            ratio = np.clip((sv - self.relaxed_red_min_sv_score) / span, 0.0, 1.0)
        return float(0.52 + 0.36 * ratio)

    def __small_lower_rescue(
        self,
        prob,
        *,
        strict_red_ok,
        bbox_bottom_frac,
        occupancy,
        cone_shape_score,
        hue_redness_score,
        sv_score,
    ):
        eligible = bool(
            strict_red_ok
            and bbox_bottom_frac >= 0.55
            and occupancy < 0.045
            and cone_shape_score >= 0.55
            and hue_redness_score >= 0.62
            and sv_score >= 0.24
        )
        if not eligible:
            return float(prob), False
        lower_score = (
            0.26
            + 0.40 * cone_shape_score
            + 0.18 * hue_redness_score
            + 1.8 * occupancy
        )
        return (
            max(float(prob), min(self.small_lower_probability_cap, lower_score)),
            True,
        )

    def __swap_rb_candidate_is_trustworthy(self, candidate):
        if candidate is None:
            return False
        if bool(candidate.get("is_reached", False)):
            return True
        prob = float(candidate.get("probability", 0.0))
        shape = float(candidate.get("cone_shape_score", 0.0))
        hue = float(candidate.get("hue_redness_score", 0.0))
        sv = float(candidate.get("sv_score", 0.0))
        roi_support = float(candidate.get("roi_support_ratio", 0.0))
        occupancy = float(candidate.get("occupancy", 0.0))
        bbox = candidate.get("bbox") or [0, 0, 0, 0]
        top = int(bbox[1]) if len(bbox) >= 2 else 0
        height = int(bbox[3]) if len(bbox) >= 4 else 0
        bbox_bottom_frac = float(top + height) / float(self.camera_height) if self.camera_height > 0 else 0.0
        quality_floor = min(shape, hue, sv)
        if prob < self.swap_rb_min_probability:
            return False
        if shape < self.swap_rb_min_shape or hue < self.swap_rb_min_hue or sv < self.swap_rb_min_sv:
            return False
        if quality_floor < 0.14:
            return False
        if occupancy < 0.006 and roi_support < 0.06:
            return False
        if bbox_bottom_frac < 0.34 and prob < 0.26:
            return False
        return True

    def __temporal_stabilize_probability(self, prob, candidate):
        if candidate is None:
            self._prev_probability = float(prob)
            return float(prob)

        shape = float(candidate.get("cone_shape_score", 0.0))
        hue = float(candidate.get("hue_redness_score", 0.0))
        sv = float(candidate.get("sv_score", 0.0))
        occ = float(candidate.get("occupancy", 0.0))
        quality_ok = (
            shape >= self.temporal_prob_quality_shape
            and hue >= self.temporal_prob_quality_hue
            and sv >= self.temporal_prob_quality_sv
            and occ >= self.temporal_prob_quality_occ
        )
        if quality_ok and self._prev_probability > 0.0:
            ema = (self.temporal_prob_alpha * float(prob)) + ((1.0 - self.temporal_prob_alpha) * self._prev_probability)
            if prob < self._prev_probability:
                ema = max(ema, self._prev_probability * self.temporal_prob_drop_floor)
            prob = float(min(1.0, max(0.0, ema)))

        self._prev_probability = float(prob)
        return float(prob)

    def __close_range_reached(self, red_mask):
        img_size = float(self.camera_width * self.camera_height)
        red_occ = float(np.count_nonzero(red_mask)) / img_size if red_mask is not None else 0.0
        edge_touch = self.__edge_touch_count(red_mask)
        reached = (red_occ >= self.reached_occupancy_thresh) and (edge_touch >= self.reached_edge_touch_min)
        return reached, red_occ, edge_touch

    def __build_candidate(self, bgr_img, variant_name):
        hsv, red_mask_raw = self.__red_mask(bgr_img)
        proj, bp_mask_raw = self.__back_projection_mask(hsv)

        red_mask = self.__postprocess_mask(red_mask_raw)
        bp_mask = self.__postprocess_mask(bp_mask_raw) if bp_mask_raw is not None else None
        processed_color_masks = {
            name: self.__postprocess_mask(mask)
            for name, mask in dict(self._candidate_color_masks or {}).items()
            if mask is not None
        }
        processed_color_masks["hue"] = red_mask
        mode_masks = [("hue", red_mask)]
        for mode_name in ("ycrcb", "lab"):
            mode_mask = processed_color_masks.get(mode_name)
            if mode_mask is not None:
                mode_masks.append((mode_name, mode_mask))
        if bp_mask is not None:
            overlap = self.__postprocess_mask(cv2.bitwise_and(red_mask, bp_mask))
            mode_masks.extend(
                [
                    ("hybrid_overlap", overlap),
                    ("backproj", bp_mask),
                ]
            )

        red_candidates, dominant_red_component = self.__component_metrics(
            red_mask,
            hsv_img=hsv,
            include_largest=True,
            return_candidates=True,
            min_occupancy=self.proposal_min_component_occupancy,
        )
        proposed = []
        candidate_now = time.monotonic()
        for mode_name, mode_mask in mode_masks:
            components = red_candidates if mode_name == "hue" else (
                self.__component_metrics(
                    mode_mask,
                    hsv_img=hsv,
                    return_candidates=True,
                    min_occupancy=self.proposal_min_component_occupancy,
                )
            )
            for component in components:
                if mode_name == "backproj":
                    support_ratio = self.__component_projection_support(
                        proj,
                        component,
                    )
                else:
                    support_ratio = self.__component_roi_support(
                        mode_mask,
                        bp_mask,
                        component,
                    )
                if not self.__proposal_is_eligible(
                    component,
                    support_ratio,
                    mode_name,
                ):
                    continue
                temporal_bonus = self.__candidate_temporal_bonus(
                    component,
                    mode_name,
                    candidate_now,
                )
                proposed.append(
                    {
                        "mode": mode_name,
                        "mask": mode_mask,
                        "component": component,
                        "support": support_ratio,
                        "temporal_bonus": temporal_bonus,
                        "selection_score": (
                            self.__candidate_mode_score(
                                component,
                                support_ratio,
                                mode_name,
                            )
                            + temporal_bonus
                        ),
                    }
                )
        proposed.sort(
            key=lambda item: float(item["selection_score"]),
            reverse=True,
        )
        selected = proposed[0] if proposed else None
        best_mode = selected["mode"] if selected is not None else "hue"
        best_mask = selected["mask"] if selected is not None else red_mask
        best_component = selected["component"] if selected is not None else None
        roi_support_ratio = float(selected["support"]) if selected is not None else 0.0
        candidate_count = len(proposed)
        candidate_rank = 1 if selected is not None else 0
        candidate_temporal_bonus = (
            float(selected["temporal_bonus"]) if selected is not None else 0.0
        )
        if selected is not None:
            selected_centroid = best_component.get("centroid") or [self.camera_width * 0.5, 0]
            selected_bbox = best_component.get("bbox") or [0, 0, 0, 0]
            self._candidate_track_direction = float(selected_centroid[0]) / max(
                float(self.camera_width),
                1.0,
            )
            self._candidate_track_height = float(selected_bbox[3])
            self._candidate_track_mode = best_mode
            self._candidate_track_at = candidate_now

        reached, frame_red_occupancy, edge_touch = self.__close_range_reached(red_mask)
        dominant_red = None
        if dominant_red_component is not None:
            dominant_red = {
                "bbox": dominant_red_component["bbox"],
                "centroid": dominant_red_component["centroid"],
                "area": dominant_red_component["area"],
            }

        prob = 0.0
        centroid = None
        cone_dir = None
        occupancy = 0.0
        bbox = None
        is_detected = False
        candidate_probability = 0.0
        pre_filter_probability = 0.0
        strict_red_ok = False
        strict_red_reject_reason = "no_component"
        close_region_ok = False
        dominant_close = False
        close_reached_reject_reason = "no_dominant_red_component"
        ground_penalty = 1.0
        penalty_flags = []

        if best_component is not None:
            bbox = best_component["bbox"]
            centroid = best_component["centroid"]
            occupancy = best_component["occupancy"]
            prob = float(max(0.0, min(1.0, best_component["score"])))
            candidate_probability = prob
            cone_dir = float(best_component["centroid"][0]) / float(self.camera_width)
            is_detected = prob >= self.min_detect_probability

        # If the cone is too close, component scoring tends to latch onto a small side blob.
        # In that case, promote the dominant red region itself.
        if dominant_red is not None:
            dom_bbox = dominant_red["bbox"]
            dom_centroid = dominant_red["centroid"]
            dom_area = float(dominant_red["area"])
            dom_occ = dom_area / float(self.camera_width * self.camera_height)
            dom_width_frac = float(dom_bbox[2]) / float(self.camera_width)
            dom_height_frac = float(dom_bbox[3]) / float(self.camera_height)
            dom_bottom_frac = float(dom_bbox[1] + dom_bbox[3]) / float(self.camera_height)
            dom_shape = float(dominant_red_component.get("cone_shape_score", 0.0)) if dominant_red_component else 0.0
            dom_hue = float(dominant_red_component.get("hue_redness_score", 0.0)) if dominant_red_component else 0.0
            dom_sv = float(dominant_red_component.get("sv_score", 0.0)) if dominant_red_component else 0.0
            dominant_roi_support = self.__component_roi_support(
                red_mask,
                bp_mask,
                dominant_red_component,
            )
            dom_ground_penalty = self.__ground_band_penalty(
                dom_width_frac,
                dom_height_frac,
                dom_bottom_frac,
                float(dom_bbox[2]) / max(float(dom_bbox[3]), 1.0),
                dom_shape,
            )
            close_region_ok = (
                dom_hue >= self.close_region_min_hue
                and dom_sv >= self.close_region_min_sv
                and dom_shape >= self.close_region_min_shape
                and (
                    dominant_roi_support >= self.close_region_min_roi_support
                    or dom_occ >= 0.20
                )
                and dom_ground_penalty >= 0.5
            )
            dominant_close = (
                dom_occ >= 0.14
                or (frame_red_occupancy >= 0.22 and dom_width_frac >= 0.30 and dom_height_frac >= 0.35)
            )
            if dominant_close and close_region_ok:
                best_component = dominant_red_component
                bbox = dom_bbox
                centroid = dom_centroid
                occupancy = max(occupancy, dom_occ)
                cone_dir = float(dom_centroid[0]) / float(self.camera_width)
                prob = max(prob, min(0.98, 0.30 + 2.6 * frame_red_occupancy))
                is_detected = True
                best_mask = red_mask
                best_mode = "close_red_region"
                roi_support_ratio = dominant_roi_support

        close_reached_ok = False
        if dominant_red is not None and dominant_red_component is not None:
            dom_bbox = dominant_red["bbox"]
            dom_occ = float(dominant_red["area"]) / float(self.camera_width * self.camera_height)
            dom_width_frac = float(dom_bbox[2]) / float(self.camera_width)
            dom_height_frac = float(dom_bbox[3]) / float(self.camera_height)
            dom_bottom_frac = float(dom_bbox[1] + dom_bbox[3]) / float(self.camera_height)
            dom_aspect = float(dom_bbox[2]) / max(float(dom_bbox[3]), 1.0)
            dom_shape = float(dominant_red_component.get("cone_shape_score", 0.0))
            dom_hue = float(dominant_red_component.get("hue_redness_score", 0.0))
            dom_sv = float(dominant_red_component.get("sv_score", 0.0))
            dom_ground_penalty = self.__ground_band_penalty(
                dom_width_frac,
                dom_height_frac,
                dom_bottom_frac,
                dom_aspect,
                dom_shape,
            )
            reached_shape_ok = (
                dom_shape >= self.close_reached_min_shape
                and dom_sv >= self.close_reached_min_sv
                and dom_width_frac <= self.close_reached_max_width_frac
                and (dom_aspect <= self.close_reached_max_aspect or dom_shape >= 0.56)
            )
            close_reached_ok = (
                reached
                and dom_occ >= 0.18
                and dom_hue >= 0.60
                and reached_shape_ok
                and prob >= self.close_reached_min_prob
                and (dominant_roi_support >= 0.10 or dom_occ >= 0.24)
                and dom_ground_penalty >= 0.5
            )
            close_reached_failures = []
            if not reached:
                close_reached_failures.append("raw_reached_false")
            if dom_occ < 0.18:
                close_reached_failures.append("occupancy_below_min")
            if dom_hue < 0.60:
                close_reached_failures.append("hue_below_min")
            if not reached_shape_ok:
                close_reached_failures.append("shape_gate_failed")
            if prob < self.close_reached_min_prob:
                close_reached_failures.append("probability_below_min")
            if dominant_roi_support < 0.10 and dom_occ < 0.24:
                close_reached_failures.append("roi_or_occupancy_gate_failed")
            if dom_ground_penalty < 0.5:
                close_reached_failures.append("ground_band_rejected")
            close_reached_reject_reason = "|".join(close_reached_failures)

        if close_reached_ok:
            is_detected = True
            prob = 1.0
            if dominant_red is not None:
                best_component = dominant_red_component
                bbox = dominant_red["bbox"]
                centroid = dominant_red["centroid"]
                cone_dir = float(centroid[0]) / float(self.camera_width)
                occupancy = max(
                    occupancy,
                    float(dominant_red["area"]) / float(self.camera_width * self.camera_height),
                )
            elif centroid is None and red_mask is not None and np.count_nonzero(red_mask) > 0:
                m = cv2.moments(red_mask)
                if m["m00"] > 0:
                    cx = int(m["m10"] / m["m00"])
                    cy = int(m["m01"] / m["m00"])
                    centroid = [cx, cy]
                    cone_dir = float(cx) / float(self.camera_width)

        pre_filter_probability = prob

        if cone_dir is None:
            cone_dir = 0.5

        if best_component is not None:
            top = int(best_component["bbox"][1])
            height = int(best_component["bbox"][3])
            width_frac = float(best_component.get("width_frac", 0.0))
            height_frac = float(best_component.get("height_frac", 0.0))
            aspect = float(best_component.get("aspect", 0.0))
            cone_shape_score = float(best_component.get("cone_shape_score", 0.0))
            hue_redness_score = float(best_component.get("hue_redness_score", 0.0))
            sv_score = float(best_component.get("sv_score", 0.0))
            bbox_top_frac = float(top) / float(self.camera_height)
            bbox_bottom_frac = float(top + height) / float(self.camera_height)
            if (
                bbox_top_frac < 0.22
                and width_frac > 0.38
                and aspect > 1.10
                and cone_shape_score < 0.62
                and roi_support_ratio < 0.10
            ):
                prob *= 0.45
                penalty_flags.append("wide_upper_x0.45")
            if (
                bbox_top_frac < self.upper_sky_reject_top_frac
                and bbox_bottom_frac < 0.72
                and width_frac > self.upper_sky_reject_width_frac
                and height_frac < 0.55
                and cone_shape_score < 0.72
                and roi_support_ratio < 0.10
            ):
                prob *= 0.18
                penalty_flags.append("upper_sky_x0.18")
            ground_penalty = self.__ground_band_penalty(
                width_frac,
                height_frac,
                bbox_bottom_frac,
                aspect,
                cone_shape_score,
            )
            prob *= ground_penalty
            if ground_penalty < 0.999:
                penalty_flags.append(f"ground_x{ground_penalty:.2f}")
            strict_red_reject_reason = self.__strict_red_candidate_reject_reason(
                best_component,
                roi_support_ratio,
            )
            strict_red_ok = not strict_red_reject_reason
            if not strict_red_ok:
                if self.__relaxed_red_candidate_ok(best_component, strict_red_reject_reason):
                    relaxed_penalty = self.__relaxed_red_penalty(
                        best_component,
                        strict_red_reject_reason,
                    )
                    prob = min(
                        prob * relaxed_penalty,
                        self.relaxed_red_probability_cap,
                    )
                    penalty_flags.append(f"strict_red_relaxed_x{relaxed_penalty:.2f}")
                else:
                    prob *= 0.08
                    penalty_flags.append("strict_red_x0.08")
            quality_floor = min(cone_shape_score, hue_redness_score, sv_score)
            if (
                bbox_bottom_frac < 0.48
                and occupancy < 0.05
                and cone_shape_score < 0.58
                and hue_redness_score < 0.62
            ):
                prob *= 0.45
                penalty_flags.append("small_upper_weak_x0.45")
            if (
                occupancy < 0.10
                and quality_floor < self.min_detect_quality_floor
            ):
                prob *= 0.35
                penalty_flags.append("low_quality_x0.35")
            if bp_mask is not None:
                pure_red_strong = (
                    cone_shape_score >= 0.72
                    and hue_redness_score >= 0.60
                    and sv_score >= 0.66
                    and occupancy >= 0.020
                    and bbox_bottom_frac >= 0.42
                )
                if roi_support_ratio < 0.08:
                    if best_mode == "hue" and pure_red_strong:
                        prob *= 0.80
                        penalty_flags.append("roi_low_strong_hue_x0.80")
                    else:
                        prob *= 0.55
                        penalty_flags.append("roi_low_x0.55")
                elif roi_support_ratio < self.roi_hist_min_support_ratio:
                    if best_mode == "hue" and pure_red_strong:
                        prob *= 0.92
                        penalty_flags.append("roi_partial_strong_hue_x0.92")
                    elif best_mode == "hue":
                        prob *= 0.35
                        penalty_flags.append("roi_partial_hue_x0.35")
                    else:
                        prob *= 0.78
                        penalty_flags.append("roi_partial_x0.78")
                elif best_mode in ("backproj", "hybrid_overlap", "hybrid_union"):
                    prob = max(prob, min(0.36, 0.18 + 2.8 * occupancy))
                    penalty_flags.append("roi_supported_floor")
                if best_mode == "backproj" and roi_support_ratio < 0.20:
                    prob *= 0.22
                    penalty_flags.append("backproj_low_roi_x0.22")
                # Proven detector behavior is occupancy-first on backprojection.
                # Rescue that path when ROI support is real and sky-like penalties are not triggered.
                if (
                    best_mode == "backproj"
                    and occupancy >= self.backproj_rescue_min_occupancy
                    and roi_support_ratio >= 0.24
                    and bbox_bottom_frac >= 0.45
                    and hue_redness_score >= 0.42
                    and sv_score >= 0.20
                ):
                    prob = max(prob, min(0.82, 0.34 + 5.5 * occupancy))
                    penalty_flags.append("backproj_rescue_floor")
            prob, lower_rescued = self.__small_lower_rescue(
                prob,
                strict_red_ok=strict_red_ok,
                bbox_bottom_frac=bbox_bottom_frac,
                occupancy=occupancy,
                cone_shape_score=cone_shape_score,
                hue_redness_score=hue_redness_score,
                sv_score=sv_score,
            )
            if lower_rescued:
                penalty_flags.append("small_lower_cone_floor")

        legacy_prob = 0.0

        # Internal detection state must describe the final penalized score, not
        # the pre-filter component score.
        is_detected = bool(reached or prob >= self.min_detect_probability)

        # Candidate quality to resolve RGB/BGR ambiguity across camera setups.
        overall_score = (
            (2.0 if reached else 0.0)
            + prob
            + 0.20 * min(frame_red_occupancy / 0.6, 1.0)
            + (0.04 if self.__roi_hist is not None and best_mode and "hybrid" in best_mode else 0.0)
        )

        result = {
            "variant_name": variant_name,
            "bgr_img": bgr_img,
            "projected_img": proj if proj is not None else red_mask_raw,
            "binarized_img": best_mask if best_mask is not None else red_mask,
            "bbox": bbox,
            "centroid": centroid,
            "cone_direction": cone_dir,
            "occupancy": occupancy,
            "frame_red_occupancy": frame_red_occupancy,
            "is_detected": is_detected,
            "is_reached": reached,
            "probability": prob,
            "debug_method": f"{variant_name}:{best_mode or 'none'}",
            "overall_score": overall_score,
            "edge_touch_count": edge_touch,
            "roi_support_ratio": roi_support_ratio,
            "candidate_count": candidate_count,
            "candidate_rank": candidate_rank,
            "candidate_temporal_bonus": candidate_temporal_bonus,
            "legacy_prob": legacy_prob,
            "candidate_probability": candidate_probability,
            "pre_filter_probability": pre_filter_probability,
            "raw_reached": reached,
            "close_reached_ok": close_reached_ok,
            "strict_red_ok": strict_red_ok,
            "strict_red_reject_reason": strict_red_reject_reason,
            "close_region_ok": close_region_ok,
            "dominant_close": dominant_close,
            "close_reached_reject_reason": close_reached_reject_reason,
            "ground_penalty": ground_penalty,
            "penalty_flags": "|".join(penalty_flags),
            "roi_hist_available": self.__roi_hist is not None,
            "image_direction": float(cone_dir),
            "image_direction_valid": int(best_component is not None),
        }
        if best_component is not None:
            result.update(best_component)
        result_bbox = result.get("bbox") or [0, 0, 0, 0]
        result.update(
            {
                "bbox_x": int(result_bbox[0]),
                "bbox_y": int(result_bbox[1]),
                "bbox_width": int(result_bbox[2]),
                "bbox_height": int(result_bbox[3]),
                "bbox_width_frac": float(result_bbox[2]) / float(self.camera_width),
                "bbox_height_frac": float(result_bbox[3]) / float(self.camera_height),
                "bbox_bottom_frac": float(result_bbox[1] + result_bbox[3]) / float(self.camera_height),
                "bbox_aspect": float(result_bbox[2]) / max(float(result_bbox[3]), 1.0),
            }
        )
        return result

    def detect_cone(self):
        self.is_detected = False
        self.is_reached = False
        self.probability = 0.0
        self.cone_direction = None
        self.occupancy = 0.0
        self.frame_red_occupancy = 0.0
        self.centroids = None
        self.projected_img = None
        self.binarized_img = None
        self.debug_method = "reset"
        self.debug_scores = {}

        raw_img = self.__get_camera_img()
        if raw_img is None:
            return False

        # Re-evaluate with swap_rb because some Picamera2 setups still deliver RGB-like
        # arrays in practice. Keep the rescue path heavily gated to preserve false-positive resistance.
        variant_bgr = raw_img
        cand1 = self.__build_candidate(variant_bgr, "as_is")
        cand1_select = self.__variant_selection_score(cand1)
        best = cand1
        cand2 = None
        cand2_select = -1.0
        score_margin = 0.0
        if self.allow_swap_rb_rescue:
            cand2 = self.__build_candidate(cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR), "swap_rb")
            cand2_select = self.__variant_selection_score(cand2)
            score_margin = cand2_select - cand1_select
            cand1_prob = float(cand1.get("probability", 0.0))
            cand2_prob = float(cand2.get("probability", 0.0))
            cand1_hue = float(cand1.get("hue_redness_score", 0.0))
            cand2_hue = float(cand2.get("hue_redness_score", 0.0))
            cand1_shape = float(cand1.get("cone_shape_score", 0.0))
            cand2_shape = float(cand2.get("cone_shape_score", 0.0))
            swap_force_win = (
                score_margin >= self.swap_rb_force_margin
                and cand2_prob >= self.swap_rb_min_probability
                and cand2_hue >= max(self.swap_rb_min_hue, cand1_hue + 0.08)
                and cand2_shape >= max(self.swap_rb_min_shape, cand1_shape - 0.05)
            )
            swap_rescue_win = (
                cand1_prob < self.min_detect_probability
                and cand2_prob >= self.swap_rb_min_probability
            )
            if self.__swap_rb_candidate_is_trustworthy(cand2) and (
                swap_force_win
                or score_margin >= self.variant_selection_margin
                or swap_rescue_win
            ):
                best = cand2

        self.input_img = best["bgr_img"]
        self.projected_img = best["projected_img"]
        self.binarized_img = best["binarized_img"]
        self.detected = best["bbox"]
        raw_probability = float(best["probability"])
        self.probability = self.__temporal_stabilize_probability(raw_probability, best)
        self.centroids = np.array(best["centroid"], dtype=float) if best["centroid"] is not None else None
        self.cone_direction = float(best["cone_direction"])
        self.occupancy = float(best["occupancy"])
        self.frame_red_occupancy = float(best["frame_red_occupancy"])
        self.is_detected = bool(best["is_detected"])
        self.is_reached = bool(best["is_reached"])
        self.debug_method = best["debug_method"]
        self.debug_scores = {
            "schema_version": CONE_DIAGNOSTIC_SCHEMA_VERSION,
            "is_detected": int(bool(self.is_detected)),
            "probability_detected": int(self.probability >= self.min_detect_probability),
            "is_reached": int(bool(self.is_reached)),
            "raw_reached": int(bool(best.get("raw_reached", False))),
            "close_reached_ok": int(bool(best.get("close_reached_ok", False))),
            "raw_probability": raw_probability,
            "candidate_probability": float(best.get("candidate_probability", 0.0)),
            "pre_filter_probability": float(best.get("pre_filter_probability", 0.0)),
            "occupancy": float(best.get("occupancy", 0.0)),
            "frame_red_occupancy": float(best.get("frame_red_occupancy", 0.0)),
            "shape": float(best.get("shape_score", 0.0)),
            "cone_shape": float(best.get("cone_shape_score", 0.0)),
            "aspect": float(best.get("aspect_score", 0.0)),
            "sv": float(best.get("sv_score", 0.0)),
            "hue": float(best.get("hue_redness_score", 0.0)),
            "occ": float(best.get("occupancy", 0.0)),
            "edge_touch": float(best.get("edge_touch_count", 0.0)),
            "roi_support": float(best.get("roi_support_ratio", 0.0)),
            "image_direction": float(best.get("image_direction", 0.5)),
            "image_direction_valid": int(best.get("image_direction_valid", 0)),
            "candidate_count": int(best.get("candidate_count", 0)),
            "candidate_rank": int(best.get("candidate_rank", 0)),
            "candidate_temporal_bonus": float(
                best.get("candidate_temporal_bonus", 0.0)
            ),
            "shape_score": float(best.get("shape_score", 0.0)),
            "cone_shape_score": float(best.get("cone_shape_score", 0.0)),
            "sv_score": float(best.get("sv_score", 0.0)),
            "hue_redness_score": float(best.get("hue_redness_score", 0.0)),
            "roi_support_ratio": float(best.get("roi_support_ratio", 0.0)),
            "edge_touch_count": int(best.get("edge_touch_count", 0)),
            "bbox_x": int(best.get("bbox_x", 0)),
            "bbox_y": int(best.get("bbox_y", 0)),
            "bbox_width": int(best.get("bbox_width", 0)),
            "bbox_height": int(best.get("bbox_height", 0)),
            "bbox_width_frac": float(best.get("bbox_width_frac", 0.0)),
            "bbox_height_frac": float(best.get("bbox_height_frac", 0.0)),
            "bbox_bottom_frac": float(best.get("bbox_bottom_frac", 0.0)),
            "bbox_aspect": float(best.get("bbox_aspect", 0.0)),
            "strict_red_ok": int(bool(best.get("strict_red_ok", False))),
            "strict_red_reject_reason": str(best.get("strict_red_reject_reason", "")),
            "close_region_ok": int(bool(best.get("close_region_ok", False))),
            "dominant_close": int(bool(best.get("dominant_close", False))),
            "close_reached_reject_reason": str(best.get("close_reached_reject_reason", "")),
            "ground_penalty": float(best.get("ground_penalty", 1.0)),
            "penalty_flags": str(best.get("penalty_flags", "")),
            "roi_hist_available": int(bool(best.get("roi_hist_available", False))),
            "swap_margin": float(score_margin),
            "swap_used": 1.0 if (cand2 is not None and best is cand2) else 0.0,
            "raw_prob": raw_probability,
            "stabilized_prob": float(self.probability),
            "as_is_prob": float(cand1.get("probability", 0.0)),
            "swap_prob": float(cand2.get("probability", 0.0)) if cand2 is not None else 0.0,
            "as_is_shape": float(cand1.get("cone_shape_score", 0.0)),
            "swap_shape": float(cand2.get("cone_shape_score", 0.0)) if cand2 is not None else 0.0,
            "as_is_select": float(cand1_select),
            "swap_select": float(cand2_select),
            "roi_pos_count": float(self.roi_positive_count),
            "roi_neg_count": float(self.roi_negative_count),
            "roi_pos_weight": float(self.roi_positive_weight),
            "roi_neg_weight": float(self.roi_negative_weight),
            "roi_positive_count": int(self.roi_positive_count),
            "roi_negative_count": int(self.roi_negative_count),
            "roi_positive_weight": float(self.roi_positive_weight),
            "roi_negative_weight": float(self.roi_negative_weight),
            "as_is_probability": float(cand1.get("probability", 0.0)),
            "swap_probability": float(cand2.get("probability", 0.0)) if cand2 is not None else 0.0,
        }

        self.__save_periodic_evidence()

        if self.is_reached:
            try:
                self.picam2.capture_file(str(self.capture_reached_path))
            except Exception:
                pass

        return True
