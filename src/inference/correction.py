#!/bin/env python3

from dataclasses import dataclass, asdict

import cv2
import numpy as np
from numpy.typing import NDArray
from cv2.typing import MatLike
from typing import cast, List, TypedDict, Tuple, Optional, Dict, Any
from enum import Enum, auto
from scipy.signal import savgol_filter, find_peaks, peak_widths
from scipy.ndimage import gaussian_filter1d, grey_opening
import joblib

#セグメント分割時に使用 データクラス
@dataclass
class SegmentStats:
    index: int
    y_top: int
    y_bottom: int
    valid_pixels_ratio: float
    glare_mask_ratio: float
    weight: float
    ab_variance: float
    z_score: float
    quality_score: float

@dataclass
class BackGroundStats:
    color_lab: Tuple[float, float, float]
    occupancy_ratio: float
    ab_variance: float
    resistor_body_color_mask: NDArray[np.uint8]
    is_reliable: bool

@dataclass
class DetectBand:
    x: int
    band_width: int
    mean_color_lab: Tuple[float, float, float]
    energy_score: float

@dataclass
class EstimateBand:
    sn_ratio: float
    sharpness: float
    delta_chroma: float

    symmetry: float

@dataclass
class BandColor:
    median_l: float
    median_a: float
    median_b: float
    median_c: float
    delta_chroma: float
    std_l: float

class Color(Enum):
    BLACK = 0
    BROWN = 1
    RED = 2
    ORANGE = 3
    YELLOW = 4
    GREEN = 5
    BLUE = 6
    PURPLE = 7
    GRAY = 8
    WHITE = 9
    GOLD = 10

@dataclass
class BandColorResult:
    x: int
    color: Color

RESISTOR_PARAMS = {
    Color.BLACK:     {"value": 0, "multiplier": 1,           "tolerance": None},
    Color.BROWN:     {"value": 1, "multiplier": 10,          "tolerance": 1.0},
    Color.RED:       {"value": 2, "multiplier": 100,         "tolerance": 2.0},
    Color.ORANGE:    {"value": 3, "multiplier": 1000,        "tolerance": None},
    Color.YELLOW:    {"value": 4, "multiplier": 10000,       "tolerance": None},
    Color.GREEN:     {"value": 5, "multiplier": 100000,      "tolerance": 0.5},
    Color.BLUE:      {"value": 6, "multiplier": 1000000,     "tolerance": 0.25},
    Color.PURPLE:    {"value": 7, "multiplier": 10000000,    "tolerance": 0.1},
    Color.GRAY:      {"value": 8, "multiplier": None,        "tolerance": None},
    Color.WHITE:     {"value": 9, "multiplier": None,        "tolerance": None},
    Color.GOLD:      {"value": None, "multiplier": 0.1,      "tolerance": 5.0},
}

IMG_INPUT_DIR:str = "./output_resistors"
IMG_OUTPUT_DIR:str = "./result"

#定数
MIN_EXPECTED_ASPECT_AFTER_ALIGNMENT:float = 2.5

Z_SCORE_THRESHOLD = 1.0

# ノイズモデル: Noise ≈ sqrt( ReadNoise^2 + (Gain * Signal)^2 )
# SHOT_NOISE_FACTOR: 信号強度に比例するノイズ成分 (例: 0.05 = 輝度の5%程度のゆらぎを許容)
SHOT_NOISE_FACTOR = 0.001
# READ_NOISE_EPSILON: 暗部でのゼロ除算防止および最低限のフロアノイズ
READ_NOISE_EPSILON = 2.0

# 無彩色判定の半径 (0-127 scale)
LAB_CHROMA_LIMIT = 30.0

# 局所平均を計算する窓サイズ比率
LOCAL_WINDOW_RATIO = 0.15

MIN_SPLIT_SEGMENT: int = 5

AB_VALIANCE_CLIP = 800.0

#WEIGHT_CHROMA:float = 2.0       # 色差(a,b)重視
WEIGHT_LUMA_DARK:float = 1.2    # 輝度(L)は標準
WEIGHT_LUMA_BRIGHT:float = 0.1  # テカリは抑制

# 幾何パラメータ (Geometry)
# 最小バンド幅: 物理的な最小サイズ（抵抗器の規格や解像度から決まる定数）
# ここでは画像幅の1%としているが、本来は固定ピクセル数が望ましい
MIN_BAND_WIDTH_RATIO:float = 0.01
BASELINE_WINDOW_FACTOR:int = 5   # ベースライン推定窓は最小バンド幅の何倍か

# 閾値パラメータ (Thresholds)
MAD_FACTOR:float = 1.0             # ノイズフロア推定係数
SYSTEM_NOISE_FACTOR:float = 0.6
PROMINENCE_RATIO:float = 1.5       # S/N比として、ノイズの何倍の突出を信号とするか

# 色差判定 (JND: Just Noticeable Difference)
# CIE76においてDelta E < 2.3 は「目視で差が識別できない」とされる
# ここでは安全マージンを取り 3.0 を閾値とする
JAD_AB_THRESHOLD:float = 2.3
SHADOW_LUMA_THRESHOLD:float = 20.0
#SHADOW_LUMA_THRESHOLD:float = 12.5

BAND_FEATURE_COLS = [ "sn_ratio", "sharpness", "delta_chroma", "symmetry" ]
COLOR_FEATURE_COLS = ["median_l", "median_a", "median_b", "median_c", "delta_chroma", "std_l"]

class InferenceResistorValue:
    _band_classifier = None
    _color_classifier = None
    
    def __init__(self, band_classifier_rf_model_pass: str, resistor_color_rf_model_pass: str):
        if InferenceResistorValue._band_classifier is None:
            InferenceResistorValue._band_classifier = joblib.load(band_classifier_rf_model_pass)
        if InferenceResistorValue._color_classifier is None:
            InferenceResistorValue._color_classifier = joblib.load(resistor_color_rf_model_pass)

        self._band_classifier = InferenceResistorValue._band_classifier
        self._color_classifier = InferenceResistorValue._color_classifier
    
    def inference(self, resistor_roi: np.ndarray) -> Optional[float]:
        assert self._band_classifier is not None
        assert self._color_classifier is not None
        
        if resistor_roi is None:
            return None
    
        ret = check_resistor_roi_quality(cast(NDArray[np.uint8], resistor_roi))
        if ret == False:
            return None
        
        clipped_roi = clip_resistor_roi(cast(NDArray[np.uint8], resistor_roi))
        if clipped_roi is None:
            return None

        filtered_roi = apply_bilateral_filter(clipped_roi)
        if filtered_roi is None:
            return None
            
        w = filtered_roi.shape[1]
    
        specular_mask = create_specular_mask(cast(NDArray[np.uint8], filtered_roi))
        if specular_mask is None:
            return None
    
        lab_segment, valid_pixels_ratio = select_best_segment(cast(NDArray[np.uint8], filtered_roi), specular_mask)
        if lab_segment is None:
            return None
    
        lab_one_line = compress_segment_to_one_line(lab_segment)
    
        back_ground_stats: BackGroundStats = analyze_resistor_background(lab_one_line, valid_pixels_ratio)
    
        result_bands, _ = detect_bands(lab_one_line, back_ground_stats)
    
        features = calculate_band_features(result_bands, lab_segment, back_ground_stats)
    
        valid_bands: List[DetectBand] = []
    
        for band, feat in zip(result_bands, features):
            # [band_rate, symmetry, log_norm, std_l, energy]
            feature_vector = np.array([[
                feat.sn_ratio,
                feat.sharpness,
                feat.delta_chroma,
                feat.symmetry
            ]])
            
            # 0: Band (有効), 1: Shadow/Noise (無効)
            prediction = self._band_classifier.predict(feature_vector)[0]
            
            if prediction == 0:
                valid_bands.append(band)
    
        result_bands = valid_bands
        
        result_bands_with_color: List[BandColorResult] = []
    
        color_features = extract_color_features_fixed_width(valid_bands, lab_segment, back_ground_stats)
        
        for band, feat in zip(valid_bands, color_features):
            color_vec = np.array([[
                feat.median_l,
                feat.median_a,
                feat.median_b,
                feat.median_c,
                feat.delta_chroma,
                feat.std_l
            ]])
            
            pred_id = int(self._color_classifier.predict(color_vec)[0])
            
            result_obj = BandColorResult(x=band.x, color=Color(pred_id))
            result_bands_with_color.append(result_obj)
    
        resistor_value = read_resistor_value(result_bands_with_color, w)
    
        return resistor_value

       
def check_resistor_roi_quality(roi: NDArray[np.uint8] | None) -> bool:
    """check_resistor_roi_quality

    Args1
        roi (np.ndarray): 角度補正済みROI

    Returns:
        bool: True -> 処理続行 False -> 引数として渡したROIを破棄
    """

    if roi is None:
        return False

    h, w = roi.shape[:2]
    if h < 10 or w < 30:
        return False
    
    area = h * w
    if area < 600:
        return False
    
    aspect = w / h
    if aspect < MIN_EXPECTED_ASPECT_AFTER_ALIGNMENT:
        return False
    
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    projection_y = np.sum(gray > 30, axis=1)
    raw_is_valid_mask = projection_y > (w * 0.2)

    valid_ys = np.where(raw_is_valid_mask)[0]
    if len(valid_ys) == 0:
        return False
    
    diffs = np.diff(valid_ys)

    split_indices = np.where(diffs > 1)[0] + 1
    
    segments = np.split(valid_ys, split_indices)

    max_continuous_hight = max(len(seg) for seg in segments)
    if max_continuous_hight < (h * 0.4):
        return False
    
    return True

def clip_resistor_roi(roi: NDArray[np.uint8] | None) -> NDArray[np.uint8] | None:
    """clip_resistor_roi
    YOLOで検出した画像の4辺のごみをなくすために5%カット

    Args:
        roi (np.ndarray): クオリティチェック後の画像

    Returns:
        ndarray: 4辺を5%カットした後の画像
    """
    if roi is None:
        return None

    h, w = roi.shape[:2]

    trim_w = int(w * 0.005)
    trim_h = int(h * 0.05)

    return roi[trim_h : h-trim_h, trim_w : w-trim_w]

def apply_bilateral_filter(roi: NDArray[np.uint8] | None) -> NDArray[np.uint8] | None:
    if roi is None:
        return None

    return cast(NDArray[np.uint8], cv2.bilateralFilter(roi, d=9, sigmaColor=75, sigmaSpace=75))

def create_specular_mask(
    clipped_roi: NDArray[np.uint8] | None
) -> NDArray[np.uint8] | None:
    if clipped_roi is None:
        return None
    
    if clipped_roi is None:
        return None

    h, w = clipped_roi.shape[:2]

    # ---------------------------------------------------------
    # 1. Color Space Conversion (Raw Data)
    # ---------------------------------------------------------
    # 前処理フィルタはかけず、ピーク形状を保存する
    lab = cv2.cvtColor(clipped_roi, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    
    l_float = l_ch.astype(np.float32)

    # ---------------------------------------------------------
    # 2. Chromaticity Constraint (Physics)
    # ---------------------------------------------------------
    # 中心(128)からのユークリッド距離で彩度を評価
    a_centered = a_ch.astype(np.float32) - 128.0
    b_centered = b_ch.astype(np.float32) - 128.0
    chroma = np.sqrt(a_centered**2 + b_centered**2)
    
    # Gate 1: 色度による制約 (無彩色であること)
    mask_achromatic = chroma < LAB_CHROMA_LIMIT

    # ---------------------------------------------------------
    # 3. Statistical Peak Detection (Signal-Dependent Z-score)
    # ---------------------------------------------------------
    # 窓サイズ決定
    ksize = int(w * LOCAL_WINDOW_RATIO)
    if ksize % 2 == 0: ksize += 1
    ksize = max(3, ksize)

    # モーメント計算 (BoxFilter)
    # E[X] (局所平均)
    mu = cv2.boxFilter(l_float, -1, (ksize, ksize))
    # E[X^2]
    mu2 = cv2.boxFilter(l_float**2, -1, (ksize, ksize))
    
    # Variance = E[X^2] - (E[X])^2
    variance = mu2 - mu**2
    variance = np.maximum(variance, 0)
    sigma_local = np.sqrt(variance)
    
    # --- ノイズモデルの適用 (修正箇所) ---
    # 定数フロアではなく、信号強度(mu)に依存する項を追加
    # Denominator^2 = σ_local^2 + (k * μ)^2 + ε^2
    # 明部では (k * μ) が支配的になり、Z-scoreの過敏な反応を抑える
    # 暗部では ε が支配的になり、ゼロ除算を防ぐ
    noise_model = (SHOT_NOISE_FACTOR * mu)**2 + READ_NOISE_EPSILON**2
    denominator = np.sqrt(sigma_local**2 + noise_model)
    
    # Z-score = (Signal - Mean) / Estimated_Noise_spread
    z_score = (l_float - mu) / denominator
    
    # Gate 2: 統計的有意性 (局所ピーク判定)
    mask_peak = z_score > Z_SCORE_THRESHOLD

    # ---------------------------------------------------------
    # 4. Integration
    # ---------------------------------------------------------
    # 統計条件(Peak) AND 物理条件(Achromatic)
    candidates = np.logical_and(mask_peak, mask_achromatic)
    candidates_u8 = (candidates * 255).astype(np.uint8)

    # モルフォロジー演算は行わない (微小なピーク情報を保存するため)

    # ---------------------------------------------------------
    # 5. Geometry Check
    # ---------------------------------------------------------
    contours, _ = cv2.findContours(candidates_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask = np.zeros_like(candidates_u8)
    
    # 像として成立する最小面積 (ホットピクセル除去)
    MIN_PIXEL_AREA = 2.0 

    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        if area < MIN_PIXEL_AREA:
            continue

        x, y, cw, ch = cv2.boundingRect(cnt)
        if cw == 0 or ch == 0: continue
        
        # 形状フィルタ (極端なノイズ除去)
        aspect = float(cw) / float(ch)
        if aspect < 0.1:
            continue
            
        cv2.drawContours(mask, [cnt], -1, 255, -1)
    
    return cast(NDArray[np.uint8], mask)

def select_best_segment(
    clipped_roi: NDArray[np.uint8] | None,
    glare_mask: NDArray[np.uint8] | None
) -> tuple[NDArray[np.float32] | None, float]:
   
    if clipped_roi is None or glare_mask is None:
       return None, 0.0

    roi_f = clipped_roi.astype(np.float32) / 255.0

    h, w = roi_f.shape[:2]
    roi_area = h * w

    NUM_SEGMENTS = max(1, h // MIN_SPLIT_SEGMENT)

    glare_mask_ratio = cv2.countNonZero(glare_mask) / roi_area

    lab_f = cv2.cvtColor(roi_f, cv2.COLOR_BGR2LAB)

    segment_h = h / NUM_SEGMENTS

    center_index = (NUM_SEGMENTS - 1) / 2.0
    max_dist_norm = NUM_SEGMENTS / 2.0

    stats_list: List[SegmentStats] = []
    ab_variance_list: List[float] = []

    for i in range(NUM_SEGMENTS):
        y_top = int(i * segment_h)
    
        if i < NUM_SEGMENTS - 1:
            y_bottom = int((i + 1) * segment_h)
        else:
            y_bottom = h
        
        if y_bottom <= y_top:
            continue
        
        glare_mask_seg = glare_mask[y_top:y_bottom, :]
        lab_f_seg = lab_f[y_top:y_bottom, :]

        segment_area = (y_bottom - y_top) * w
        
        glare_seg_pixels = cv2.countNonZero(glare_mask_seg)
        valid_pixels = segment_area - glare_seg_pixels

        glare_pixels_ratio = glare_seg_pixels / segment_area
        valid_pixels_ratio = valid_pixels / segment_area if segment_area > 0 else 0.0

        valid_glare_mask = cv2.bitwise_not(glare_mask_seg)

        if valid_pixels > 0:
            mean, std_deviation = cv2.meanStdDev(lab_f_seg, mask=valid_glare_mask)

            a = std_deviation[1][0] ** 2
            b = std_deviation[2][0] ** 2

            ab_variance = a + b
        else:
            ab_variance = 9999.0    #ペナルティ
        
        if ab_variance < 5000.0:    #まともな値のみを採用
            ab_variance_list.append(ab_variance)

        dist_from_center = abs(i - center_index)

        weight = 1.0 - 0.4 * (dist_from_center / max_dist_norm)

        stats_list.append(SegmentStats(i, y_top, y_bottom, float(valid_pixels_ratio), float(glare_pixels_ratio), weight, float(ab_variance), 0, 0))

        #最もいいセグメントを選択
        #優先順位 中央付近 有効ピクセル数 = roi_area * 0.2に近ければ近いほどOK((roi_are * 0.2) * 0.8)程度でOKとしておく
        #分散 後で
        #有効ピクセル数でソート
    
    if len(ab_variance_list) > 1:
        ab_variance_median = np.median(ab_variance_list)

        # mad : 中央絶対偏差
        ab_variance_mad = np.median(np.abs(np.array(ab_variance_list) - ab_variance_median))

        if ab_variance_mad < 1e-6:
            ab_variance_mad = 1.0
    else:
        ab_variance_median = 0.0
        ab_variance_mad = 1.0

    glare_ratios = [s.glare_mask_ratio for s in stats_list]
    glare_reference = np.percentile(glare_ratios, 20)
    
    for l in stats_list:
        #Quality Weight via Robust Z
        if l.ab_variance < AB_VALIANCE_CLIP:
            z_score = abs(l.ab_variance - ab_variance_median) / ab_variance_mad
        else:
            z_score = 999.0

        l.z_score = float(z_score)

#        if l.glare_mask_ratio > 0.05:
#            glare_factor = 0.0
#        elif l.glare_mask_ratio > 0.0:
#            glare_factor = 0.5
#        else:
#            glare_factor = 1.0

        glare_factor = np.exp(-(l.glare_mask_ratio - glare_reference) / (glare_reference + 1e-3))
        if l.glare_mask_ratio < glare_reference:
            glare_factor = 1.0
        
        if l.z_score > 3.5:
            l.quality_score = 0.0
        else:
            ab_variance_factor = np.exp(-0.5 * l.z_score)
        
            l.quality_score = l.valid_pixels_ratio * l.weight * ab_variance_factor * glare_factor
        
    stats_list.sort(key=lambda x: x.quality_score, reverse=True)

    return cast(NDArray[np.float32], lab_f[stats_list[0].y_top:stats_list[0].y_bottom, :]), stats_list[0].valid_pixels_ratio

def compress_segment_to_one_line(segment_lab: NDArray[np.float32]) -> NDArray[np.float32]:
    l_plane = segment_lab[:, :, 0]

    med_l = np.median(l_plane, axis=0)

    # ブロードキャストにより (H, W) - (W,) の計算が一発で行われます
    abs_diff = np.abs(l_plane - med_l)

    mad_l = np.median(abs_diff, axis=0)

    # ゼロ除算対策: MADが極小の場合は 1.0 に置換 (np.whereで条件分岐もベクトル化)
    mad_l = np.where(mad_l < 1e-6, 1.0, mad_l)

    modified_z_score = 0.6745 * abs_diff / mad_l

    valid_mask = modified_z_score <= 3.5

    # 列ごとの有効画素数をカウント -> shape: (W,)
    valid_counts = np.sum(valid_mask, axis=0)
    
    # フォールバックすべき列のフラグ -> shape: (W,)
    fallback_cols = valid_counts < 3

    # fallback_cols[np.newaxis, :] で (1, W) にして (H, W) と演算
    final_mask = valid_mask | fallback_cols[np.newaxis, :]

    masked_data = segment_lab.copy()
    masked_data[~final_mask] = np.nan

    # np.nanmedian を使い、NaNを無視して縦方向(axis=0)の中央値を計算
    # keepdims=True で (1, W, C) の形状を維持
    one_line = np.nanmedian(masked_data, axis=0, keepdims=True)

    return one_line.astype(np.float32)

def analyze_resistor_background(
    lab_one_line: NDArray[np.float32],
    valid_pixels_ratio: float
) -> BackGroundStats:

    w = lab_one_line.shape[1]

    pixels = lab_one_line[0]

    ab_pixels = pixels[:, 1:]

    #diff: (W, 1, 2) - (1, W, 2) -> (W, W, 2)
    diff = ab_pixels[:, np.newaxis, :] - ab_pixels[np.newaxis, :, :]
    dist_matrix = np.sqrt(np.sum(diff**2, axis=2))

    k_neighbor_index = int(w * valid_pixels_ratio * 0.3)
    if k_neighbor_index < 1:
        k_neighbor_index = 1
    if k_neighbor_index >= w:
        k_neighbor_index = w - 1
    
    sorted_dists = np.sort(dist_matrix, axis=1)

    tightness_scores = sorted_dists[:, k_neighbor_index]
    peak_index = int(np.argmin(tightness_scores))

    peak_dist = sorted_dists[peak_index]

    core_limit = int(w * 0.4)
    if core_limit < 1:
        core_limit = 1
    
    back_ground_spread = np.median(peak_dist[: core_limit])

    color_threshold = max(3.0, back_ground_spread * 2.5)

    dist_from_peak = dist_matrix[peak_index]

    back_ground_mask_bool = dist_from_peak < color_threshold
    back_ground_mask = back_ground_mask_bool.astype(np.uint8) * 255
    
    occupancy = float(np.count_nonzero(back_ground_mask)) / float(w)

    back_ground_pixels_lab = pixels[back_ground_mask_bool]

    if len(back_ground_pixels_lab) > 0:
        back_ground_mean_l = float(np.median(back_ground_pixels_lab[:, 0]))
        back_ground_mean_a = float(np.median(back_ground_pixels_lab[:, 1]))
        back_ground_mean_b = float(np.median(back_ground_pixels_lab[:, 2]))

        a_variance = np.var(back_ground_pixels_lab[:, 1])
        b_variance = np.var(back_ground_pixels_lab[:, 2])

        ab_variance = float(a_variance + b_variance) 
    else:
        back_ground_mean_l = 0.0
        back_ground_mean_a = 0.0
        back_ground_mean_b = 0.0

        ab_variance = 9999.0
    
    is_reliable = (occupancy >= 0.40) and (ab_variance < 200.0)

    mask_reshaped = back_ground_mask.reshape(1, w)

    result: BackGroundStats = BackGroundStats(
        (back_ground_mean_l, back_ground_mean_a, back_ground_mean_b),
        occupancy,
        ab_variance,
        mask_reshaped,
        is_reliable
    )

    return result

def calculate_dynamic_chroma_weight(ab_variance: float) -> float:
    # 基準となる標準偏差 (経験的に、綺麗な抵抗器背景は sigma=5.0~8.0 程度)
    REFERENCE_SIGMA = 10.0
    
    # 現在の背景の標準偏差
    current_sigma = np.sqrt(ab_variance)
    if current_sigma < 1.0: current_sigma = 1.0

    # 基準より綺麗なら重みが増え、汚ければ減る
    # ベースを 1.5 とし、最大 2.0倍 程度までブーストさせるイメージ
    weight = 1.5 * (REFERENCE_SIGMA / current_sigma)

    # クランプ処理
    # 最小 1.5: どんなに汚くても「金」などを検出するために最低限の色評価は必要
    # 最大 3.5: 綺麗ならかなり強気攻めてOK
    return float(np.clip(weight, 1.5, 3.5))

def detect_bands(
    lab_one_line: NDArray[np.float32],
    back_ground_stats: BackGroundStats,
    visualize: bool = False
) -> Tuple[List[DetectBand], dict]:

    w_img = lab_one_line.shape[1]
    pixels = lab_one_line[0]

    # --- 1. エネルギー信号計算 (Weighted Euclidean Distance) ---
    # 旧: ssd_signal (名称誤り) -> 新: energy_signal
    # 定義: 背景ベクトルからの「重み付き距離」
    
    # Chroma (Color distance squared)
    pixel_ab = pixels[:, 1:]
    back_ground_ab = np.array(back_ground_stats.color_lab[1:])
    diff_sq_ab = np.sum((pixel_ab - back_ground_ab)**2, axis=1)

    # Luma (Brightness distance squared with asymmetric potential)
    back_ground_l = back_ground_stats.color_lab[0]
    pixel_l = pixels[:, 0]
    delta_l = back_ground_l - pixel_l

    dynamic_weight_chroma = calculate_dynamic_chroma_weight(back_ground_stats.ab_variance)
    
    term_chroma = dynamic_weight_chroma * diff_sq_ab

    term_luma_dark = (WEIGHT_LUMA_DARK * np.maximum(0, delta_l)) ** 2
    term_luma_bright = (WEIGHT_LUMA_BRIGHT * np.minimum(np.maximum(0, -delta_l), 50.0)) ** 2

    # Unit: Distance in weighted Lab space
    energy_signal = np.sqrt(term_chroma + term_luma_dark + term_luma_bright)

    # --- 2. ベースライン補正 ---
    # 物理的な「最小バンド幅」を基準に、それより大きなうねりをベースラインとして除去
    min_band_width_px = max(3, int(w_img * MIN_BAND_WIDTH_RATIO))
    
    baseline_window_size = (min_band_width_px * BASELINE_WINDOW_FACTOR) | 1
    base_line = grey_opening(energy_signal, size=baseline_window_size)
    
    signal_corrected = energy_signal - base_line
    
    # --- 3. ノイズ除去 (Gaussian Filter) ---
    # 修正: SGフィルタ(多項式近似)は廃止。
    # 目的は「センサ起因の高周波ノイズ除去」であり、バンド幅(幾何)とは無関係。
    # sigma=1.0 は一般的なセンサノイズ抑制の標準値。
    signal_smooth = gaussian_filter1d(signal_corrected, sigma=1.0)

    # --- 4. ノイズフロア推定 (MAD) ---
    # ベースライン補正済みのため、信号の中央値は0付近と仮定できるが、
    # 念のため median を引いた偏差を見る (Robust MAD)
    signal_mad = np.median(np.abs(signal_smooth - np.median(signal_smooth)))
    if signal_mad < 1e-6: signal_mad = 1e-6

    # 最小システムノイズのガード (背景分散由来)
    back_ground_std = np.sqrt(back_ground_stats.ab_variance)
    system_noise_floor = back_ground_std * 0.8
    
    estimated_noise_floor = signal_mad * MAD_FACTOR
    final_noise_floor = max(estimated_noise_floor, system_noise_floor)

    # --- 5. ピーク検出 (Prominence Only) ---
    # 修正: height制限を撤廃 (二重拘束の解消)。
    # 「絶対的な高さ」ではなく「周囲のノイズフロアに対してどれだけ突出しているか」のみで判定する。
    min_prominence = final_noise_floor * PROMINENCE_RATIO

    peaks, properties = find_peaks(
        signal_smooth,
        prominence=min_prominence,     # 相対高さのみを見る
        width=min_band_width_px * 0.5, # 物理的な幅制約
        rel_height=0.75
    )

    result_bands = []
    
    if len(peaks) > 0:
        left_ips = properties["left_ips"]
        right_ips = properties["right_ips"]
        
        back_ground_l = back_ground_stats.color_lab[0]
        back_ground_a = back_ground_stats.color_lab[1]
        back_ground_b = back_ground_stats.color_lab[2]

        for i, x_peak in enumerate(peaks):
            x_left = left_ips[i]
            x_right = right_ips[i]
            
            index_start = max(0, int(np.floor(x_left)))
            index_end = min(w_img, int(np.ceil(x_right)))
            band_width = index_end - index_start
            
            if band_width < 2: continue

            band_pixels = pixels[index_start:index_end]
            mean_l = float(np.median(band_pixels[:, 0]))
            mean_a = float(np.median(band_pixels[:, 1]))
            mean_b = float(np.median(band_pixels[:, 2]))

            # a,b 平面での距離 (ユークリッド距離)
            dist_ab = np.sqrt((mean_a - back_ground_a)**2 + (mean_b - back_ground_b)**2)

            delta_l_band = back_ground_l - mean_l
            
            is_low_chroma_diff = (dist_ab < JAD_AB_THRESHOLD)
            is_darker = (delta_l_band > 0)
            
            is_shadow = is_low_chroma_diff and is_darker and (delta_l_band < SHADOW_LUMA_THRESHOLD)

            if is_shadow:
                continue

            result_bands.append(DetectBand(
                x=int(x_peak),
                band_width=int(band_width),
                mean_color_lab=(mean_l, mean_a, mean_b),
                energy_score=float(signal_smooth[x_peak]) 
            ))

    debug_data = {
        "energy_signal": energy_signal, # 変数名変更を反映
        "baseline": base_line,
        "signal_smooth": signal_smooth,
        "noise_floor": final_noise_floor,
        "peaks": peaks
    }

    return result_bands, debug_data
    

def scan_bands_directional(
    candidate_bands: List[DetectBand],
    pitch: float,
    is_reverse: bool
) -> List[DetectBand]:
    
    if is_reverse:
        work_list = list(reversed(candidate_bands))
    else:
        work_list = list(candidate_bands)
    
    valid_chain = [work_list[0]]
    last_x = work_list[0].x

    for i in range(1, len(work_list)):
        b = work_list[i]
        dist = abs(b.x - last_x)

        if pitch == 0:
            ratio = 0
        else:
            ratio = dist / pitch
        
        if 0.75 <= ratio <= 2.5:   #金のバンドをわざと離している抵抗器もあるので離れているのは有効とみなす
            valid_chain.append(b)

            last_x = b.x
    
    if is_reverse:
        valid_chain.reverse()
    
    return valid_chain

def calculate_band_features(
    result_bands: List[DetectBand],
    lab_segment: NDArray[np.float32],
    back_ground_stats: BackGroundStats
) -> List[EstimateBand]:
    
    if not result_bands:
        return []

    h_img, w_img, c_img = lab_segment.shape
    bg_l, bg_a, bg_b = back_ground_stats.color_lab
    bg_noise_std = np.sqrt(back_ground_stats.ab_variance) + 1e-6

    features_list = []

    for band in result_bands:
        # --- 1. 予備切り出し & 重心計算 ---
        # 探索マージン (バンド幅の75%程度外側まで見る)
        search_margin = max(5, int(band.band_width * 0.75))
        
        x_start_temp = max(0, int(band.x - band.band_width/2 - search_margin))
        x_end_temp = min(w_img, int(band.x + band.band_width/2 + search_margin))
        
        if x_end_temp - x_start_temp < 3:
            features_list.append(EstimateBand(0, 0, 0, 0))
            continue

        # Lプロファイル取得
        roi_l_temp = lab_segment[:, x_start_temp:x_end_temp, 0]
        profile_l = np.mean(roi_l_temp, axis=0)

        # 重心 (Centroid) 計算
        weights = np.maximum(0, bg_l - profile_l)
        weight_sum = np.sum(weights)
        
        if weight_sum < 1e-6:
            corrected_center_idx = band.x - x_start_temp
        else:
            indices = np.arange(len(profile_l))
            corrected_center_idx = np.sum(indices * weights) / weight_sum
        
        # --- 2. 特徴量計算 ---

        # [A] Sharpness (最大勾配)
        # エッジを見るため、ここは5pxに限定せずプロファイル全体を見る
        gradients = np.gradient(profile_l)
        sharpness = float(np.max(np.abs(gradients)))

        # [B] Delta Chroma (背景色差) - 修正箇所
        # 【変更】重心を中心に左右2px (合計5px) の固定幅で色を取得する
        FIXED_CORE_RADIUS = 2
        
        c_idx = int(corrected_center_idx)
        s_idx = max(0, c_idx - FIXED_CORE_RADIUS)
        e_idx = min(len(profile_l), c_idx + FIXED_CORE_RADIUS + 1) # +1 はスライス用
        
        # 範囲が確保できた場合のみ計算
        if e_idx > s_idx:
            roi_a = lab_segment[:, x_start_temp:x_end_temp, 1]
            roi_b = lab_segment[:, x_start_temp:x_end_temp, 2]
            
            prof_a = np.mean(roi_a, axis=0)
            prof_b = np.mean(roi_b, axis=0)
            
            # 中心5pxの平均色
            mean_a = np.mean(prof_a[s_idx:e_idx])
            mean_b = np.mean(prof_b[s_idx:e_idx])
            
            delta_chroma = float(np.sqrt((mean_a - bg_a)**2 + (mean_b - bg_b)**2))
        else:
            delta_chroma = 0.0

        # [C] Symmetry (対称性)
        # 対称性は「形状」を見るものなので、5px固定ではなく、
        # 切り出せた範囲内で最大限の半径(radius)を使って計算する
        radius = min(int(corrected_center_idx), len(profile_l) - 1 - int(corrected_center_idx))
        
        if radius < 2:
            symmetry = 0.0
        else:
            left_wing = profile_l[c_idx - radius : c_idx]
            right_wing = profile_l[c_idx + 1 : c_idx + 1 + radius]
            right_wing_flipped = right_wing[::-1]
            
            diff_sum = np.sum(np.abs(left_wing - right_wing_flipped))
            amp_sum = np.sum(np.abs(left_wing - bg_l)) + np.sum(np.abs(right_wing - bg_l)) + 1e-6
            
            symmetry = float(diff_sum / amp_sum)

        # [D] SN Ratio (対ノイズ比)
        sn_ratio = float(band.energy_score / bg_noise_std)

        features_list.append(EstimateBand(
            sn_ratio=sn_ratio,
            sharpness=sharpness,
            delta_chroma=delta_chroma,
            symmetry=symmetry
        ))

    return features_list

def extract_color_features_fixed_width(
    bands: List['DetectBand'],        
    lab_segment: NDArray[np.float32],
    back_ground_stats: 'BackGroundStats' # 【必須】色差計算のために追加
) -> List[BandColor]:
    """
    バンド中心(x)から左右2px（合計5px）の範囲で色特徴量を抽出する。
    平均(mean)ではなく中央値(median)を使用し、ノイズ耐性を高める。
    """
    h, w, c = lab_segment.shape
    features_list = []
    
    # 中心 ±2px = 幅5px
    ROI_HALF_WIDTH = 2 
    
    # 背景色情報の取得
    bg_l, bg_a, bg_b = back_ground_stats.color_lab

    for band in bands:
        # floatの座標を四捨五入して整数インデックスにする
        x_center = int(np.round(band.x))
        
        x_start = max(0, x_center - ROI_HALF_WIDTH)
        x_end = min(w, x_center + ROI_HALF_WIDTH + 1) # スライスは末尾を含まないので+1
        
        # 1. 範囲外ガード (幅が1px未満なら無効データ)
        if x_end - x_start < 1:
            features_list.append(BandColor(0, 0, 0, 0, 0, 0))
            continue

        roi = lab_segment[:, x_start:x_end, :]
        l_plane = roi[:, :, 0]
        a_plane = roi[:, :, 1]
        b_plane = roi[:, :, 2]

        # 2. 中央値 (Median) の計算
        # np.median は外れ値（境界付近の背景色混入など）を無視するのに最適です
        median_l = float(np.median(l_plane))
        median_a = float(np.median(a_plane))
        median_b = float(np.median(b_plane))

        # 3. Chroma (彩度) の計算
        # 原点(無彩色)からの距離
        median_c = float(np.sqrt(median_a**2 + median_b**2))

        # 4. Delta Chroma (背景との色差) の計算
        # 以前の議論に基づき、輝度(L)を無視したab平面上の距離を計算
        delta_chroma = float(np.sqrt((median_a - bg_a)**2 + (median_b - bg_b)**2))

        # 5. Std L (標準偏差)
        # 均一性の評価用（ここだけはばらつきを見るためstdのままが適切）
        std_l = float(np.std(l_plane))

        features_list.append(BandColor(
            median_l=median_l,
            median_a=median_a,
            median_b=median_b,
            median_c=median_c,
            delta_chroma=delta_chroma,
            std_l=std_l
        ))

    return features_list

def is_gold(c: Color) -> bool:
    if c == Color.GOLD:
        return True
    else:
        return False

def calculate_4band_value(colors: List[Color]) -> float | None:
    """
    4本のカラーリストから抵抗値を計算して文字列化する
    [Digit1, Digit2, Multiplier, Tolerance]
    """
    if len(colors) != 4:
        return None

    # パラメータ取得 (キー名を value, multiplier, tolerance に変更)
    d1 = RESISTOR_PARAMS[colors[0]]["value"]
    d2 = RESISTOR_PARAMS[colors[1]]["value"]
    mul_data = RESISTOR_PARAMS[colors[2]]
    tol_data = RESISTOR_PARAMS[colors[3]]

    if d1 is None or d2 is None:
        return None
    
    multiplier = mul_data["multiplier"]
    if multiplier is None:
        return None

    base_val = (d1 * 10) + d2
    resistance = base_val * multiplier
    
    return float(resistance)

def read_resistor_value(color_list: List[BandColorResult], roi_width: int) -> float | None:
    if not color_list:
        return None

    sorted_bands = sorted(color_list, key=lambda b: b.x)
    colors = [b.color for b in sorted_bands]
    count = len(colors)

    if count == 4:
        first_is_gold = is_gold(colors[0])
        last_is_gold = is_gold(colors[-1])

        if first_is_gold:
            colors.reverse()
            return calculate_4band_value(colors)
        elif last_is_gold:
            return calculate_4band_value(colors)
        else:
            return calculate_4band_value(colors)

    elif count == 3:
        has_gold = any(is_gold(c) for c in colors)
        
        if not has_gold:
            start_gap = sorted_bands[0].x                # 左端の空き
            end_gap = roi_width - sorted_bands[-1].x     # 右端の空き

            if start_gap > end_gap:
                colors.reverse()
                colors.append(Color.GOLD)
                return calculate_4band_value(colors)
            else:
                colors.append(Color.GOLD)
                return calculate_4band_value(colors)
        else:
            return None

    elif count == 2:
        c1, c2 = colors[0], colors[1]
        
        is_yellow_purple = (c1 == Color.YELLOW and c2 == Color.PURPLE)
        is_purple_yellow = (c1 == Color.PURPLE and c2 == Color.YELLOW)

        if is_yellow_purple or is_purple_yellow:
            return 4.7
        else:
            return None

    elif count == 1:
        c1 = colors[0]
        x_pos = sorted_bands[0].x

        center_min = roi_width * 0.3
        center_max = roi_width * 0.7
        is_center = (center_min <= x_pos <= center_max)

        if c1 == Color.BLACK and is_center:
            return 0
        else:
            return None

    return None
