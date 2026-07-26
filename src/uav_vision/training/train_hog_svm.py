#!/usr/bin/env python3
"""Train and evaluate the bundled compact learned QR region classifier."""

import argparse
import json
from pathlib import Path
import platform
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np


HOG = cv2.HOGDescriptor((64, 64), (16, 16), (8, 8), (8, 8), 9)


def labels(path, shape):
    result = []
    if not path.is_file():
        return result
    height, width = shape[:2]
    for line in path.read_text().splitlines():
        _, cx, cy, w, h = map(float, line.split())
        result.append((int((cx - w / 2) * width),
                       int((cy - h / 2) * height),
                       int(w * width), int(h * height)))
    return result


def features(dataset, split):
    """Extract positives plus deterministic background negatives."""
    xs, ys = [], []
    for image_path in sorted((dataset / 'images' / split).glob('*.jpg')):
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        boxes = labels(
            dataset / 'labels' / split / (image_path.stem + '.txt'),
            image.shape)
        for x, y, width, height in boxes:
            x0, y0 = max(0, x), max(0, y)
            crop = image[y0:min(image.shape[0], y + height),
                         x0:min(image.shape[1], x + width)]
            if crop.size:
                xs.append(HOG.compute(cv2.resize(crop, (64, 64))).ravel())
                ys.append(1)
        for x, y in ((0, 0), (image.shape[1] - 80, 0),
                     (0, image.shape[0] - 80)):
            crop = image[max(0, y):max(0, y) + 80,
                         max(0, x):max(0, x) + 80]
            if crop.shape == (80, 80):
                xs.append(HOG.compute(cv2.resize(crop, (64, 64))).ravel())
                ys.append(-1)
    return np.asarray(xs, np.float32), np.asarray(ys, np.int32)


def score(model, x, y):
    _, predicted = model.predict(x)
    predicted = predicted.ravel().astype(int)
    tp = int(np.sum((predicted == 1) & (y == 1)))
    fp = int(np.sum((predicted == 1) & (y == -1)))
    fn = int(np.sum((predicted == -1) & (y == 1)))
    tn = int(np.sum((predicted == -1) & (y == -1)))
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    return {'precision': precision, 'recall': recall,
            'accuracy': (tp + tn) / len(y),
            'confusion_matrix': [[tn, fp], [fn, tp]]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', default='datasets/qr_shelf')
    parser.add_argument('--output', default='models/qr_detector')
    parser.add_argument('--smoke', action='store_true')
    args = parser.parse_args()
    dataset, output = Path(args.dataset), Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    train_x, train_y = features(dataset, 'train')
    val_x, val_y = features(dataset, 'val')
    if args.smoke:
        train_x, train_y = train_x[:80], train_y[:80]
    model = cv2.ml.SVM_create()
    model.setType(cv2.ml.SVM_C_SVC)
    model.setKernel(cv2.ml.SVM_LINEAR)
    model.setC(0.8)
    started = time.perf_counter()
    model.train(train_x, cv2.ml.ROW_SAMPLE, train_y)
    train_seconds = time.perf_counter() - started
    model_path = output / ('qr_hog_svm_smoke.xml' if args.smoke
                           else 'qr_hog_svm.xml')
    model.save(str(model_path))
    metrics = score(model, val_x, val_y)
    sample = val_x[:min(200, len(val_x))]
    started = time.perf_counter()
    for row in sample:
        model.predict(row.reshape(1, -1))
    metrics.update({
        'model': 'OpenCV linear SVM + HOG (64x64)',
        'opencv': cv2.__version__, 'python': platform.python_version(),
        'train_samples': len(train_y), 'validation_samples': len(val_y),
        'train_seconds': train_seconds,
        'patch_latency_ms': (time.perf_counter() - started) * 1000 /
                            max(1, len(sample)),
        'configuration': {'C': 0.8, 'kernel': 'linear',
                          'input': [64, 64], 'hog_bins': 9}})
    metrics_path = output / (
        'smoke_metrics.json' if args.smoke else 'metrics.json')
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding='utf-8')
    matrix = np.asarray(metrics['confusion_matrix'])
    fig, axis = plt.subplots(figsize=(4, 4))
    axis.imshow(matrix, cmap='Blues')
    for row in range(2):
        for column in range(2):
            axis.text(column, row, str(matrix[row, column]), ha='center')
    axis.set(xticks=[0, 1], yticks=[0, 1],
             xticklabels=['background', 'QR'],
             yticklabels=['background', 'QR'],
             xlabel='predicted', ylabel='actual',
             title='QR patch classifier')
    fig.tight_layout()
    fig.savefig(output / ('smoke_confusion.png' if args.smoke
                          else 'confusion_matrix.png'), dpi=130)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
