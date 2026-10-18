"""Bounded JPEG/PNG target-photo adapter; image order is not a capture clock."""
import base64
import binascii
import struct

import cv2
import numpy as np


def dimensions(raw):
    if raw.startswith(b'\x89PNG\r\n\x1a\n') and raw[12:16] == b'IHDR' and len(raw) >= 24:
        return struct.unpack('>II', raw[16:24])
    if raw[:2] != b'\xff\xd8':
        raise ValueError('Use JPEG or PNG checkerboard photos')
    index = 2
    while index + 4 <= len(raw):
        if raw[index] != 255:
            break
        while index < len(raw) and raw[index] == 255:
            index += 1
        if index >= len(raw):
            break
        marker = raw[index]
        index += 1
        if marker in (0xd9, 0xda):
            break
        if marker in (0x01, *range(0xd0, 0xd9)):
            continue
        if index + 2 > len(raw):
            break
        length = int.from_bytes(raw[index:index + 2], 'big')
        if length < 2 or index + length > len(raw):
            break
        if marker in (0xc0, 0xc1, 0xc2):
            if length < 8:
                break
            height, width = struct.unpack('>HH', raw[index + 3:index + 7])
            return width, height
        index += length
    raise ValueError('Could not read the JPEG dimensions; use standard JPEG or PNG photos')


def photo_corners(encoded):
    if not isinstance(encoded, str) or len(encoded) > 1024 * 1024:
        raise ValueError('Use photos below 750 KB each; keep the original geometry and resolution')
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError('Photo could not be decoded') from exc
    width, height = dimensions(raw)
    if min(width, height) < 80 or max(width, height) > 4096 or width * height > 12_000_000:
        raise ValueError('Use photos between 80 pixels and 12 megapixels (maximum side 4096 pixels)')
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None or image.shape != (height, width):
        raise ValueError('Photo could not be read at its declared resolution')
    factor = min(1., 1200 / max(width, height))
    small = cv2.resize(image, (round(width * factor), round(height * factor))) if factor < 1 else image
    found, corners = cv2.findChessboardCorners(small, (7, 5), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        raise ValueError('Show all 8 × 6 squares and the white border in every photo')
    corners[:, :, 0] *= width / small.shape[1]
    corners[:, :, 1] *= height / small.shape[0]
    corners = cv2.cornerSubPix(image, corners, (5, 5), (-1, -1),
                              (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .01))
    return corners.reshape(35, 2).tolist(), [width, height]


def observations(data):
    converted = dict(data)
    size = None
    for name, minimum in (('training', 24), ('validation', 6)):
        group = data.get(name)
        if not isinstance(group, dict) or not isinstance(group.get('images_base64'), list):
            raise ValueError('Choose both the training and fresh-check photos')
        images = group['images_base64']
        if not minimum <= len(images) <= 48:
            raise ValueError(f'{name}: choose {minimum}–48 checkerboard photos')
        samples = []
        for index, encoded in enumerate(images):
            try:
                corners, current = photo_corners(encoded)
            except ValueError as exc:
                raise ValueError(f'{name} photo {index + 1}: {exc}') from exc
            if size is not None and current != size:
                raise ValueError('All training and check photos must have the same original resolution')
            size = current
            samples.append({'time_s': float(index + 1), 'corners_px': corners})
        converted[name] = {'recording_id': group.get('recording_id'), 'samples': samples}
    converted['image_size'] = size
    converted['time_basis'] = 'image_order_only; no camera clock or timing calibration inferred'
    return converted
