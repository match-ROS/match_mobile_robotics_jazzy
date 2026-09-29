import base64

import pytest

from mir_launch_hardware.mir_camera_bridge import convert_camera_info, convert_image


def test_rosbridge_image_preserves_bytes_and_namespaces_frame():
    source = {
        'header': {'stamp': {'secs': 123, 'nsecs': 456}, 'frame_id': 'camera_color_optical_frame'},
        'height': 1,
        'width': 2,
        'encoding': 'rgb8',
        'is_bigendian': 0,
        'step': 6,
        'data': base64.b64encode(bytes((1, 2, 3, 4, 5, 6))).decode(),
    }
    image = convert_image(source, 'mur620c')
    assert image.header.frame_id == 'mur620c/camera_color_optical_frame'
    assert image.header.stamp.sec == 123
    assert image.header.stamp.nanosec == 456
    assert image.encoding == 'rgb8'
    assert bytes(image.data) == bytes((1, 2, 3, 4, 5, 6))


def test_rosbridge_depth_rejects_truncated_payload():
    source = {
        'height': 2, 'width': 2, 'encoding': '16UC1', 'step': 4,
        'data': base64.b64encode(b'\x01\x00').decode(),
    }
    with pytest.raises(ValueError, match='payload'):
        convert_image(source, 'mur620c')


def test_camera_info_preserves_calibration_and_roi():
    source = {
        'header': {'frame_id': 'mur620d/camera_depth_optical_frame'},
        'height': 240, 'width': 320, 'distortion_model': 'plumb_bob',
        'D': [0, 1, 2, 3, 4], 'K': list(range(9)),
        'R': list(range(9)), 'P': list(range(12)),
        'binning_x': 2, 'binning_y': 3,
        'roi': {'x_offset': 4, 'y_offset': 5, 'height': 6, 'width': 7, 'do_rectify': True},
    }
    info = convert_camera_info(source, 'mur620d')
    assert info.header.frame_id == 'mur620d/camera_depth_optical_frame'
    assert list(info.d) == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert list(info.k) == list(range(9))
    assert info.roi.x_offset == 4
    assert info.roi.do_rectify is True
