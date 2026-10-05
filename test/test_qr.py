import cv2
import pytest

from match_mir_camera_calibration.qr import advertised_host, qr_image, terminal_qr


@pytest.mark.parametrize('host', ['10.145.8.71', 'rosmaster.local'])
def test_saved_qr_decodes_to_full_phone_link_with_session_token(tmp_path, host):
    url = f'http://{host}:8080/?token=A_b-c0123456789012345678901234567'
    pixels = qr_image(url)
    path = tmp_path/'iphone.png'
    assert cv2.imwrite(str(path), pixels)
    decoded, corners, _ = cv2.QRCodeDetector().detectAndDecode(cv2.imread(str(path)))
    assert corners is not None and decoded == url
    assert '\033[30;47m' in terminal_qr(url)


def test_explicit_binding_uses_requested_host_for_qr_link():
    assert advertised_host('10.145.8.71', 'unavailable-observer') == '10.145.8.71'
