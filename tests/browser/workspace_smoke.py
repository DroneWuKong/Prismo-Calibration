"""Optional real-browser check: pip install playwright; playwright install chromium.

Run from an environment with Prismo Calibration installed:
    python tests/browser/workspace_smoke.py --artifacts /tmp/prismo-browser
"""
import argparse
import io
import json
from pathlib import Path
import tempfile
import threading
import zipfile

import cv2
import numpy as np
from playwright.sync_api import sync_playwright
from prismo_calibration import calibration_app as app
from prismo_calibration.calibration_examples import example


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--artifacts', type=Path, required=True)
    parser.add_argument('--browser-executable')
    args = parser.parse_args()
    args.artifacts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='prismo-browser-') as output:
        server = app.ThreadingHTTPServer(('127.0.0.1', 0), app.Handler)
        server.session = app.Session(output, 'unused')
        server.ui = Path(app.__file__).with_suffix('.html')
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with sync_playwright() as p:
                browser = p.chromium.launch(headless=True, executable_path=args.browser_executable)
                page = browser.new_page(viewport={'width': 1440, 'height': 1100})
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                base = f'http://127.0.0.1:{server.server_port}'
                page.goto(base)
                page.locator('.card').nth(3).wait_for()
                assert page.locator('.card').count() == 4
                assert page.locator('#integration').input_value() == 'orqa'
                assert page.get_by_role('link', name='Start guided setup').count() == 0
                page.click('#checkSetup')
                page.get_by_text('Recording imports · Ready').wait_for()
                page.screenshot(path=str(args.artifacts / 'home-desktop.png'), full_page=True)
                cases = ((0, 'camera', 'orqa_apb', 'orqa', 'unknown'),
                         (1, 'camera_imu', 'orqa_f405', 'orqa', 'ardupilot'),
                         (2, 'optical_flow', 'orqa_f405', 'prismo', 'ardupilot'),
                         (3, 'camera_rig', 'orqa_h7', 'prismo', 'px4'))
                for index, kind, profile, integration, firmware in cases:
                    page.select_option('#profile', profile)
                    page.select_option('#integration', integration)
                    page.select_option('#firmware', firmware)
                    page.fill('#firmwareVersion', 'browser-fixture-1')
                    page.get_by_role('button', name='Import & calibrate').nth(index).click()
                    data = example(kind)
                    page.set_input_files('#recording', {'name': 'recording.json', 'mimeType': 'application/json', 'buffer': json.dumps(data).encode()})
                    if kind == 'camera_rig':
                        page.click('#solve')
                        page.get_by_text('Confirm target corner order, rigidity and capture timing first.').wait_for()
                        page.check('#correspondence')
                    page.click('#solve')
                    page.get_by_role('heading', name='Example only · synthetic measurements').wait_for()
                    assert page.locator('#metrics .metric').count() == 4
                    with page.expect_download() as event:
                        page.click('#download')
                    path = event.value.path()
                    with zipfile.ZipFile(io.BytesIO(path.read_bytes())) as archive:
                        result = json.loads(archive.read('result.json'))
                        assert result['device_profile'] == profile
                        assert result['setup']['hardware'] == profile
                        assert result['setup']['integration'] == integration
                        assert result['setup']['firmware'] == firmware
                        assert result['setup']['firmware_version'] == 'browser-fixture-1'
                        assert result['data_origin'] == 'synthetic'
                        assert not result['bench_validated'] and result['simulation_passed']
                    page.screenshot(path=str(args.artifacts / f'{kind}-result.png'), full_page=True)
                    page.click('#back')
                page.reload()
                page.locator('#history .session').nth(3).wait_for()
                assert page.locator('#history .session').count() == 4
                assert page.locator('#profile').input_value() == 'orqa_h7'
                assert page.get_by_role('link', name='Start guided setup').first.get_attribute('aria-disabled') == 'true'
                # Bad files show a recoverable error and cannot create a session.
                page.get_by_role('button', name='Import & calibrate').first.click()
                page.set_input_files('#recording', {'name': 'bad.json', 'mimeType': 'application/json', 'buffer': b'not json'})
                page.click('#solve')
                page.get_by_text('This file could not be read. Choose a Prismo JSON recording.').wait_for()
                assert page.locator('#result').is_hidden()
                page.click('#back')
                # Exercise the ordinary-photo upload path, confirmation, target
                # helper and server error recovery with a blank image fixture.
                page.get_by_role('button', name='Import & calibrate').first.click()
                page.click('#usePhotos')
                ok, blank = cv2.imencode('.png', np.full((480, 640), 255, np.uint8))
                assert ok
                def photos(count):
                    return [{'name': f'photo-{i}.png', 'mimeType': 'image/png', 'buffer': blank.tobytes()} for i in range(count)]
                page.set_input_files('#trainingPhotos', photos(24))
                page.set_input_files('#validationPhotos', photos(6))
                page.click('#solve')
                page.get_by_text('Confirm the measured square size and fixed camera setup first.').wait_for()
                page.check('#photoScaleVerified')
                page.click('#solve')
                page.get_by_text('training photo 1: Show all 8 × 6 squares and the white border in every photo').wait_for()
                assert page.locator('#result').is_hidden()
                page.screenshot(path=str(args.artifacts / 'photo-workflow.png'), full_page=True)
                with page.expect_popup() as popup_event:
                    page.click('#targetHelper')
                target = popup_event.value
                target.locator('#start').wait_for()
                assert target.locator('#start').is_disabled()
                assert target.locator('#square').input_value() == '25'
                target.close()
                page.click('#back')
                # Simulated readiness tests the navigation gate only. It is not
                # evidence that a physical APB or FC was connected.
                page.route('**/api/workspace/check', lambda route: route.fulfill(json={
                    'guided_ready': True, 'checks': [{'name': 'Browser fixture', 'status': 'ready', 'message': 'Synthetic readiness fixture'}],
                    'capabilities': {'note': 'Fixture'}}))
                page.click('#checkSetup')
                page.get_by_text('Ready for guided Prismo recording or saved recordings.').wait_for()
                page.get_by_role('link', name='Start guided setup').first.click()
                page.get_by_role('heading', name='Match the board to your screen').wait_for()
                assert 'Orqa H7' in page.locator('#captureSetup').inner_text()
                assert 'with Prismo' in page.locator('#captureSetup').inner_text()
                page.get_by_role('link', name='← All sensors').click()
                page.set_viewport_size({'width': 390, 'height': 844})
                page.locator('.card').nth(3).wait_for()
                assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
                page.screenshot(path=str(args.artifacts / 'home-mobile.png'), full_page=True)
                assert errors == [], errors
                browser.close()
                print('Browser checks passed: Orqa/Prismo setup, all four imports, readiness/validation gates, ZIP reports, firmware provenance, history, errors, camera navigation, mobile layout.')
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    main()
