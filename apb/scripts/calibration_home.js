const $ = id => document.getElementById(id);
const workflows = {
  camera: {
    title: 'Get the lens right.',
    intro: 'Measure distortion and image geometry for an APB or attached camera.',
    steps: [['1 · Measure the target', 'Use an 8 × 6 checkerboard with a ruler-checked square size. Keep the camera resolution and focus fixed.'],
      ['2 · Capture different views', 'Record at least 24 sharp views, moving the board around the image and tilting both ways.'],
      ['3 · Make a fresh check', 'Record 6 or more new tilted views. We check them using the saved lens fit.']],
    requirements: 'Import target corners, capture times, image resolution and square size. With Prismo, you can also use guided APB camera recording.'
  },
  camera_imu: {
    title: 'Make camera and motion agree.',
    intro: 'Align a camera with the flight-controller IMU and check their timing and spacing.',
    steps: [['1 · Start with a checked lens', 'Use the lens calibration for this exact camera and resolution. The recorder must preserve synchronized capture clocks.'],
      ['2 · Record varied motion', 'Keep the fixed target visible. Gently vary roll, pitch, yaw and translation, including changes of speed and direction.'],
      ['3 · Record a second movement', 'Use a new motion recording with the same fixed-target frame. We test the result before exporting it.']],
    requirements: 'Import timestamped target poses and corrected IMU samples from two recordings, plus lens calibration. This checks camera/IMU alignment; FC IMU scale calibration is a separate procedure.'
  },
  optical_flow: {
    title: 'Make movement match.',
    intro: 'Calibrate optical-flow scale, axis mapping and drift against movement you can measure.',
    steps: [['1 · Set up a flat surface', 'Use a textured, flat target. Keep the sensor level at a measured distance. Remove propellers for aircraft work.'],
      ['2 · Move in both directions', 'Record short forward, reverse, sideways and diagonal movements with measured distances. Keep rotation small.'],
      ['3 · Record a fresh check', 'Repeat varied movements in a separate recording. We check whether the calibration predicts them.']],
    requirements: 'Include gyro-compensated angular flow, target distance, integration time and measured XY displacement. At least 12 training windows and 6 new validation windows.'
  },
  camera_rig: {
    title: 'Bring your cameras together.',
    intro: 'Measure boresight and spacing, then use the adjustment angles to align a camera mount.',
    steps: [['1 · Calibrate each lens', 'Keep a lens calibration for each camera at its current resolution. Pick the reference camera that the others will align with.'],
      ['2 · Share a target', 'Show the same 8 × 6 checkerboard to every camera. Capture 12 or more synchronized views with varied tilt.'],
      ['3 · Check, adjust, repeat', 'Use 6 or more fresh views to check alignment. Review the optical-axis adjustment, move the mount, then record again.']],
    requirements: 'Import lens calibrations and corresponding target corners for 2–8 cameras. Every camera must see the same target, with a shared capture clock and matching corner order.'
  }
};
let catalog = null, kind = null, token = '', busy = false, selectedFile = null, checkedSetup = null, photoMode = false, setupKey = '';
function node(tag, text, className) {
  const el = document.createElement(tag);
  if (text !== undefined) el.textContent = text;
  if (className) el.className = className;
  return el;
}
function setup() {
  return {hardware: $('profile').value, integration: $('integration').value,
    firmware: $('firmware').value, firmware_version: $('firmwareVersion').value.trim()};
}
function setupName(value = setup()) {
  return (catalog.profiles.find(p => p.id === value.hardware)?.name || value.hardware) +
    (value.integration === 'prismo' ? ' · with Prismo' : ' · Orqa equipment');
}
async function get(path) {
  const r = await fetch(path, {cache: 'no-store'}), body = await r.json();
  if (!r.ok) throw Error(body.error || 'Connection unavailable');
  return body;
}
async function post(path, data) {
  if (!token) token = (await get('/api/token')).token;
  const r = await fetch('/api/workspace/' + path, {method: 'POST',
    headers: {'Content-Type': 'application/json', 'X-Calibration-Token': token}, body: JSON.stringify(data)});
  const body = await r.json();
  if (!r.ok) throw Error(body.error || 'Request could not be completed');
  return body;
}
function renderCards() {
  $('cards').replaceChildren();
  const live = checkedSetup?.guided_ready && setup().integration === 'prismo';
  for (const sensor of catalog.sensors) {
    const card = node('article', undefined, 'card'), actions = node('div', undefined, 'row');
    card.append(node('span', live && sensor.href ? 'Import or guided recording' : 'Saved recording', 'mode'),
      node('h2', sensor.name), node('p', sensor.description, 'muted'));
    const action = node('button', 'Import & calibrate');
    action.onclick = () => openWorkflow(sensor.id);
    actions.append(action);
    if (sensor.href && setup().integration === 'prismo') {
      const guided = node('a', 'Start guided setup', 'button secondary');
      guided.setAttribute('role', 'link');
      guided.setAttribute('aria-disabled', String(!live));
      if (live) {
        const url = new URL(sensor.href, location.origin);
        url.searchParams.set('setup', JSON.stringify(setup()));
        guided.href = url.pathname + url.search + url.hash;
      } else guided.tabIndex = -1;
      actions.append(guided);
    }
    card.append(actions);
    $('cards').append(card);
  }
}
function setupChanged() {
  const value = setup(), key = JSON.stringify(value);
  if (key === setupKey) return;
  setupKey = key;
  checkedSetup = null;
  $('checks').replaceChildren();
  $('checkNotice').textContent = '';
  $('profileNote').textContent = value.integration === 'orqa'
    ? 'Use saved recordings from your Orqa setup. Calibration and validation run locally; Prismo services are not required.'
    : 'Guided camera recording uses Prismo APB services. H7/F405 equipment needs an attached APB camera/companion. Check the connection below.';
  try { localStorage.setItem('orqaCalibrationSetup', JSON.stringify(value)); } catch (_) {}
  renderCards();
}
for (const id of ['profile', 'integration', 'firmware', 'firmwareVersion']) $(id).addEventListener('change', setupChanged);
$('firmwareVersion').addEventListener('input', setupChanged);
$('checkSetup').onclick = async () => {
  $('checkSetup').disabled = true;
  $('setupFields').disabled = true;
  $('checkNotice').textContent = 'Checking the selected setup…';
  try {
    const sent = setup();
    checkedSetup = await post('check', {device_profile: sent.hardware, setup: sent});
    $('checks').replaceChildren(...checkedSetup.checks.map(check => {
      const el = node('div', undefined, 'checkrow ' + check.status);
      el.append(node('strong', check.name + ' · ' + (check.status === 'ready' ? 'Ready' : 'Needs attention')),
        node('small', check.message));
      return el;
    }));
    $('checkNotice').textContent = checkedSetup.guided_ready ? 'Ready for guided Prismo recording or saved recordings.' : checkedSetup.capabilities.note;
    renderCards();
  } catch (e) { $('checkNotice').textContent = e.message; }
  finally { $('checkSetup').disabled = false; $('setupFields').disabled = false; }
};
function openWorkflow(next) {
  kind = next;
  const w = workflows[kind];
  $('home').classList.add('hidden'); $('workflow').classList.remove('hidden');
  $('workflowSetup').textContent = setupName();
  $('workflowTitle').textContent = w.title; $('workflowIntro').textContent = w.intro;
  $('requirements').textContent = w.requirements;
  $('steps').replaceChildren(...w.steps.map(([title, text]) => {
    const el = node('div', undefined, 'step'); el.append(node('strong', title), node('p', text, 'muted')); return el;
  }));
  $('recording').value = ''; selectedFile = null; $('solve').disabled = true;
  $('lensInputMode').classList.toggle('hidden', kind !== 'camera');
  $('trainingPhotos').value = ''; $('validationPhotos').value = ''; $('photoScaleVerified').checked = false;
  setPhotoMode(false);
  $('correspondence').checked = false; $('correspondenceLabel').classList.toggle('hidden', kind !== 'camera_rig');
  $('result').classList.add('hidden'); $('notice').textContent = '';
  $('example').href = '/api/workspace/example/' + kind;
  $('example').download = 'orqa-' + kind + '-synthetic-example.json';
  $('workflowTitle').tabIndex = -1; $('workflowTitle').focus();
}
function setPhotoMode(enabled) {
  photoMode = enabled;
  $('photoUpload').classList.toggle('hidden', !enabled);
  $('recordingUpload').classList.toggle('hidden', enabled);
  $('solve').disabled = enabled ? !($('trainingPhotos').files.length >= 24 && $('validationPhotos').files.length >= 6) : !selectedFile;
  $('result').classList.add('hidden');
  $('notice').textContent = '';
  const url = new URL('/camera', location.origin);
  url.searchParams.set('setup', JSON.stringify(setup()));
  url.searchParams.set('board_only', '1');
  url.searchParams.set('square_mm', $('photoSquare').value);
  $('targetHelper').href = url.pathname + url.search;
}
$('photoSquare').onchange = () => { if (photoMode) setPhotoMode(true); };
$('usePhotos').onclick = () => setPhotoMode(true);
$('useRecording').onclick = () => setPhotoMode(false);
for (const id of ['trainingPhotos', 'validationPhotos']) $(id).onchange = () => setPhotoMode(true);
async function photoRecording() {
  if (!$('photoScaleVerified').checked) throw Error('Confirm the measured square size and fixed camera setup first.');
  const square = Number($('photoSquare').value);
  if (!Number.isFinite(square) || square < 10 || square > 40) throw Error('Square size must be 10–40 mm.');
  const training = [...$('trainingPhotos').files], validation = [...$('validationPhotos').files];
  const all = [...training, ...validation];
  if (all.some(f => f.size > 750 * 1024) || all.reduce((sum, f) => sum + f.size, 0) > 5.8 * 1024 * 1024) throw Error('Use photos below 750 KB each and 5.8 MB combined (8 MB encoded upload).');
  async function encode(files) {
    const result = [];
    for (const file of files) result.push(await new Promise((resolve, reject) => {
      const reader = new FileReader(); reader.onerror = () => reject(Error('Could not read a photo.'));
      reader.onload = () => resolve(reader.result.split(',')[1]); reader.readAsDataURL(file);
    }));
    return result;
  }
  const id = Date.now() + '-' + crypto.randomUUID();
  return {schema: 1, kind: 'camera', data_origin: 'recording', sensor_id: $('cameraName').value.trim(),
    square_mm: square, distortion_model: 'auto', target_scale_operator_confirmed: true, camera_geometry_operator_confirmed: true,
    training: {recording_id: id + '-training', images_base64: await encode(training)},
    validation: {recording_id: id + '-validation', images_base64: await encode(validation)}};
}
function metric(title, value) {
  const item = node('div', undefined, 'metric'); item.append(node('small', title), node('strong', value)); return item;
}
function showResult(r) {
  $('result').classList.remove('hidden');
  $('resultTitle').textContent = r.data_origin === 'synthetic' ? 'Example only · synthetic measurements' : 'Recording checks passed';
  const items = [];
  if (r.kind === 'camera') {
    items.push(metric('Lens model', r.distortion_model), metric('Focal length X', r.camera_matrix[0][0].toFixed(2) + ' px'),
      metric('Focal length Y', r.camera_matrix[1][1].toFixed(2) + ' px'), metric('New recording error', r.validation_rms_px.toFixed(3) + ' px'));
    $('adjustmentNote').textContent = 'Use these intrinsics for this camera, resolution and lens setup. Changing focus, crop or the lens may require calibration again.';
  } else if (r.kind === 'camera_imu') {
    items.push(metric('Timing offset', (r.camera_to_imu_time_shift_s * 1000).toFixed(1) + ' ms'),
      metric('Spacing', (Math.hypot(...r.t_camera_from_imu_m) * 1000).toFixed(1) + ' mm'),
      metric('Gyro check', r.validation_gyro_rms_rad_s.toFixed(4) + ' rad/s'),
      metric('Acceleration check', r.validation_accel_rms_m_s2.toFixed(3) + ' m/s²'));
    $('adjustmentNote').textContent = r.openvins_chain ? 'The download includes an OpenVINS camera/IMU configuration. Review its frame and timing conventions before loading it into your estimator. Nothing is activated automatically.' : r.export_note;
  } else if (r.kind === 'optical_flow') {
    items.push(metric('X scale', r.axis_gain[0].toFixed(4)), metric('Y scale', r.axis_gain[1].toFixed(4)),
      metric('Validation error', (r.validation_error.relative_error * 100).toFixed(2) + '%'),
      metric('Axis mapping', r.axes_reflected ? 'Reversed axis' : r.rotation_deg.toFixed(2) + '°'));
    $('adjustmentNote').textContent = 'Use the full calibration matrix for scale and axis mapping. The reference frame is ' + r.reference_frame + '.';
  } else {
    for (const [camera, v] of Object.entries(r.cameras)) {
      items.push(metric(camera + ' · boresight', v.boresight_error_deg.toFixed(3) + '°'),
        metric(camera + ' · spacing', (v.baseline_m * 1000).toFixed(1) + ' mm'),
        metric(camera + ' · validation', v.validation_rms_px.toFixed(3) + ' px'),
        metric(camera + ' · adjust X / Y / Z', v.parallel_adjustment_xyz_deg ? v.parallel_adjustment_xyz_deg.map(x => x.toFixed(2) + '°').join(' / ') : 'See rotation matrix'));
    }
    $('adjustmentNote').textContent = 'Adjustments use the reference camera’s optical frame: X right, Y down, Z forward. R = Rz(z) Ry(y) Rx(x). Record again after moving a mount.';
  }
  $('metrics').replaceChildren(...items); $('details').textContent = JSON.stringify(r, null, 2);
  $('download').href = '/api/workspace/export/' + encodeURIComponent(r.session_id);
  $('download').download = 'orqa-' + r.session_id + '.zip';
}
async function history() {
  const data = await get('/api/workspace/history'); $('history').replaceChildren();
  if (!data.sessions.length) { $('history').append(node('p', 'Your checked recordings will appear here.', 'muted')); return; }
  for (const r of data.sessions) {
    const row = node('div', undefined, 'session'), text = node('div');
    const label = catalog.sensors.find(s => s.id === r.kind)?.name || r.kind;
    text.append(node('strong', label), node('small', r.created_utc + ' · ' + (r.setup_label || r.device_profile)),
      node('div', r.data_origin === 'synthetic' ? 'Example only · synthetic measurements' : 'Passed imported recording checks', 'status'));
    const link = node('a', 'Download', 'button secondary');
    link.href = '/api/workspace/export/' + encodeURIComponent(r.session_id); link.download = 'orqa-' + r.session_id + '.zip';
    row.append(text, link); $('history').append(row);
  }
}
$('back').onclick = () => { if (busy) return; kind = null; $('home').classList.remove('hidden'); $('workflow').classList.add('hidden'); };
$('recording').onchange = () => {
  if (busy) return;
  selectedFile = $('recording').files[0] || null; $('solve').disabled = !selectedFile;
  $('result').classList.add('hidden'); $('notice').textContent = '';
};
$('solve').onclick = async () => {
  if (busy || (!photoMode && !selectedFile)) return;
  busy = true; $('solve').disabled = true; $('back').disabled = true; $('recording').disabled = true;
  $('result').classList.add('hidden'); $('notice').classList.remove('error');
  for (const id of ['usePhotos', 'useRecording', 'trainingPhotos', 'validationPhotos']) $(id).disabled = true;
  $('notice').textContent = 'Checking setup, solving calibration and testing the separate recording…';
  try {
    if (!photoMode && selectedFile.size > 8 * 1024 * 1024 - 1024) throw Error('Recording is too large. Use a file below 8 MB.');
    const recording = photoMode ? await photoRecording() : JSON.parse(await selectedFile.text());
    if (recording.kind !== kind) throw Error('This recording belongs to a different sensor. Choose the matching workflow.');
    if (kind === 'camera_rig' && !$('correspondence').checked) throw Error('Confirm target corner order, rigidity and capture timing first.');
    const value = setup();
    const data = await post('calibrate', {recording, device_profile: value.hardware, setup: value});
    showResult(data.result);
    $('notice').textContent = data.result.data_origin === 'synthetic' ? 'Example complete. Import your own recordings to calibrate a sensor.' : 'Saved. The calibration passed the separate recording checks.';
    try { await history(); } catch (_) { $('connection').textContent = 'Result saved. Refresh to reload the session list.'; }
  } catch (e) {
    $('notice').classList.add('error');
    $('notice').textContent = e instanceof SyntaxError ? 'This file could not be read. Choose a Prismo JSON recording.' : e.message;
  } finally {
    busy = false; $('solve').disabled = photoMode ? false : !selectedFile; $('back').disabled = false; $('recording').disabled = false;
    for (const id of ['usePhotos', 'useRecording', 'trainingPhotos', 'validationPhotos']) $(id).disabled = false;
  }
};
async function init() {
  try {
    catalog = await get('/api/workspace/catalog');
    try {
      const saved = JSON.parse(localStorage.getItem('orqaCalibrationSetup'));
      if (saved && catalog.profiles.some(p => p.id === saved.hardware) && catalog.integrations.some(p => p.id === saved.integration) && catalog.firmwares.some(p => p.id === saved.firmware)) {
        $('profile').value = saved.hardware; $('integration').value = saved.integration;
        $('firmware').value = saved.firmware; $('firmwareVersion').value = saved.firmware_version || '';
      }
    } catch (_) {}
    setupChanged(); await history();
  } catch (_) {
    $('connection').textContent = 'Calibration service unavailable. Start Sensor Cal and refresh this page.';
    $('history').textContent = 'Saved sessions could not be loaded.';
  }
}
init();
