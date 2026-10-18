# Manual camera alignment and lever-arm measurements

Open **Manual camera alignment & lever-arm bench** from either app's home screen,
or visit `/alignment`. The same portable component is packaged independently in
Sensor Calibration Lab and the Orqa edition. Neither app imports the other's
mutable checkout at runtime. Four static files and a hash manifest define the
shared boundary; the Lab repository's `scripts/sync_alignment_component.py` performs an explicit
bounded copy when the component is deliberately updated.

## Visual mount adjustment

1. Use a fixed bench setup, remove propellers and keep aircraft disarmed. Fix
   focus, zoom, image format and camera orientation externally.
2. Connect 2–8 cameras visible to the browser, or load JPEG/PNG stills from the
   actual sensors. Browser cameras and evidence hashes require localhost or
   HTTPS. Simultaneous camera access depends on browser/driver/USB support;
   loading still images is also available. Network/MJPEG/ROS camera adapters
   are not added by this workbench.
3. Select a reference camera and a distant, stationary feature shared by the
   views. Use the grid/crosshairs and a long horizontal/vertical feature while
   adjusting physical pointing and roll. Display zoom affects all views;
   images retain their aspect ratio and are not warped, mirrored or rotated.
4. Click the common target to record a native-pixel offset. This is a manual
   click, not a tracked feature: click again after adjusting a mount. The default
   crosshair is an unverified image center. Optionally link that camera's
   matching neutral version 1 lens result to use its principal point; identity,
   image geometry, source hashes and synthetic provenance remain explicit.
5. Compare the reference and another view with transparent overlay or split/wipe.
   Cyan/pink crosshairs show their respective aiming references. Different
   focal lengths, distortion, spacing and parallax are not rectified or converted
   into physical angles. Try a second target distance before drawing conclusions.
   **Focus camera views** hides setup fields while retaining camera labels,
   reference selection, the image guides and stop controls.
6. Tighten mounts and recheck. Freeze the final views and record notes. Freezing
   holds the display while cameras remain connected; **Stop all cameras** releases
   tracks and retains an available final preview. Backgrounding/leaving the page
   also stops tracks, including cancellation of pending camera requests.

Disconnects or observable setting changes stop the affected stream. Reconnect
explicitly and check configuration. Browser camera IDs are privacy-scoped
identifiers, not hardware serials. The camera acquisition APIs supply no verified
hardware-exposure synchronization in this implementation. Snapshot times are
browser observations, never physical exposure timestamps. Imported-file
orientation must decode to its encoded geometry; the workbench refuses a changed
orientation rather than silently relabelling coordinates.

## Manual lever arms

A lever arm describes a sensor's fixed position relative to a reference point;
it is separate from rotation and from a flow/range sensor's distance to a target.
The existing numerical camera/IMU workflow estimates spacing from motion. This
worksheet records direct operator measurements without replacing that fit.

Choose one physical datum and a clearly named right-handed measurement frame.
Defaults describe rig-forward/right/down; they do not establish an aircraft or
camera transform. Label +X/+Y/+Z on the actual rig. Enter each sensor's X/Y/Z
position in **that same frame**, using mm, cm or metres. Unit changes convert
entered values. Changing axis descriptions redefines coordinates and requires
checking the measurements again; no unknown frame rotation is inferred.

Describe the actual point used: camera projection-center estimate, IMU chip or
triad estimate, flow sensor, GNSS antenna point, or explicitly estimated case/board
point. Record the tool/method, mount configuration and uncertainty estimate.
Unknown uncertainty stays unknown; a board or case measurement is not silently
promoted to an exact sensing-center measurement. Up to 16 sensor rows are
supported, including the camera views and added IMU/flow/GNSS/other sensors.

Select the lever-arm reference sensor and measure it as well. The review table
reports `p_sensor − p_reference`, in metres and in the declared measurement frame,
plus spacing. It preserves signed components. A supplied uncertainty is an
operator-declared ± bound per coordinate; pairwise bounds are summed
conservatively without assuming statistical independence. Unknown bounds remain
null. These are neither fitted standard deviations nor verified accuracy bounds.

Positions can be recorded without camera images. Leave all three coordinates
empty for an unmeasured sensor; partial vectors, invalid units, duplicate sensor
IDs and unconfirmed frame handedness are rejected. No camera/IMU rotation or
firmware parameter convention is inferred from the measurements.

## Worksheet and evidence

Confirm context and download `manual-alignment-worksheet.json`. It uses the
separate `sensor-alignment-observation` schema, version 1. It is not a neutral
`sensor-calibration` result and cannot create a numerical calibration pass.
The file contains source identity, native geometry, display settings, target
marks, browser observation times, bounded PNG previews and their SHA-256 hashes,
linked lens documents/hashes, input positions/units/frame/method, normalized
offsets, declared bounds and operator notes. It can be reopened in either app.

Imported images are bounded to 16 MiB, 24 MP and 8192 px per side before decode.
Preview PNGs are display images bounded to 1280 px per side, with processing
recorded. Imported originals remain unchanged and are identified by filename,
size and hash when available; they are not embedded in the worksheet. Worksheet
files are bounded to 32 MiB. Reopening checks PNG and linked-document hashes,
geometry and measurement structure, and requires context confirmation again.
Frozen observation times remain retained instead of becoming fresh acquisitions.

Worksheets live in the user's downloaded files and do not enter numerical
calibration history. They retain manual/unverified claims; physical, numerical,
bench, timing, flight and settings-application qualification remain false.
Synthetic evidence, including a linked synthetic lens result, stays synthetic.
Hashes establish byte identity, not authenticity. For calibrated rig transforms,
collect fresh numerical calibration recordings after final mount adjustment.

## References and limits

The [W3C Media Capture and Streams specification](https://www.w3.org/TR/mediacapture-streams/)
defines multiple-stream requests, source selection, actual settings and track
lifetime. This component records browser observations and makes no independently
verified synchronized-exposure claim. The [HTML canvas specification](https://html.spec.whatwg.org/multipage/canvas.html)
defines source/destination drawing geometry, pixel access and bitmap export.
Aspect-preserving fit and native-pixel inverse mapping are checked separately.

[ArduPilot's sensor-position offset guidance](https://ardupilot.org/copter/docs/common-sensor-offset-compensation.html)
illustrates why a firmware's origin and signs must be explicit: its reference
can depend on configuration. This worksheet therefore emits generic measured
vectors only. It supplies no ArduPilot/PX4/MSP parameter mapping and applies no
settings. Browser tests use generated images and synthetic MediaStreams; physical
multi-camera/mount accuracy remains unverified.
