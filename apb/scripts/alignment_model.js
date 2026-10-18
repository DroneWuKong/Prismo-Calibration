/* Manual observations only. No solver, transport, firmware or settings application. */
(function (root) {
  'use strict';
  const units = Object.freeze({mm: 0.001, cm: 0.01, m: 1});
  function number(value, label, limit = 1000000) {
    if (typeof value !== 'number' || !Number.isFinite(value) || Math.abs(value) > limit) throw new Error(`Invalid ${label}`);
    return value;
  }
  function text(value, label, limit = 2000) {
    if (typeof value !== 'string' || !value.trim() || value.length > limit) throw new Error(`Provide ${label}`);
    return value.trim();
  }
  function fitRect(width, height, targetWidth, targetHeight, zoom = 1) {
    for (const value of [width,height,targetWidth,targetHeight,zoom]) if (!Number.isFinite(value) || value <= 0) throw new Error('Invalid display geometry');
    const scale = Math.min(targetWidth / width, targetHeight / height) * zoom;
    return {x: (targetWidth-width*scale)/2, y: (targetHeight-height*scale)/2, width: width*scale, height: height*scale, scale};
  }
  function imageSize(bytes) {
    const b=bytes instanceof Uint8Array?bytes:new Uint8Array(bytes), view=new DataView(b.buffer,b.byteOffset,b.byteLength);
    let size=null;
    if(b.length>=24 && b.slice(0,8).every((v,i)=>v===[137,80,78,71,13,10,26,10][i]) && String.fromCharCode(...b.slice(12,16))==='IHDR')size=[view.getUint32(16),view.getUint32(20)];
    else if(b[0]===255 && b[1]===216){
      let i=2;
      while(i+4<=b.length){
        if(b[i++]!==255)break;while(b[i]===255)i++;const marker=b[i++];
        if(marker===217||marker===218)break;
        if(marker===1||(marker>=208&&marker<=215))continue;
        if(i+2>b.length)break;const length=view.getUint16(i);
        if(length<2||i+length>b.length)break;
        if([192,193,194].includes(marker)&&length>=8){size=[view.getUint16(i+5),view.getUint16(i+3)];break;}i+=length;
      }
    }
    if(!size || size.some(v=>v<16||v>8192)||size[0]*size[1]>24000000)throw new Error('Use standard JPEG/PNG up to 24 MP and 8192 px per side');
    return size;
  }
  function imagePoint(rect, x, y, width, height) {
    const point = [(x-rect.x)/rect.scale, (y-rect.y)/rect.scale];
    return point.every((v,i)=>v >= 0 && v < [width,height][i]) ? point : null;
  }
  function lensDocument(doc) {
    if (!doc || doc.schema !== 'sensor-calibration' || doc.version !== 1 || doc.task !== 'camera_intrinsics' || doc.validation?.status !== 'passed') throw new Error('Choose a neutral version 1 lens result with a passed fresh-view check');
    const c = doc.calibration, k = c?.camera_matrix, size = c?.image_size;
    if (!Array.isArray(k) || k.length !== 3 || !k.every(row=>Array.isArray(row) && row.length===3 && row.every(v=>typeof v==='number' && Number.isFinite(v))) || k[0][0]<=0 || k[1][1]<=0 || k[2][0]!==0 || k[2][1]!==0 || k[2][2]!==1) throw new Error('Invalid lens matrix');
    if (!Array.isArray(size) || size.length!==2 || !size.every(v=>Number.isInteger(v) && v>=16 && v<=8192) || size[0]*size[1]>24000000 || k[0][2]<0 || k[0][2]>=size[0] || k[1][2]<0 || k[1][2]>=size[1]) throw new Error('Invalid calibrated image geometry');
    if (doc.coordinates?.convention!=='x_right_y_down_z_forward' || doc.units?.image!=='px' || doc.units?.target!=='mm' || doc.units?.time!=='s') throw new Error('Lens result must declare pixels and the camera optical frame');
    if (!['synthetic','recording','unknown'].includes(doc.data_origin) || !doc.claims || !/^[0-9a-f]{64}$/.test(doc.provenance?.source_sha256||'')) throw new Error('Keep the lens origin, claims and source hash');
    for (const key of ['id','serial']) text(doc.sensor?.[key], 'lens sensor '+key);
    text(doc.coordinates.frame_id,'lens frame ID');
    return {size: size.slice(), axis: [k[0][2],k[1][2]], label: 'Linked lens principal point; provenance supplied'};
  }
  function leverArms(rows, unit, frame, reference) {
    if (!Object.hasOwn(units,unit)) throw new Error('Use mm, cm or m');
    text(frame.id,'measurement frame ID');text(frame.origin,'physical origin / datum');text(frame.method,'measurement method');
    for (const axis of ['x','y','z']) text(frame.axes?.[axis], 'positive '+axis.toUpperCase()+' direction');
    if (frame.right_handed_confirmed!==true) throw new Error('Confirm a right-handed measurement frame');
    if (!Array.isArray(rows) || rows.length>16) throw new Error('Use at most 16 sensor positions');
    const ids = new Set();
    const positions = rows.map(row=>{
      const id=text(row.sensor_id,'sensor ID',100);if(ids.has(id))throw new Error('Sensor IDs must be distinct');ids.add(id);
      text(row.serial,'sensor serial');
      const p=row.position;
      if (p===null) return {...row,position_m:null,uncertainty_bound_per_axis_m:null};
      if (!Array.isArray(p) || p.length!==3) throw new Error('Provide all three coordinates or leave all three empty');
      text(row.point_definition,'physical sensing point for '+id);
      const position=p.map(v=>number(v,'position')*units[unit]);
      if(position.some(v=>Math.abs(v)>100))throw new Error('Positions must be within 100 m of the declared origin');
      const u=row.uncertainty===null ? null : number(row.uncertainty,'declared uncertainty bound')*units[unit];
      if(u!==null && u<0)throw new Error('Uncertainty bounds cannot be negative');
      return {...row,position_m:position,uncertainty_bound_per_axis_m:u};
    });
    const measured=positions.filter(row=>row.position_m!==null), origin=measured.find(row=>row.key===reference);
    if (!measured.length) return {positions,reference_sensor_id:null,offsets:[],status:'not_measured'};
    if (!origin) throw new Error('Measure the selected lever-arm reference sensor as well');
    const offsets=measured.filter(row=>row!==origin).map(row=>{
      const vector=row.position_m.map((v,i)=>v-origin.position_m[i]);
      return {sensor_id:row.sensor_id,reference_sensor_id:origin.sensor_id,
        vector_reference_to_sensor_in_measurement_frame_m:vector,spacing_m:Math.hypot(...vector),
        declared_bound_per_axis_m:row.uncertainty_bound_per_axis_m===null || origin.uncertainty_bound_per_axis_m===null ? null : row.uncertainty_bound_per_axis_m+origin.uncertainty_bound_per_axis_m};
    });
    return {positions,reference_sensor_id:origin.sensor_id,offsets,status:'operator_measured',
      convention:'p_sensor - p_reference, expressed in the one declared measurement frame; not a calibrated camera/IMU transform',
      uncertainty_scope:'Operator-declared coordinate bounds; pair bounds are summed conservatively without an independence assumption. Unknown bounds remain null.'};
  }
  function origin(declared, sources, lenses) {
    const origins=[declared,...sources.filter(v=>v!=='operator_declared'),...lenses.map(l=>l.claims?.synthetic_check_passed===true?'synthetic':l.data_origin)];
    return origins.includes('synthetic') ? 'synthetic' : origins.every(v=>v==='recording') ? 'recording' : 'unknown';
  }
  root.AlignmentModel=Object.freeze({version:1,units,number,text,fitRect,imagePoint,imageSize,lensDocument,leverArms,origin});
})(globalThis);
