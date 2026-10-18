/* Portable local workbench. All results are manual observations, never solver results. */
(function () {
  'use strict';
  const M=AlignmentModel,$=id=>document.getElementById(id),views=[],rows=[];
  let nextId=0,devices=[],reference=null,revision=0,pageGeneration=0,lastDraw=0,unit='mm',saving=false;
  const copy=value=>JSON.parse(JSON.stringify(value));
  function error(message=''){$('alignment-error').textContent=message;$('alignment-error').hidden=!message;}
  function changed(){revision++;$('manual-confirm').checked=false;}
  function status(message){$('alignment-status').textContent=message;}
  function button(label,action){const b=document.createElement('button');b.type='button';b.textContent=label;b.addEventListener('click',()=>Promise.resolve().then(action).catch(e=>error(e.message)));return b;}
  function input(value='',placeholder=''){const e=document.createElement('input');e.value=value;e.placeholder=placeholder;return e;}
  function field(label,element){const e=document.createElement('label');e.append(document.createTextNode(label),element);return e;}
  function selected(select,options,value){select.replaceChildren();for(const [key,label]of options){const o=document.createElement('option');o.value=key;o.textContent=label;select.append(o);}if(options.some(([key])=>key===value))select.value=value;}
  function identity(v){return {id:v.sensor.value.trim(),serial:v.serial.value.trim(),frame_id:v.frame.value.trim()};}
  function sourceSize(v){return v.nativeSize||null;}
  function sourceImage(v){return v.frozen||v.image||(v.track?.readyState==='live'&&v.video.readyState>=2?v.video:null);}
  function axis(v){return v.lens?M.lensDocument(v.lens.document).axis:[v.nativeSize[0]/2,v.nativeSize[1]/2];}
  function stable(s){return JSON.stringify(Object.fromEntries(['deviceId','width','height','resizeMode','focusMode','focusDistance','zoom'].filter(k=>s[k]!==undefined).map(k=>[k,s[k]])));}
  async function hash(bytes){if(!crypto.subtle)throw new Error('Open this workbench at localhost or HTTPS to export evidence hashes');return Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');}
  function pngBytes(dataUrl){return Uint8Array.from(atob(dataUrl.split(',',2)[1]),c=>c.charCodeAt(0));}
  function rawSnapshot(v,maximum=1280){
    const source=sourceImage(v);if(!source||!v.nativeSize)return null;
    const size=v.nativeSize,scale=Math.min(1,maximum/Math.max(...size)),canvas=document.createElement('canvas');
    canvas.width=Math.max(1,Math.round(size[0]*scale));canvas.height=Math.max(1,Math.round(size[1]*scale));canvas.getContext('2d').drawImage(source,0,0,canvas.width,canvas.height);
    const metadata=v.frozen&&v.frameMetadata?copy(v.frameMetadata):{browser_monotonic_ms:performance.now(),browser_wall_time:new Date().toISOString(),hardware_exposure_time:null};
    return {canvas,metadata:{...metadata,native_size_px:size.slice(),preview_size_px:[canvas.width,canvas.height],processing:'Aspect-preserving bounded display preview; no rectification, rotation or digital alignment'}};
  }
  function stopView(v,retain=true){
    v.generation++;
    if(retain&&!v.frozen){const frame=rawSnapshot(v)||v.lastRaw;if(frame){v.frozen=frame.canvas;v.frameMetadata=frame.metadata;}}
    if(v.stream)v.stream.getTracks().forEach(t=>t.stop());v.stream=null;v.track=null;v.video.srcObject=null;
    v.pendingDevice=null;v.pendingGeneration=null;
  }
  function clearSource(v){stopView(v,false);v.image?.close?.();v.image=null;v.frozen=null;v.lastRaw=null;v.nativeSize=null;v.source=null;v.lens=null;v.point=null;v.frameMetadata=null;v.lensInput.value='';}
  function stopAll(){pageGeneration++;for(const v of views){stopView(v);v.message.textContent=v.frozen?'Stopped · retained display frame':'Camera stopped';}changed();}
  function freezeAll(invalidate=true){for(const v of views){const frame=rawSnapshot(v);if(frame){v.frozen=frame.canvas;v.frameMetadata=frame.metadata;v.message.textContent=v.track?'Frozen · camera remains connected; Stop all cameras releases devices':'Still image / frozen observation';}}if(invalidate)changed();}
  function refreshChoices(){
    if(!views.some(v=>v.key===reference))reference=views[0]?.key;
    selected($('compare-view'),views.filter(v=>v.key!==reference).map(v=>[v.key,identity(v).id]),$('compare-view').value);
    selected($('lever-reference'),rows.map(r=>[r.key,r.sensor.value||'Unnamed sensor']),$('lever-reference').value);
    for(const v of views){v.radio.checked=v.key===reference;v.card.classList.toggle('is-reference',v.key===reference);}
    $('add-view').disabled=views.length>=8||rows.length>=16;$('add-sensor').disabled=rows.length>=16;
  }
  function refreshDevices(){for(const v of views){const previous=v.device.value;const options=[['','Browser default'],...devices.map((d,i)=>[d.deviceId,d.label||`Camera ${i+1}`])];if(previous&&!options.some(([id])=>id===previous))options.push([previous,'Previously selected camera (unavailable)']);selected(v.device,options,previous);}}
  async function listCameras(){
    if(!navigator.mediaDevices?.getUserMedia)throw new Error('Browser cameras need localhost or HTTPS. Local image views are also supported.');
    const generation=pageGeneration;let probe=null;
    try{if(!views.some(v=>v.track?.readyState==='live'))probe=await navigator.mediaDevices.getUserMedia({audio:false,video:true});
      if(generation!==pageGeneration)return;
      devices=(await navigator.mediaDevices.enumerateDevices()).filter(d=>d.kind==='videoinput');
      if(generation===pageGeneration){refreshDevices();status(`${devices.length} browser camera(s) listed. Connect each desired camera on its card; audio is disabled.`);}
    }finally{probe?.getTracks().forEach(t=>t.stop());}
  }
  async function connect(v){
    error('');if(!navigator.mediaDevices?.getUserMedia)throw new Error('Use localhost or HTTPS for browser cameras; load images for an offline comparison.');
    const requested=v.device.value;
    if(requested&&views.some(other=>other!==v&&((other.track?.readyState==='live'&&other.settings.deviceId===requested)||other.pendingDevice===requested)))throw new Error('That camera is already selected in another view');
    clearSource(v);changed();const generation=v.generation,page=pageGeneration;v.pendingDevice=requested||'default';v.pendingGeneration=generation;v.message.textContent='Waiting for camera permission…';
    try{
      const constraints={width:{ideal:1280},height:{ideal:720}};if(requested)constraints.deviceId={exact:requested};
      if(navigator.mediaDevices.getSupportedConstraints().resizeMode)constraints.resizeMode={exact:'none'};
      const stream=await navigator.mediaDevices.getUserMedia({audio:false,video:constraints});
      if(generation!==v.generation||page!==pageGeneration){stream.getTracks().forEach(t=>t.stop());return;}
      v.stream=stream;v.track=stream.getVideoTracks()[0];v.settings=v.track.getSettings();
      if(views.some(other=>other!==v&&other.track?.readyState==='live'&&other.settings.deviceId&&other.settings.deviceId===v.settings.deviceId))throw new Error('The browser returned a camera already used by another view');
      const size=[v.settings.width,v.settings.height];if(size.some(n=>!Number.isInteger(n)||n<16||n>8192)||size[0]*size[1]>24000000)throw new Error('Camera mode exceeds 24 MP / 8192 px');
      v.nativeSize=size;v.source={kind:'browser_camera',data_origin:'operator_declared',device_label:v.track.label,actual_settings:copy(v.settings),identity_scope:'Browser permission identifiers are not hardware serials',synchronized:false};
      v.track.addEventListener('ended',()=>{if(v.generation!==generation)return;stopView(v);v.message.textContent='Disconnected · retained frame if available; reconnect explicitly';changed();},{once:true});
      v.video.srcObject=stream;await v.video.play();if(generation!==v.generation||page!==pageGeneration)return;
      v.message.textContent=`Live · ${size.join(' × ')} px · fix focus/zoom externally · no audio`;
      devices=(await navigator.mediaDevices.enumerateDevices()).filter(d=>d.kind==='videoinput');if(generation===v.generation)refreshDevices();
    }catch(exc){if(generation===v.generation){stopView(v);v.message.textContent='Camera unavailable';throw new Error(`${exc.message}. For multiple cameras, check permissions, driver support and USB bandwidth; local images remain available.`);}}
    finally{if(v.pendingGeneration===generation){v.pendingDevice=null;v.pendingGeneration=null;}}
  }
  async function loadImage(v,file){
    if(!file)return;if(file.size>16*1024*1024)throw new Error('Use a JPEG/PNG at most 16 MiB');
    stopView(v);v.message.textContent='Loading local image; previous camera stopped';const generation=v.generation,bytes=await file.arrayBuffer(),size=M.imageSize(bytes);
    const image=await createImageBitmap(new Blob([bytes]),{imageOrientation:'none'});
    if(image.width!==size[0]||image.height!==size[1]){image.close();throw new Error('Browser-decoded orientation differs from the encoded geometry; use an unrotated source image');}
    if(generation!==v.generation){image.close();return;}
    const sha=crypto.subtle?await hash(bytes):null;
    if(generation!==v.generation){image.close();return;}
    clearSource(v);v.image=image;v.nativeSize=size;v.source={kind:'local_image',name:file.name,size_bytes:file.size,sha256:sha,data_origin:'operator_declared',synchronized:false};
    v.message.textContent=`Local image · ${size.join(' × ')} px · original file unchanged`;changed();
  }
  async function loadLens(v,file){
    if(!file)return;if(file.size>8*1024*1024)throw new Error('Lens JSON exceeds 8 MiB');
    if(!v.nativeSize)throw new Error('Load or connect this view before linking its lens result');
    const generation=v.generation,raw=await file.text(),doc=JSON.parse(raw),checked=M.lensDocument(doc);
    if(checked.size.some((n,i)=>n!==v.nativeSize[i]))throw new Error('Linked lens resolution must match the actual source exactly');
    const sha=await hash(new TextEncoder().encode(raw)),parsedHash=await hash(new TextEncoder().encode(JSON.stringify(doc)));if(generation!==v.generation)return;
    v.lens={document:doc,original_document_sha256:sha,parsed_document_sha256:parsedHash,parsed_hash_encoding:'UTF-8 JSON.stringify(document)'};v.sensor.value=doc.sensor.id;v.serial.value=doc.sensor.serial;v.frame.value=doc.coordinates.frame_id;
    syncIdentity(v);v.message.textContent='Matching lens linked · crosshair uses its principal point';changed();
  }
  function syncIdentity(v){v.title.textContent=v.sensor.value;const row=rows.find(r=>r.key===v.key);if(row){row.sensor.value=v.sensor.value;row.serial.value=v.serial.value;}refreshChoices();}
  function addRow(key,view=null){
    if(rows.length>=16)throw new Error('Use at most 16 sensor positions');
    const row={key,view},tr=document.createElement('tr');tr.dataset.key=key;
    row.sensor=input(view?view.sensor.value:`sensor-${rows.length+1}`);row.serial=input(view?view.serial.value:'unknown');
    row.type=document.createElement('select');selected(row.type,['camera','imu','flow','gnss','other'].map(v=>[v,v.toUpperCase()]),view?'camera':'imu');
    row.point=input('',view?'Estimated camera sensing center':'Describe the sensing point');
    row.position=[input(),input(),input()];row.uncertainty=input('','Unknown');
    for(const e of [...row.position,row.uncertainty]){e.type='number';e.step='any';e.setAttribute('aria-label',e===row.uncertainty?'Estimated bound per axis':`${['X','Y','Z'][row.position.indexOf(e)]} position`);}
    row.uncertainty.min='0';row.sensor.readOnly=!!view;row.serial.readOnly=!!view;row.type.disabled=!!view;
    const first=document.createElement('td');first.append(row.sensor,row.type,row.serial);tr.append(first);
    for(const e of [row.point,...row.position,row.uncertainty]){const td=document.createElement('td');td.append(e);tr.append(td);}
    const last=document.createElement('td');if(!view)last.append(button('Remove',()=>{rows.splice(rows.indexOf(row),1);tr.remove();refreshChoices();changed();}));tr.append(last);
    tr.addEventListener('input',()=>{changed();refreshChoices();});row.tr=tr;rows.push(row);$('lever-rows').append(tr);refreshChoices();return row;
  }
  function addView(){
    if(views.length>=8||rows.length>=16)throw new Error('Use at most eight views and sixteen total sensors');
    const key='view-'+(++nextId),v={key,generation:0,settings:{},point:null};
    v.card=document.createElement('article');v.card.className='view-card';v.card.dataset.key=key;
    const head=document.createElement('div');head.className='view-head';const title=document.createElement('h3');v.title=title;title.textContent=`View ${views.length+1}`;
    v.radio=input();v.radio.type='radio';v.radio.name='reference-camera';const referenceLabel=field('Reference ',v.radio);
    v.radio.addEventListener('change',()=>{reference=key;refreshChoices();changed();});head.append(title,referenceLabel);
    const fields=document.createElement('div');fields.className='view-fields';v.sensor=input(`camera-${nextId}`);v.serial=input('unknown');v.frame=input(`camera_${nextId}_optical`);
    fields.append(field('Sensor ID',v.sensor),field('Actual serial / unknown',v.serial),field('Optical frame ID',v.frame));
    for(const e of [v.sensor,v.serial,v.frame])e.addEventListener('input',()=>{v.lens=null;v.lensInput.value='';syncIdentity(v);changed();});
    const source=document.createElement('div');source.className='view-source';v.device=document.createElement('select');v.device.setAttribute('aria-label','Camera device');
    v.device.addEventListener('change',()=>{stopView(v);v.message.textContent='Selection changed; reconnect explicitly';changed();});
    const imageInput=input();imageInput.type='file';imageInput.accept='image/jpeg,image/png';imageInput.addEventListener('change',()=>loadImage(v,imageInput.files[0]).catch(e=>error(e.message)));
    source.append(v.device,button('Connect',()=>connect(v)),button('Stop',()=>{stopView(v);v.message.textContent='Stopped · retained frame if available';changed();}),field('Load image',imageInput));
    v.canvas=document.createElement('canvas');v.canvas.width=640;v.canvas.height=360;v.canvas.setAttribute('aria-label','Camera view; click the common target');v.video=document.createElement('video');v.video.muted=true;v.video.playsInline=true;
    v.canvas.addEventListener('click',event=>{if(!v.nativeSize||!sourceImage(v))return;const box=v.canvas.getBoundingClientRect(),rect=M.fitRect(...v.nativeSize,640,360,Number($('view-zoom').value));
      const point=M.imagePoint(rect,(event.clientX-box.left)*640/box.width,(event.clientY-box.top)*360/box.height,...v.nativeSize);
      if(point){v.point={native_px:point,browser_wall_time:new Date().toISOString(),tracking:'manual click; not tracked; reclick after mount adjustment'};changed();}});
    v.message=document.createElement('p');v.message.className='view-status';v.message.textContent='No image loaded';v.offset=document.createElement('p');v.offset.className='view-offset';
    v.lensInput=input();v.lensInput.type='file';v.lensInput.accept='.json,application/json';v.lensInput.addEventListener('change',()=>loadLens(v,v.lensInput.files[0]).catch(e=>error(e.message)));
    const details=document.createElement('details'),summary=document.createElement('summary');summary.textContent='Optional lens result / actual source details';v.info=document.createElement('pre');details.append(summary,field('Link matching neutral lens JSON',v.lensInput),v.info);
    const actions=document.createElement('div');actions.className='toolbar';actions.append(button('Clear target mark',()=>{v.point=null;changed();}),button('Remove view',()=>{
      if(views.length<=2)throw new Error('Keep at least two view slots');clearSource(v);views.splice(views.indexOf(v),1);const row=rows.find(r=>r.key===key);if(row){rows.splice(rows.indexOf(row),1);row.tr.remove();}v.card.remove();refreshChoices();changed();}));
    v.card.append(head,fields,source,v.canvas,v.video,v.message,v.offset,details,actions);views.push(v);$('alignment-views').append(v.card);addRow(key,v);refreshChoices();refreshDevices();changed();return v;
  }
  function draw(v,canvas,guides=true,opacity=1){
    const ctx=canvas.getContext('2d'),source=sourceImage(v);if(!source||!v.nativeSize)return false;
    const rect=M.fitRect(...v.nativeSize,canvas.width,canvas.height,Number($('view-zoom').value));ctx.save();ctx.globalAlpha=opacity;ctx.drawImage(source,rect.x,rect.y,rect.width,rect.height);ctx.restore();
    if(guides){ctx.save();ctx.lineWidth=1;ctx.strokeStyle='#97c7ac88';
      if($('view-grid').checked)for(const fraction of [.25,.5,.75]){ctx.beginPath();ctx.moveTo(rect.x+rect.width*fraction,rect.y);ctx.lineTo(rect.x+rect.width*fraction,rect.y+rect.height);ctx.moveTo(rect.x,rect.y+rect.height*fraction);ctx.lineTo(rect.x+rect.width,rect.y+rect.height*fraction);ctx.stroke();}
      const aim=axis(v),x=rect.x+aim[0]*rect.scale,y=rect.y+aim[1]*rect.scale;
      ctx.strokeStyle=v.lens?'#e9be50':'#e4e9e2';ctx.beginPath();ctx.moveTo(x-20,y);ctx.lineTo(x+20,y);ctx.moveTo(x,y-20);ctx.lineTo(x,y+20);ctx.stroke();
      ctx.translate(x,y);ctx.rotate(Number($('roll-guide').value)*Math.PI/180);ctx.setLineDash([6,5]);ctx.beginPath();ctx.moveTo(-canvas.width,0);ctx.lineTo(canvas.width,0);ctx.stroke();ctx.restore();
      if(v.point){const [px,py]=v.point.native_px.map((n,i)=>(i===0?rect.x:rect.y)+n*rect.scale);ctx.save();ctx.strokeStyle='#ef8cbd';ctx.beginPath();ctx.arc(px,py,7,0,Math.PI*2);ctx.moveTo(x,y);ctx.lineTo(px,py);ctx.stroke();ctx.restore();}
    }return true;
  }
  function repaint(time){
    if(time-lastDraw>=100){lastDraw=time;
      for(const v of views){
        if(v.track?.readyState==='live'){
          const actual=v.track.getSettings();
          if(stable(actual)!==stable(v.settings)){stopView(v);v.lens=null;v.point=null;v.message.textContent='Observable camera settings changed · stopped; check configuration and reconnect';changed();}
          else if(!v.frozen && v.video.readyState>=2)v.lastRaw=rawSnapshot(v,640);
        }
        const ctx=v.canvas.getContext('2d');ctx.fillStyle='#132522';ctx.fillRect(0,0,640,360);
        if(!draw(v,v.canvas)){ctx.fillStyle='#aac2b2';ctx.font='16px system-ui';ctx.fillText('Connect a camera or load an image',25,180);}
        if(v.nativeSize){const a=axis(v),p=v.point?.native_px;v.offset.textContent=`${v.lens?'Linked principal point':'Unverified image center'} [${a.map(n=>n.toFixed(1)).join(', ')}] px`+(p?` · last click ΔX ${(p[0]-a[0]).toFixed(1)}, ΔY ${(p[1]-a[1]).toFixed(1)} px; not tracked`:' · click a common target');}
        else v.offset.textContent='';
        if(v.info.parentElement.open)v.info.textContent=JSON.stringify({source:v.source,lens_source_sha256:v.lens?.document.provenance.source_sha256||null},null,2);
      }
      const canvas=$('comparison-canvas'),ctx=canvas.getContext('2d');ctx.fillStyle='#132522';ctx.fillRect(0,0,960,540);
      const a=views.find(v=>v.key===reference),b=views.find(v=>v.key===$('compare-view').value);
      if(a&&b&&sourceImage(a)&&sourceImage(b)){
        draw(a,canvas,false);const amount=Number($('compare-amount').value)/100;
        if($('compare-mode').value==='wipe'){ctx.save();ctx.beginPath();ctx.rect(960*amount,0,960*(1-amount),540);ctx.clip();draw(b,canvas,false);ctx.restore();ctx.strokeStyle='#f5cc62';ctx.beginPath();ctx.moveTo(960*amount,0);ctx.lineTo(960*amount,540);ctx.stroke();}
        else draw(b,canvas,false,amount);
        for(const [v,color,length]of [[a,'#70e2d0',22],[b,'#f58fc3',12]]){const rect=M.fitRect(...v.nativeSize,960,540,Number($('view-zoom').value)),aim=axis(v),x=rect.x+aim[0]*rect.scale,y=rect.y+aim[1]*rect.scale;ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(x-length,y);ctx.lineTo(x+length,y);ctx.moveTo(x,y-length);ctx.lineTo(x,y+length);ctx.stroke();}
        $('comparison-caption').textContent=`Reference ${identity(a).id} (cyan) / ${identity(b).id} (pink) aiming guides. Full-frame aspect ratio preserved; different fields of view, no rectification or exposure synchronization.`;
        $('comparison-origin').textContent=M.origin($('alignment-origin').value,[a.source.data_origin,b.source.data_origin],[a.lens?.document,b.lens?.document].filter(Boolean))==='synthetic'?'SYNTHETIC EXAMPLE':'Unverified visual comparison';
      }else $('comparison-caption').textContent='Load a reference and a second view. Views remain unwarped; no field-of-view or distortion correction is inferred.';
    }requestAnimationFrame(repaint);
  }
  function frameInput(){return {id:$('lever-frame').value,origin:$('lever-origin').value,axes:{x:$('lever-x-axis').value,y:$('lever-y-axis').value,z:$('lever-z-axis').value},method:$('lever-method').value,right_handed_confirmed:$('lever-handed').checked};}
  function rowInputs(){return rows.map(r=>{const raw=r.position.map(e=>e.value),empty=raw.every(v=>v==='');if(!empty&&raw.some(v=>v===''))throw new Error('Provide all three coordinates or leave all three empty');return {key:r.key,sensor_id:r.sensor.value,serial:r.serial.value,type:r.type.value,point_definition:r.point.value,position:empty?null:raw.map(Number),uncertainty:r.uncertainty.value===''?null:Number(r.uncertainty.value)};});}
  function reviewLevers(required=false){
    const inputRows=rowInputs(),any=inputRows.some(r=>r.position!==null);
    if(!any&&!required)return {status:'not_measured',positions:inputRows,offsets:[],reference_sensor_id:null};
    return M.leverArms(inputRows,unit,frameInput(),$('lever-reference').value);
  }
  function showLevers(){
    error('');const result=reviewLevers(true);$('lever-result').replaceChildren();const p=document.createElement('p');p.textContent=result.offsets.length?`From ${result.reference_sensor_id}, expressed in ${$('lever-frame').value}. Operator measurements; not solver-validated.`:'Measure at least the reference and one other sensor for an offset.';$('lever-result').append(p);
    if(result.offsets.length){const wrap=document.createElement('div');wrap.className='table-wrap';const table=document.createElement('table'),head=document.createElement('tr');for(const label of ['Sensor','ΔX (m)','ΔY (m)','ΔZ (m)','Spacing (m)','Declared ± per axis (m)']){const th=document.createElement('th');th.textContent=label;head.append(th);}table.append(head);
      for(const offset of result.offsets){const tr=document.createElement('tr');for(const value of [offset.sensor_id,...offset.vector_reference_to_sensor_in_measurement_frame_m,offset.spacing_m,offset.declared_bound_per_axis_m]){const td=document.createElement('td');td.textContent=value===null?'Unknown':typeof value==='number'?String(Number(value.toPrecision(7))):value;tr.append(td);}table.append(tr);}wrap.append(table);$('lever-result').append(wrap);}
    const details=document.createElement('details'),summary=document.createElement('summary'),pre=document.createElement('pre');summary.textContent='Inspect measurement definitions and normalized values';pre.textContent=JSON.stringify(result,null,2);details.append(summary,pre);$('lever-result').append(details);
  }
  function download(name,text,type='application/json'){const url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),30000);}
  async function save(){
    if(saving)return;error('');if(!$('manual-confirm').checked)throw new Error('Confirm the identities, acquisition and measurement context before export');
    const levers=reviewLevers(),ready=views.filter(v=>sourceImage(v));if(ready.length<2&&levers.offsets.length<1)throw new Error('Load two camera views or measure the reference and at least one other sensor');
    if(ready.length>=2&&!ready.some(v=>v.key===reference))throw new Error('Choose a loaded reference camera before exporting the visual observation');
    const ids=new Set(),frames=new Set();for(const v of views){const id=identity(v);M.text(id.id,'sensor ID');M.text(id.serial,'serial or unknown');M.text(id.frame_id,'optical frame ID');if(ids.has(id.id)||frames.has(id.frame_id))throw new Error('Camera IDs and optical frame IDs must be distinct');ids.add(id.id);frames.add(id.frame_id);}
    saving=true;try{
      freezeAll(false);const savedRevision=revision,observations=[];
      for(const v of ready){
        const png=v.frozen.toDataURL('image/png'),blob=pngBytes(png);
        observations.push({key:v.key,sensor:identity(v),source:copy(v.source),lens_calibration:v.lens?copy(v.lens):null,
          native_size_px:v.nativeSize.slice(),frame:copy(v.frameMetadata),preview:{data_url:png,sha256:await hash(blob),role:'bounded display preview; original file not embedded'},
          target_mark:v.point?copy(v.point):null,aim_reference:{native_px:axis(v),basis:v.lens?'linked_lens_principal_point':'unverified_image_center'}});
      }
      if(savedRevision!==revision)throw new Error('Workbench changed during export; freeze and review it again');
      const origin=M.origin($('alignment-origin').value,observations.map(v=>v.source.data_origin),observations.map(v=>v.lens_calibration?.document).filter(Boolean));
      const doc={schema:'sensor-alignment-observation',version:1,component_version:'1.0.0',task:'manual_boresight_and_lever_arms',data_origin:origin,
        recorded_at:new Date().toISOString(),reference_view:reference,views:observations,
        display:{zoom:Number($('view-zoom').value),grid:$('view-grid').checked,roll_guide_clockwise_deg:Number($('roll-guide').value),comparison_mode:$('compare-mode').value,comparison_amount:Number($('compare-amount').value),comparison_target_view:$('compare-view').value},
        lever_measurements:{input_unit:unit,frame:frameInput(),reference_key:$('lever-reference').value,rows:rowInputs(),normalized:levers},
        context:{target_distance_and_method:$('target-distance').value,mount_configuration:$('mount-note').value,notes:$('alignment-notes').value},
        claims:{operator_context_confirmed:true,manual_visual_observation:ready.length>=2,manual_lever_arm_observation:levers.offsets.length>0,numerically_validated:false,physical_verified:false,bench_validated:false,timing_calibrated:false,flight_qualified:false,settings_applied:false},
        limitations:['Independent sequential browser observations; no synchronized hardware exposures','Target marks are manual clicks, not tracked features','Image center without linked intrinsics is not a calibrated optical axis','Parallax and differing distortion/field of view are not corrected','Lever-arm vectors are in the declared measurement frame; no calibrated sensor rotation is inferred','Hashes establish bytes, not source authenticity or physical accuracy']};
      const raw=JSON.stringify(doc,null,2);if(new TextEncoder().encode(raw).length>32*1024*1024)throw new Error('Worksheet exceeds 32 MiB; use fewer views or smaller images');
      download('manual-alignment-worksheet.json',raw);status(`Manual worksheet downloaded · ${origin} · ${observations.length} retained views · ${levers.offsets.length} lever arm(s). No calibration pass assigned.`);
    }finally{saving=false;}
  }
  function clearBench(){for(const v of views){clearSource(v);v.card.remove();}for(const r of rows)r.tr.remove();views.length=0;rows.length=0;reference=null;refreshChoices();}
  function demo(){
    if(views.some(v=>v.source)&&!confirm('Replace the current camera views and measurements with a synthetic example?'))return;
    clearBench();unit='mm';$('lever-unit').value=unit;$('lever-frame').value='synthetic_rig_frame';$('lever-x-axis').value='Rig forward';$('lever-y-axis').value='Rig right';$('lever-z-axis').value='Rig down';for(let i=0;i<2;i++){
      const v=addView(),c=document.createElement('canvas');c.width=800;c.height=450;const ctx=c.getContext('2d');ctx.fillStyle='#c7dad4';ctx.fillRect(0,0,800,450);
      ctx.fillStyle='#3b6860';ctx.fillRect(0,260+i*12,800,190);ctx.strokeStyle='#edf5ed';ctx.lineWidth=5;ctx.beginPath();ctx.moveTo(0,240+i*12);ctx.lineTo(800,240+i*12);ctx.stroke();
      const x=400+i*32,y=225-i*20;ctx.fillStyle='#c65f3a';ctx.fillRect(x-18,y-18,36,36);ctx.strokeStyle='#fff9df';ctx.lineWidth=3;ctx.strokeRect(x-25,y-25,50,50);
      ctx.fillStyle='#1b4039';ctx.font='18px system-ui';ctx.fillText('SYNTHETIC SCENE · view '+(i+1),24,35);
      v.image=c;v.nativeSize=[800,450];v.source={kind:'synthetic_scene',data_origin:'synthetic',generator:'Manual bench deterministic scene v1',synchronized:false};v.message.textContent='SYNTHETIC EXAMPLE · no physical camera';
      const r=rows.find(r=>r.key===v.key);r.point.value='Synthetic camera center';r.position.forEach((e,j)=>e.value=j===0?String(i*60):'0');r.uncertainty.value='0';
    }
    $('alignment-origin').value='synthetic';$('lever-origin').value='Synthetic reference camera center';$('lever-method').value='Analytic fixture, no physical measurement';$('lever-handed').checked=true;changed();status('Synthetic example loaded. The two views have a deliberate visual offset and a 60 mm baseline.');
  }
  async function openWorksheet(file){
    if(!file)return;if(file.size>32*1024*1024)throw new Error('Worksheet exceeds 32 MiB');
    const doc=JSON.parse(await file.text());if(doc.schema!=='sensor-alignment-observation'||doc.version!==1||!Array.isArray(doc.views)||doc.views.length>8||!['synthetic','recording','unknown'].includes(doc.data_origin))throw new Error('Choose a version 1 manual alignment worksheet');
    const prepared=[],keys=new Set();
    try{
      for(const item of doc.views){
        if(keys.has(item.key)||typeof item.key!=='string')throw new Error('Worksheet view keys must be distinct');keys.add(item.key);
        for(const k of ['id','serial','frame_id'])M.text(item.sensor?.[k],'camera '+k);
        if(!Array.isArray(item.native_size_px)||item.native_size_px.length!==2||!item.native_size_px.every(n=>Number.isInteger(n)&&n>=16&&n<=8192)||item.native_size_px[0]*item.native_size_px[1]>24000000)throw new Error('Invalid native image geometry');
        if(typeof item.preview?.data_url!=='string'||item.preview.data_url.length>8*1024*1024||!/^data:image\/png;base64,[A-Za-z0-9+/=]+$/.test(item.preview.data_url))throw new Error('Worksheet needs bounded embedded PNG previews');
        if(!item.source||!['operator_declared','synthetic','recording','unknown'].includes(item.source.data_origin))throw new Error('Keep source provenance and origin');
        const bytes=pngBytes(item.preview.data_url);if(await hash(bytes)!==item.preview.sha256)throw new Error('Worksheet preview SHA-256 mismatch');
        const previewSize=M.imageSize(bytes);if(Math.max(...previewSize)>1280)throw new Error('Worksheet preview exceeds 1280 px');
        if(item.lens_calibration){const checked=M.lensDocument(item.lens_calibration.document);if(checked.size.some((n,i)=>n!==item.native_size_px[i]))throw new Error('Linked lens geometry changed');const lens=item.lens_calibration.document;if(lens.sensor.id!==item.sensor.id||lens.sensor.serial!==item.sensor.serial||lens.coordinates.frame_id!==item.sensor.frame_id)throw new Error('Linked lens identity changed');if(await hash(new TextEncoder().encode(JSON.stringify(lens)))!==item.lens_calibration.parsed_document_sha256)throw new Error('Linked lens document SHA-256 mismatch');}
        if(item.target_mark){if(!Array.isArray(item.target_mark.native_px)||item.target_mark.native_px.length!==2||!item.target_mark.native_px.every((n,i)=>typeof n==='number'&&Number.isFinite(n)&&n>=0&&n<item.native_size_px[i]))throw new Error('Invalid target mark');}
        const image=await createImageBitmap(new Blob([bytes],{type:'image/png'}));prepared.push({item,image});
      }
      const lever=doc.lever_measurements;if(!lever||!Array.isArray(lever.rows)||lever.rows.length>16||new Set(lever.rows.map(r=>r.key)).size!==lever.rows.length)throw new Error('Invalid lever-arm rows');
      if(!lever.frame||!lever.frame.axes||['id','origin','method'].some(k=>typeof lever.frame[k]!=='string')||['x','y','z'].some(k=>typeof lever.frame.axes[k]!=='string'))throw new Error('Keep the lever-arm frame declarations, even when positions are unmeasured');
      for(const row of lever.rows){if(!row||typeof row.key!=='string'||typeof row.sensor_id!=='string'||typeof row.serial!=='string'||typeof row.point_definition!=='string'||!(row.uncertainty===null||(typeof row.uncertainty==='number'&&Number.isFinite(row.uncertainty)&&row.uncertainty>=0)))throw new Error('Invalid sensor identity or uncertainty');}
      for(const {item}of prepared){const row=lever.rows.find(r=>r.key===item.key);if(!row||row.sensor_id!==item.sensor.id||row.serial!==item.sensor.serial)throw new Error('Each view needs its matching lever-arm identity row');}
      if(lever.rows.some(r=>r.position!==null))M.leverArms(lever.rows,lever.input_unit,lever.frame,lever.reference_key);
      if(!Object.hasOwn(M.units,lever.input_unit))throw new Error('Unknown input unit');
      const d=doc.display;if(!d||typeof d.zoom!=='number'||d.zoom<1||d.zoom>4||!Number.isFinite(d.roll_guide_clockwise_deg)||Math.abs(d.roll_guide_clockwise_deg)>45||!['overlay','wipe'].includes(d.comparison_mode)||!Number.isFinite(d.comparison_amount)||d.comparison_amount<0||d.comparison_amount>100)throw new Error('Invalid display settings');
      if(views.some(v=>v.source)&&!confirm('Open this worksheet? Current cameras will stop and this bench will be replaced.'))return;
      stopAll();clearBench();const mapping=new Map();
      for(const {item,image}of prepared){const v=addView();mapping.set(item.key,v.key);v.sensor.value=item.sensor.id;v.serial.value=item.sensor.serial;v.frame.value=item.sensor.frame_id;v.image=image;v.frozen=image;v.nativeSize=item.native_size_px.slice();v.source={...copy(item.source),data_origin:M.origin(doc.data_origin,[item.source.data_origin],[]),reopened_worksheet:true};v.lens=item.lens_calibration?copy(item.lens_calibration):null;v.point=item.target_mark?copy(item.target_mark):null;v.frameMetadata=item.frame;v.message.textContent='Reopened manual worksheet · retained preview; acquisition not authenticated';syncIdentity(v);}
      for(const saved of lever.rows){let row=rows.find(r=>r.key===mapping.get(saved.key));if(!row){if(saved.type==='camera'&&views.length<2){const v=addView();v.sensor.value=saved.sensor_id;v.serial.value=saved.serial;syncIdentity(v);row=rows.find(r=>r.key===v.key);}else{row=addRow('sensor-'+(++nextId));row.sensor.value=saved.sensor_id;row.serial.value=saved.serial;row.type.value=['camera','imu','flow','gnss','other'].includes(saved.type)?saved.type:'other';}mapping.set(saved.key,row.key);}row.point.value=saved.point_definition||'';row.position.forEach((e,i)=>e.value=saved.position===null?'':saved.position[i]);row.uncertainty.value=saved.uncertainty===null?'':saved.uncertainty;}
      while(views.length<2&&rows.length<16)addView();reference=mapping.get(doc.reference_view)||views[0]?.key||null;
      unit=lever.input_unit;$('lever-unit').value=unit;const f=lever.frame;for(const [id,value]of [['lever-frame',f.id],['lever-origin',f.origin],['lever-method',f.method],['lever-x-axis',f.axes?.x],['lever-y-axis',f.axes?.y],['lever-z-axis',f.axes?.z]])$(id).value=typeof value==='string'?value:'';
      $('lever-handed').checked=f.right_handed_confirmed===true;refreshChoices();$('lever-reference').value=mapping.get(lever.reference_key)||rows[0]?.key;
      $('view-zoom').value=d.zoom;$('view-grid').checked=d.grid===true;$('roll-guide').value=d.roll_guide_clockwise_deg;$('compare-mode').value=d.comparison_mode;$('compare-amount').value=d.comparison_amount;if(mapping.has(d.comparison_target_view))$('compare-view').value=mapping.get(d.comparison_target_view);$('zoom-value').textContent=d.zoom+'×';$('roll-value').textContent=d.roll_guide_clockwise_deg+'°';
      $('alignment-origin').value=doc.data_origin;for(const[id,key]of[['target-distance','target_distance_and_method'],['mount-note','mount_configuration'],['alignment-notes','notes']])$(id).value=doc.context?.[key]||'';
      changed();status('Manual worksheet reopened. Confirm context again before exporting; no validation pass is inherited.');prepared.length=0;
    }finally{for(const p of prepared)p.image.close();}
  }
  function action(id,fn){$(id).addEventListener('click',()=>Promise.resolve().then(fn).catch(e=>error(e.message)));}
  action('find-cameras',listCameras);action('add-view',addView);action('freeze-views',()=>freezeAll());action('stop-views',stopAll);action('alignment-demo',demo);
  action('focus-views',()=>{const focus=document.body.classList.toggle('view-focus');$('focus-views').textContent=focus?'Show setup controls':'Focus camera views';if(focus){document.querySelector('.guide').open=false;$('alignment-views').scrollIntoView({block:'start'});}});
  action('resume-views',()=>{for(const v of views)if(v.track?.readyState==='live'){v.frozen=null;v.point=null;v.message.textContent='Live · reclick the target after adjusting the mount';}changed();});
  action('add-sensor',()=>{addRow('sensor-'+(++nextId));changed();});action('calculate-levers',showLevers);action('save-worksheet',save);
  $('worksheet-file').addEventListener('change',()=>openWorksheet($('worksheet-file').files[0]).catch(e=>error(e.message)));
  $('lever-unit').addEventListener('change',()=>{const next=$('lever-unit').value,factor=M.units[unit]/M.units[next];for(const r of rows)for(const e of [...r.position,r.uncertainty])if(e.value!=='')e.value=String(Number((Number(e.value)*factor).toPrecision(12)));unit=next;changed();});
  for(const id of ['view-zoom','view-grid','roll-guide','compare-view','compare-mode','compare-amount','lever-frame','lever-origin','lever-x-axis','lever-y-axis','lever-z-axis','lever-method','lever-reference','lever-handed','alignment-origin','target-distance','mount-note','alignment-notes'])$(id).addEventListener('input',()=>{changed();$('zoom-value').textContent=$('view-zoom').value+'×';$('roll-value').textContent=$('roll-guide').value+'°';});
  window.addEventListener('pagehide',stopAll);window.addEventListener('beforeunload',stopAll);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll();});
  navigator.mediaDevices?.addEventListener?.('devicechange',()=>{navigator.mediaDevices.enumerateDevices().then(ds=>{devices=ds.filter(d=>d.kind==='videoinput');refreshDevices();}).catch(()=>status('Camera list changed; refresh devices when ready.'));});
  addView();addView();requestAnimationFrame(repaint);
})();
