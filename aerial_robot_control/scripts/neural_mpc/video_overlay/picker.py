"""Generate a standalone HTML point picker for one video frame.

cv2.imshow is unreliable over SSH / remote VS Code, and clicking tile corners
needs sub-pixel care, so the picker is a self-contained HTML file with the frame
embedded as a data URI: open it in any browser, zoom in with the wheel, click the
landmarks, type their world coordinates, and export the JSON that
`project_reference.py calibrate` consumes.
"""

from __future__ import annotations

import base64
import cv2

_HTML = r"""<!doctype html><html><head><meta charset="utf-8">
<title>Reference projection - point picker</title>
<style>
 :root{--bg:#14161a;--panel:#1d2027;--line:#333a45;--fg:#e8ecf2;--accent:#3cdcff;--warn:#ffb03c}
 *{box-sizing:border-box}
 body{margin:0;font:13px/1.45 ui-sans-serif,system-ui,sans-serif;background:var(--bg);color:var(--fg);
      display:grid;grid-template-columns:1fr 430px;height:100vh;overflow:hidden}
 #stage{position:relative;overflow:hidden;cursor:crosshair;background:#0b0d10}
 #cv{position:absolute;left:0;top:0;transform-origin:0 0;image-rendering:pixelated}
 #mag{position:absolute;right:14px;top:14px;width:220px;height:220px;border:1px solid var(--line);
      border-radius:8px;background:#000;pointer-events:none;image-rendering:pixelated}
 #hint{position:absolute;left:14px;bottom:14px;background:rgba(0,0,0,.66);padding:8px 12px;
       border-radius:8px;font-size:12px;color:#b9c2cf;max-width:60%}
 aside{background:var(--panel);border-left:1px solid var(--line);display:flex;flex-direction:column;
       overflow:hidden}
 aside h1{font-size:14px;margin:14px 14px 4px;letter-spacing:.3px}
 aside .sub{margin:0 14px 10px;color:#93a0b0;font-size:12px}
 .row{display:flex;gap:8px;align-items:center;padding:8px 14px;border-top:1px solid var(--line);
      flex-wrap:wrap}
 label{color:#93a0b0}
 input,select,button{background:#262b34;color:var(--fg);border:1px solid var(--line);
        border-radius:6px;padding:5px 8px;font:inherit}
 input[type=number]{width:82px}
 button{cursor:pointer}
 button.primary{background:var(--accent);color:#08222b;border-color:var(--accent);font-weight:600}
 #tbl{flex:1;overflow:auto;padding:0 6px 10px}
 table{width:100%;border-collapse:collapse}
 th{position:sticky;top:0;background:var(--panel);text-align:left;font-weight:600;color:#93a0b0;
    padding:8px 6px;border-bottom:1px solid var(--line);font-size:11px}
 td{padding:3px 4px;border-bottom:1px solid #262b34}
 tr.sel{background:#2a3340}
 td input{width:100%;padding:3px 5px}
 .idx{color:#93a0b0;width:26px}
 .uv{color:#93a0b0;font-variant-numeric:tabular-nums;white-space:nowrap;font-size:11px}
 .del{color:#ff7a7a;cursor:pointer;user-select:none;padding:0 4px}
 #out{margin:0 14px 14px;height:104px;width:calc(100% - 28px);background:#11141a;color:#8fe;
      border:1px solid var(--line);border-radius:6px;font:11px ui-monospace,monospace;padding:8px}
</style></head><body>
<div id="stage">
  <canvas id="cv"></canvas><canvas id="mag" width="220" height="220"></canvas>
  <div id="hint"><b>click</b> to add a point &middot; <b>wheel</b> zoom &middot; <b>drag</b> or
   <b>space+drag</b> pan &middot; <b>arrows</b> nudge selected 1&nbsp;px (<b>shift</b> 0.25&nbsp;px)
   &middot; <b>del</b> remove</div>
</div>
<aside>
  <h1>Point picker</h1>
  <p class="sub">Click a landmark, then give it a world coordinate in the <b>mocap frame</b>
   (the frame trajs.py is written in).  4 floor points are the minimum; 8&ndash;12 spread
   across the floor is much better.</p>
  <div class="row">
    <label>mode</label>
    <select id="mode">
      <option value="tile">floor tiles (enter i, j)</option>
      <option value="free">free 3D (enter X, Y, Z)</option>
    </select>
    <label>tile</label><input id="tile" type="number" step="0.01" value="__TILE__"> m
  </div>
  <div class="row">
    <label>floor z</label><input id="floorz" type="number" step="0.01" value="0"> m
    <span style="color:#93a0b0">height of the floor in the mocap frame</span>
  </div>
  <div id="tbl"><table><thead><tr>
    <th class="idx">#</th><th class="uv">u, v [px]</th>
    <th id="hX">i</th><th id="hY">j</th><th id="hZ">Z [m]</th><th></th>
  </tr></thead><tbody id="tb"></tbody></table></div>
  <div class="row">
    <button class="primary" id="save">Download points.json</button>
    <button id="copy">Copy JSON</button>
    <button id="clear">Clear all</button>
  </div>
  <textarea id="out" readonly></textarea>
</aside>
<script>
const IMG_SRC="__IMG__", W=__W__, H=__H__;
const img=new Image(); img.src=IMG_SRC;
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
const mag=document.getElementById('mag'), mctx=mag.getContext('2d');
const stage=document.getElementById('stage');
cv.width=W; cv.height=H;
let view={s:1,x:0,y:0}, pts=[], sel=-1, panning=false, space=false, last=null, moved=0;

img.onload=()=>{const r=stage.getBoundingClientRect();
  view.s=Math.min(r.width/W, r.height/H); view.x=0; view.y=0; draw();};
addEventListener('resize',draw);

function applyView(){cv.style.transform=`translate(${view.x}px,${view.y}px) scale(${view.s})`;}
function draw(){
  ctx.clearRect(0,0,W,H); ctx.drawImage(img,0,0);
  pts.forEach((p,i)=>{
    const r=Math.max(3,7/view.s), lw=Math.max(1,1.6/view.s);
    ctx.lineWidth=lw; ctx.strokeStyle=(i===sel)?'#ffb03c':'#3cdcff';
    ctx.beginPath(); ctx.arc(p.u,p.v,r,0,7); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p.u-r*1.9,p.v); ctx.lineTo(p.u+r*1.9,p.v);
    ctx.moveTo(p.u,p.v-r*1.9); ctx.lineTo(p.u,p.v+r*1.9); ctx.stroke();
    ctx.fillStyle=(i===sel)?'#ffb03c':'#3cdcff'; ctx.font=`${Math.max(9,13/view.s)}px sans-serif`;
    ctx.fillText(i+1, p.u+r*2.2, p.v-r*1.2);
  });
  applyView(); sync();
}
function toImg(e){const r=cv.getBoundingClientRect();
  return {u:(e.clientX-r.left)/view.s, v:(e.clientY-r.top)/view.s};}

stage.addEventListener('wheel',e=>{e.preventDefault();
  const r=cv.getBoundingClientRect();
  const ix=(e.clientX-r.left)/view.s, iy=(e.clientY-r.top)/view.s;
  const k=Math.exp(-e.deltaY*0.0016); const ns=Math.min(60,Math.max(0.05,view.s*k));
  view.x+= (e.clientX-r.left) - ix*ns - (e.clientX-r.left) + ix*view.s;
  view.x = e.clientX - stage.getBoundingClientRect().left - ix*ns;
  view.y = e.clientY - stage.getBoundingClientRect().top  - iy*ns;
  view.s=ns; draw();},{passive:false});

stage.addEventListener('mousedown',e=>{panning=true;moved=0;last=[e.clientX,e.clientY];});
addEventListener('mouseup',e=>{
  if(panning && moved<4 && !space && e.target===cv){const p=toImg(e);
    if(p.u>=0&&p.v>=0&&p.u<=W&&p.v<=H){pts.push({u:+p.u.toFixed(2),v:+p.v.toFixed(2),
      X:'',Y:'',Z:''}); sel=pts.length-1; draw(); focusRow();}}
  panning=false;});
addEventListener('mousemove',e=>{
  if(panning){const dx=e.clientX-last[0], dy=e.clientY-last[1];
    moved+=Math.abs(dx)+Math.abs(dy);
    if(space||e.buttons===4||e.buttons===2||moved>4){view.x+=dx;view.y+=dy;applyView();}
    last=[e.clientX,e.clientY];}
  drawMag(e);});
stage.addEventListener('contextmenu',e=>e.preventDefault());

function drawMag(e){
  const p=toImg(e); const Z=8, S=220/Z;
  mctx.imageSmoothingEnabled=false; mctx.fillStyle='#000'; mctx.fillRect(0,0,220,220);
  mctx.drawImage(img, p.u-S/2, p.v-S/2, S, S, 0,0,220,220);
  mctx.strokeStyle='#ffb03c'; mctx.lineWidth=1;
  mctx.beginPath(); mctx.moveTo(110,0);mctx.lineTo(110,220);mctx.moveTo(0,110);mctx.lineTo(220,110);
  mctx.stroke();
  mctx.fillStyle='rgba(0,0,0,.7)'; mctx.fillRect(0,198,220,22);
  mctx.fillStyle='#e8ecf2'; mctx.font='12px ui-monospace,monospace';
  mctx.fillText(`${p.u.toFixed(1)}, ${p.v.toFixed(1)} px`, 8, 213);
}

addEventListener('keydown',e=>{
  if(e.code==='Space'){space=true; return;}
  if(document.activeElement.tagName==='INPUT') return;
  if(sel<0) return;
  const d=e.shiftKey?0.25:1;
  if(e.key==='ArrowLeft'){pts[sel].u-=d;e.preventDefault();}
  else if(e.key==='ArrowRight'){pts[sel].u+=d;e.preventDefault();}
  else if(e.key==='ArrowUp'){pts[sel].v-=d;e.preventDefault();}
  else if(e.key==='ArrowDown'){pts[sel].v+=d;e.preventDefault();}
  else if(e.key==='Delete'||e.key==='Backspace'){pts.splice(sel,1);sel=-1;e.preventDefault();}
  else return;
  pts.forEach(p=>{p.u=+(+p.u).toFixed(2);p.v=+(+p.v).toFixed(2);}); draw();});
addEventListener('keyup',e=>{if(e.code==='Space')space=false;});

function tileMode(){return document.getElementById('mode').value==='tile';}
function sync(){
  const tb=document.getElementById('tb'); tb.innerHTML='';
  const tm=tileMode();
  document.getElementById('hX').textContent = tm?'i (tiles)':'X [m]';
  document.getElementById('hY').textContent = tm?'j (tiles)':'Y [m]';
  document.getElementById('hZ').textContent = tm?'Z [m] (blank = floor)':'Z [m]';
  pts.forEach((p,i)=>{
    const tr=document.createElement('tr'); if(i===sel) tr.className='sel';
    tr.innerHTML=`<td class="idx">${i+1}</td><td class="uv">${p.u.toFixed(1)}, ${p.v.toFixed(1)}</td>`
      +`<td><input data-i="${i}" data-k="X" value="${p.X}"></td>`
      +`<td><input data-i="${i}" data-k="Y" value="${p.Y}"></td>`
      +`<td><input data-i="${i}" data-k="Z" value="${p.Z}"></td>`
      +`<td class="del" data-del="${i}">&times;</td>`;
    tr.onclick=ev=>{if(ev.target.dataset.del===undefined&&ev.target.tagName!=='INPUT'){
      sel=i;draw();}};
    tb.appendChild(tr);});
  tb.querySelectorAll('input').forEach(inp=>{
    inp.oninput=()=>{pts[+inp.dataset.i][inp.dataset.k]=inp.value; emit();};
    inp.onfocus=()=>{sel=+inp.dataset.i;
      document.querySelectorAll('#tb tr').forEach((r,j)=>r.className=(j===sel)?'sel':'');
      drawOnly();};});
  tb.querySelectorAll('.del').forEach(d=>d.onclick=()=>{pts.splice(+d.dataset.del,1);sel=-1;draw();});
  emit();
}
function drawOnly(){const s=sel; ctx.clearRect(0,0,W,H); ctx.drawImage(img,0,0);
  pts.forEach((p,i)=>{const r=Math.max(3,7/view.s);
    ctx.lineWidth=Math.max(1,1.6/view.s); ctx.strokeStyle=(i===s)?'#ffb03c':'#3cdcff';
    ctx.beginPath(); ctx.arc(p.u,p.v,r,0,7); ctx.stroke();});}
function focusRow(){const el=document.querySelector(`#tb input[data-i="${sel}"]`); if(el)el.focus();}

function build(){
  const tile=parseFloat(document.getElementById('tile').value)||0.5;
  const fz=parseFloat(document.getElementById('floorz').value)||0;
  const tm=tileMode(); const out=[];
  pts.forEach(p=>{
    const a=parseFloat(p.X), b=parseFloat(p.Y);
    if(!isFinite(a)||!isFinite(b)) return;
    const z = (p.Z==='' || !isFinite(parseFloat(p.Z))) ? fz : parseFloat(p.Z);
    out.push({uv:[p.u,p.v], world:[tm?a*tile:a, tm?b*tile:b, z]});});
  return {image_size:[W,H], floor_z:fz, tile:tile,
          note:"world coordinates are in the mocap frame used by trajs.py",
          points:out};
}
function emit(){document.getElementById('out').value=JSON.stringify(build(),null,1);}
document.getElementById('mode').onchange=sync;
document.getElementById('tile').oninput=emit;
document.getElementById('floorz').oninput=emit;
document.getElementById('copy').onclick=()=>{const o=document.getElementById('out');
  o.select(); document.execCommand('copy');};
document.getElementById('clear').onclick=()=>{pts=[];sel=-1;draw();};
document.getElementById('save').onclick=()=>{
  const b=new Blob([JSON.stringify(build(),null,1)],{type:'application/json'});
  const a=document.createElement('a'); a.href=URL.createObjectURL(b);
  a.download='points.json'; a.click();};
</script></body></html>
"""


def write_picker_html(frame, out_path: str, tile: float = 0.5, jpeg_quality: int = 95) -> str:
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        raise RuntimeError("failed to encode the frame")
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    html = (_HTML
            .replace("__IMG__", "data:image/jpeg;base64," + b64)
            .replace("__W__", str(frame.shape[1]))
            .replace("__H__", str(frame.shape[0]))
            .replace("__TILE__", f"{tile:g}"))
    with open(out_path, "w") as f:
        f.write(html)
    return out_path
