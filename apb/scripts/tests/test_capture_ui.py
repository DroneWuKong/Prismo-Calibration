"""Run UI lifecycle logic with a simulated DOM and transport, without aircraft."""
from pathlib import Path
import re
import subprocess


def test_capture_errors_keep_board_visible_and_stale_polls_do_not_end_new_capture():
    html=(Path(__file__).resolve().parents[1]/"calibration_app.html").read_text()
    script=re.search(r"<script>(.*?)</script>",html,re.S).group(1)
    harness=r"""
const vm=require('vm'), assert=require('assert');
const elements={};
function element(id){return elements[id]??=( {
 value:id==='square'?'15':id==='preset'?'px13':'', checked:true, textContent:'',
 style:{}, classList:{hidden:id==='target',add(){this.hidden=true},remove(){this.hidden=false},contains(){return this.hidden},toggle(){}},
 addEventListener(){}, getContext(){return {fillRect(){}}}, requestFullscreen:async()=>{throw Error('denied')}
});}
let status={state:'idle',count:0,message:'idle'};
let rejectStart=true, pendingStatus=null;
const context={
 console,document:{getElementById:element,querySelectorAll:()=>[],addEventListener(){},hidden:false,
 fullscreenElement:null,exitFullscreen:async()=>{throw Error('unexpected exit')}},
 devicePixelRatio:2,screen:{width:1440,height:900},innerWidth:1440,innerHeight:900,
 localStorage:{getItem(){return null},setItem(){}},window:{addEventListener(){}},
 location:{protocol:'file:'},setTimeout(){},
 fetch:async(url)=>{if(url==='/api/token')return {json:async()=>({token:'test'})};
 if(url==='/api/status'){if(pendingStatus)return pendingStatus;return {ok:true,json:async()=>status};}
 return {ok:!rejectStart,json:async()=>rejectStart?{error:'Flight controller not connected'}:{ok:true}}}
};
vm.createContext(context);
vm.runInContext(SCRIPT,context);
(async()=>{
 await vm.runInContext("$('start').onclick()",context);
 assert.equal(element('target').classList.hidden,false);
 assert.match(element('targetStatus').textContent,/Flight controller not connected/);
 rejectStart=false;
 await vm.runInContext("$('start').onclick()",context);
 status={state:'error',count:0,message:'Camera unavailable'};
 await vm.runInContext("poll()",context);
 assert.equal(element('target').classList.hidden,false);
 assert.equal(element('targetStatus').textContent,'Camera unavailable');
 // A terminal response requested before a new capture must not close that capture.
 let release;pendingStatus=new Promise(r=>release=r);
 const oldPoll=vm.runInContext("poll()",context);
 await vm.runInContext("$('start').onclick()",context);
 release({ok:true,json:async()=>({state:'stopped',count:0,message:'old session'})});
 await oldPoll;
 assert.equal(vm.runInContext('active',context),true);
 assert.equal(element('target').classList.hidden,false);
})().catch(e=>{console.error(e);process.exitCode=1});
"""
    subprocess.run(["node","-e",harness.replace("SCRIPT",repr(script))],check=True)
