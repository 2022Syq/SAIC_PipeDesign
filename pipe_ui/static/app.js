"use strict";
const $ = id => document.getElementById(id);
const canvas=$("scene"), ctx=canvas.getContext("2d");
let state={revision:0,routes:[],pipes:[]}, selected=0, inspected=0, picked=null, preview=null, gesture=null;
let cameraBasis=presetBasis("iso"),center=[0,0,0],scale=1,pan=[0,0],width=1,height=1,frame=0;
let editMode=true, busy=false, activeAction=null, mode="route", pending=null, sourceChoices=[],modelView=null;
let modelReady=false, modelFit=false, previewComponents=null;
const loadedModels=new Set();
let serverSpecs={}, serverParameters={}, currentJob=null;
let layoutStage=null;
const warningDisclosure=new Map();
function hasUnsavedInput(){return !!pending||specsChanged();}
function guardUnload(event){if(hasUnsavedInput()){event.preventDefault();event.returnValue="";}}
function resetPanelScroll(){for(const id of ["sidebarContext","inspector"])$(id).scrollTop=0;}
function adoptState(next){state=next;serverSpecs=Object.fromEntries(state.pipes.map(p=>[p.pipe_id,p.spec]));serverParameters=Object.fromEntries(state.pipes.map(p=>[p.pipe_id,JSON.stringify(p.engineering_parameters||{})]));state.pipes.forEach(p=>p.edits={...(p.engineering_parameters||{})});}
function specsChanged(){return state.pipes.some(p=>serverSpecs[p.pipe_id]!==p.spec||serverParameters[p.pipe_id]!==JSON.stringify(p.edits||{}));}
function updateProgress(jobInfo=currentJob){
  currentJob=jobInfo;
  if(!busy||activeAction!=="plan")return;
  const seconds=Math.max(0,Math.floor(Date.now()/1000-(jobInfo?.started_at||Date.now()/1000)));
  $("elapsed").textContent=seconds<60?`已运行 ${seconds} 秒`:`已运行 ${Math.floor(seconds/60)} 分 ${seconds%60} 秒`;
  const progress=jobInfo?.progress;
  const message=progress?`${progress.message} · 已完成 ${progress.completed} / ${progress.total} 根`:"正在准备规划…";
  if($("planningProgress").textContent!==message)$("planningProgress").textContent=message;
}
setInterval(()=>updateProgress(),1000);
try {modelView=new ModelViewport($("modelScene"));}
catch(error){setStatus(error.message,"error");}

function setStatus(message,kind="") {
  $("status").textContent=message;$("indicator").className=kind;
  $("operationError").hidden=kind!=="error";
  $("operationError").textContent=kind==="error"?message:"";
}
function element(tag,text,className) {const node=document.createElement(tag);if(text!==undefined)node.textContent=text;if(className)node.className=className;return node;}
function selectedPointPosition(){return picked ? add(picked.point,gesture?.kind==="edit"?gesture.delta:(pending?.kind==="route"?pending.delta:[0,0,0])):[0,0,0];}
function fmt(value,unit=""){return value===null||value===undefined?"未提供":typeof value==="number"?`${Number(value.toFixed(2))}${unit==="°"?"°":""}`:String(value);}

async function request(path,body) {
  const response=await fetch(path,body?{method:"POST",headers:{"Content-Type":"application/json","X-Workbench-Token":state.token},body:JSON.stringify({...body,revision:state.revision})}:{});
  const result=await response.json();if(!response.ok)throw Error(result.error||"请求失败");return result;
}
async function job(action,args={},message="正在处理…") {
  if(busy)return;
  if(specsChanged()&&["resume_result","open_project"].includes(action)&&
      !window.confirm("规格修改尚未用于规划。继续会放弃这些规格选择，是否继续？"))return;
  busy=true;activeAction=action;currentJob={started_at:Date.now()/1000};renderControls();setStatus(message,"busy");
  if(action==="plan")resetPanelScroll();
  try{
    await request("/api/"+action,args);
    let result;
    while(true){await new Promise(resolve=>setTimeout(resolve,700));result=await request("/api/job");updateProgress(result);if(result.status!=="running")break;}
    if(result.status==="error")throw Error(result.message);
    const oldStage=state.stage,selections=Object.fromEntries(state.pipes.map(p=>[p.pipe_id,p.spec])),parameterEdits=Object.fromEntries(state.pipes.map(p=>[p.pipe_id,{...(p.edits||{})}]));
    adoptState(await request("/api/state"));
    if(["open_project","plan"].includes(action))warningDisclosure.clear();
    if(!["plan","open_project"].includes(action)&&state.stage==="configure")
      for(const pipe of state.pipes)if(selections[pipe.pipe_id]){pipe.spec=selections[pipe.pipe_id];pipe.parameters=pipe.spec_parameters[pipe.spec];pipe.edits=parameterEdits[pipe.pipe_id]||{};}
    selected=Math.max(0,Math.min(selected,state.routes.length-1));inspected=Math.min(inspected,Math.max(0,state.pipes.length-1));
    if(["edit","open_project","plan","configure","resume_result"].includes(action)){pending=null;preview=null;picked=null;previewComponents=null;$("pickInfo").hidden=true;}
    if(result.result?.reset_models){for(const mesh of modelView.models.values())modelView.dispose(mesh);modelView.models.clear();loadedModels.clear();modelFit=false;modelReady=false;}
    renderAll();
    if(oldStage!==state.stage){fitView();resetPanelScroll();}
    setStatus(result.result?.message||"操作完成");
    return result.result;
  }catch(error){setStatus(error.message,"error");}
  finally{busy=false;activeAction=null;renderControls();schedule();}
}
function renderControls(){
  const editing=state.stage==="edit",planning=busy&&activeAction==="plan";
  $("sidebar").classList.toggle("editing-workspace",editing);
  if(layoutStage!==state.stage){$("projectDetails").open=!editing;layoutStage=state.stage;}
  window.removeEventListener("beforeunload",guardUnload);
  if(hasUnsavedInput())window.addEventListener("beforeunload",guardUnload);
  $("routeMode").setAttribute("aria-pressed",String(mode==="route"));
  $("hoseMode").setAttribute("aria-pressed",String(mode==="hose"));
  $("inputPanel").hidden=editing||planning;
  $("projectState").textContent=planning?"规划中":editing?"已有结果":specsChanged()?"规格待规划":modelReady&&state.pipes.length?"输入就绪":"待完善输入";
  $("pointSource").textContent=state.point_source||"未记录来源";
  $("pointSource").title=$("pointSource").textContent;
  $("resultState").hidden=!state.has_result;
  $("resultState").textContent=editing?"正在查看已完成结果 · 已保存":specsChanged()?"规格已修改，尚未重新规划。第 3 步仍为上次结果。":"已保留上次结果，可点击第 3 步返回。";
  $("resultState").classList.toggle("changed",specsChanged());
  updateProgress();$("editPanel").hidden=!editing;
  $("planningPanel").hidden=!planning;
  $("configureStep").disabled=busy||!!pending;
  $("configureStep").title=editing?"查看模型与规格，保留已完成路线":"查看模型与规格";
  $("configureStep").classList.toggle("return-step",editing);
  $("resultStep").disabled=busy||!!pending||!state.has_result;
  $("resultStep").title=state.has_result?"返回上次规划结果及其对应模型与规格，无需重新规划":"完成规划后可进入";
  $("resultStep").classList.toggle("return-step",!editing&&!!state.has_result);
  $("plan").hidden=editing;$("exportCatia").hidden=!editing;
  $("plan").textContent=planning?"正在规划…":state.has_result?"重新生成规划":"生成规划";
  $("plan").disabled=busy||!modelReady||!state.pipes.length;
  $("exportCatia").disabled=busy||!!pending||!state.routes.length||state.routes.some(r=>!r.exportable);
  for(const id of ["chooseModel","readCatia","openProject","routeMode","hoseMode","hangerDemo"])
    $(id).disabled=busy||!!pending;
  const target=state.routes[selected]?.pipe_id;
  $("editTarget").textContent=target?`编辑对象：${target}`:"请点击选择编辑管路";
  $("applyEdit").textContent=target?`应用到 ${target}`:"应用微调";
  $("exportState").hidden=!editing;
  $("exportState").textContent=pending?"预览尚未应用，请先应用或取消":state.routes.some(r=>!r.exportable)?"存在不可导出的几何，请查看检查结果":state.routes.some(r=>r.warnings.length)?"可导出 · 存在约束提醒 / 未判定项":"可导出 · 结果已保存";
  $("applyEdit").disabled=busy||!pending;$("discardEdit").disabled=busy||!pending;
  $("influence").disabled=busy||!!pending;
  document.querySelectorAll(".pipe-card select").forEach(node=>node.disabled=busy);
  $("showFolder").hidden=!state.saved;$("showFolder").disabled=busy;
  $("pendingInfo").hidden=!pending;
  $("modeLabel").textContent=planning?"路径规划":editing?"画面微调":"规格设置";
  $("sceneTitle").textContent=planning?"正在规划路径":editing?"最终工程路线":"模型与端点";
  $("sceneSubtitle").textContent=planning?"完成后自动进入微调与导出":editing?"悬停查看参数 · 拖动调整局部":"设置每根管的规格，然后生成规划";
  $("inputHint").textContent=state.pipes.length?`已识别 ${state.pipes.length} 根管路；起终点及接口方向已读取。`:"选择环境模型，读取命名点组后设置每根管的规格。";
  $("emptyHint").hidden=!!state.pipes.length;
  $("steps").querySelectorAll("li").forEach((li,i)=>{
    const active=i===(planning?1:editing?2:0);li.classList.toggle("active",active);
    if(active)li.setAttribute("aria-current","step");else li.removeAttribute("aria-current");
  });
}
function renderList(){
  $("pipeList").replaceChildren();$("pipeCount").textContent=state.pipes.length;
  state.pipes.forEach((pipe,index)=>{
    const card=element("div",undefined,"pipe-card"+(index===inspected?" active":""));card.dataset.pipeId=pipe.pipe_id;
    card.classList.toggle("final",state.stage==="edit");
    const row=element("div",undefined,"row"),name=element("button",pipe.pipe_id,"pipe-select");name.type="button";name.setAttribute("aria-label",`选择 ${pipe.pipe_id}`);row.append(name);
    const route=state.routes.find(r=>r.pipe_id===pipe.pipe_id);
    name.title=pipe.pipe_id+" · "+(state.specs.find(s=>s.id===pipe.spec)?.label||pipe.spec)+(route?` · ${route.warnings.length} 条检查提醒`:"");
    row.append(element("span",route?`${route.warnings.length} 条警告`:`${pipe.guardians.length} 个固定挂点`,"muted"));card.append(row);
    if(state.stage!=="edit"){
      const select=element("select");select.setAttribute("aria-label",pipe.pipe_id+" 管径与材料");
      state.specs.forEach(spec=>{const option=element("option",spec.label);option.value=spec.id;select.append(option);});select.value=pipe.spec;
      select.addEventListener("change",()=>{pipe.spec=select.value;pipe.parameters=pipe.spec_parameters[select.value];inspect(index);renderControls();});
      select.addEventListener("click",event=>event.stopPropagation());card.append(select);
    }else card.append(element("div",state.specs.find(s=>s.id===pipe.spec)?.label||pipe.spec,"sub"));
    card.addEventListener("click",()=>{inspect(index);if(!pending){selected=state.routes.findIndex(r=>r.pipe_id===pipe.pipe_id);picked=null;renderControls();inspect(index);schedule();}});
    $("pipeList").append(card);
  });
}
function inspect(index){
  if(index<0||index>=state.pipes.length)return;
  inspected=index;
  document.querySelectorAll(".pipe-card").forEach((card,i)=>{
    card.classList.toggle("active",i===index);
    card.classList.toggle("editing",state.stage==="edit"&&state.pipes[i].pipe_id===state.routes[selected]?.pipe_id);
    card.querySelector(".pipe-select").setAttribute("aria-pressed",String(state.stage==="edit"&&state.pipes[i].pipe_id===state.routes[selected]?.pipe_id));
  });
  renderParameters();
  schedule();
}
function renderParameters(){
  const pipe=state.pipes[inspected];
  $("parameters").replaceChildren();$("keyParameters").replaceChildren();$("warnings").replaceChildren();
  if(!pipe)return;
  if(!warningDisclosure.has(pipe.pipe_id))warningDisclosure.set(pipe.pipe_id,new Map());
  const disclosure=warningDisclosure.get(pipe.pipe_id);
  const route=state.routes.find(r=>r.pipe_id===pipe.pipe_id);
  $("parameterTitle").textContent="正在查看："+pipe.pipe_id;
  $("parameterHint").textContent=route?(pending?"拖动预览尚未应用，下列为已应用的参数。":"显示当前路线实际值与工艺要求；返回规格设置可修改参数。"):"当前显示选定规格；下方带输入框的参数可直接修改，生成规划时生效。";
  const rows=route?.parameters||pipe.parameters;
  const edits=pipe.edits||{};
  for(const item of rows)for(const target of ["actual","recommended","minimum"]){const key=item[`${target}_key`];if(key&&Object.prototype.hasOwnProperty.call(edits,key))item[target]=edits[key];}
  const keys=route?["金属外径","总长度","金属最小弯曲半径","胶管长度","胶管中心距起点（沿线）","每端铝套长度"]:["金属牌号","金属外径","金属壁厚","胶管型号","胶管外径","固定挂点"];
  for(const key of keys){const item=rows.find(row=>row.label===key);if(!item)continue;
    const entry=element("div",undefined,item.status);entry.append(element("dt",key),element("dd",fmt(item.actual,item.unit)));
    $("keyParameters").append(entry);
  }
  const appendValue=(tr,item,target)=>{
    const td=element("td");const key=item[`${target}_key`];
    if(!key||state.stage==="edit")td.textContent=item[target]==null?(target==="actual"?"未提供":"—"):fmt(item[target],item.unit);
    else{
      const input=document.createElement("input");input.className="parameter-input";input.type=item.input_type||"number";input.value=item[target]==null?"":String(item[target]);input.setAttribute("aria-label",`${item.label} ${target==="actual"?"实际/选定":target==="recommended"?"推荐":"极限"}`);input.title=item.source;input.disabled=busy;
      if(input.type==="number"){input.step=item.unit==="°"?"0.1":"0.01";input.min="0";}
      input.addEventListener("change",()=>{const raw=input.value.trim(),next=input.type==="number"?(raw===""?null:Number(raw)):raw; if(input.type==="number"&&raw!==""&&!Number.isFinite(next))return; edits[key]=next;item[target]=next;pipe.edits=edits;renderParameters();renderControls();});
      td.append(input);
    }
    tr.append(td);
  };
  for(const item of rows){
    const tr=element("tr",undefined,item.status);const label=element("td",item.label);label.title=item.source;tr.append(label);
    appendValue(tr,item,"actual");appendValue(tr,item,"recommended");appendValue(tr,item,"minimum");
    $("parameters").append(tr);
  }
  $("warningsPanel").hidden=!route;
  if(route){
    $("warningCount").textContent=route.warnings.length;
    // Group messages without dropping any detail or pretending repeated checks are distinct locations.
    const groups=new Map();
    for(const warning of route.warnings){
      const unknown=/未提供|缺少|未校核|未重新校核|无法判定/.test(warning);
      const category=unknown?"未判定":"约束提醒",name=warning.split("：")[0],key=category+name;
      if(!groups.has(key))groups.set(key,{category,name,items:[]});groups.get(key).items.push(warning);
    }
    for(const category of ["约束提醒","未判定"]){
      const matching=[...groups.values()].filter(g=>g.category===category);if(!matching.length)continue;
      $("warnings").append(element("h3",category));
      for(const group of matching){const details=element("details"),summary=element("summary",`${group.name}${group.items.length>1?` · ${group.items.length} 项`:""}`),list=element("ul");
        const key=group.category+group.name;
        details.open=disclosure.get(key)||false;
        details.addEventListener("toggle",()=>disclosure.set(key,details.open));
        group.items.forEach(item=>list.append(element("li",item)));details.append(summary,list);$("warnings").append(details);
      }
    }
    if(!route.warnings.length)$("warnings").append(element("p","已执行的检查未发现违规；缺少数据的条目仍未判定。","hint"));
  }
}
function renderAll(){
  renderList();inspect(inspected);renderControls();
  $("saddles").replaceChildren();for(const row of state.saddles||[]){const tr=element("tr");row.forEach(value=>tr.append(element("td",String(value))));$("saddles").append(tr);}
  schedule();
}
function fitView(){
  const points=state.pipes.flatMap(p=>[p.start,p.goal]);
  for(const model of modelView?.models.values()||[]){const [a,b]=model.bounds;for(let i=0;i<8;i++)points.push([0,1,2].map(k=>(i&(1<<k))?b[k]:a[k]));}
  if(!points.length)return;
  const lo=[0,1,2].map(k=>Math.min(...points.map(p=>p[k]))),hi=[0,1,2].map(k=>Math.max(...points.map(p=>p[k])));
  center=mul(add(lo,hi),.5);pan=[0,0];
  const b=basis();let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
  function include(x,y,z){const dx=x-center[0],dy=y-center[1],dz=z-center[2],px=dx*b.right[0]+dy*b.right[1]+dz*b.right[2],py=dx*b.up[0]+dy*b.up[1]+dz*b.up[2];minX=Math.min(minX,px);maxX=Math.max(maxX,px);minY=Math.min(minY,py);maxY=Math.max(maxY,py);}
  for(const pipe of state.pipes){include(...pipe.start);include(...pipe.goal);}
  for(const route of state.routes)for(const point of route.path)include(...point);
  for(const model of modelView?.models.values()||[])for(let i=0;i<model.vertices.length;i+=6)include(model.vertices[i],model.vertices[i+1],model.vertices[i+2]);
  scale=Math.min((width-80)/Math.max(10,maxX-minX),(height-160)/Math.max(10,maxY-minY));
  pan=[-(minX+maxX)*.5*scale,(minY+maxY)*.5*scale];
  schedule();
}
function resize(){const rect=$("viewer").getBoundingClientRect();width=rect.width;height=rect.height;const dpr=Math.min(devicePixelRatio||1,2);canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);schedule();}
new ResizeObserver(resize).observe($("viewer"));
function pointMark(point,color,size,label){const p=project(point);ctx.beginPath();ctx.arc(p[0],p[1],size,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();ctx.strokeStyle="#ffffff";ctx.lineWidth=1.5;ctx.stroke();if(label){ctx.font="10px Microsoft YaHei";ctx.fillStyle=color;ctx.fillText(label,p[0]+8,p[1]-7);}}
function draw(){
  ctx.clearRect(0,0,width,height);
  if(modelView){
    const components=[];
    state.routes.forEach((route,i)=>components.push(...(i===selected&&previewComponents?previewComponents:route.components).map(c=>({...c,highlighted:i===selected}))));
    try{modelView.draw({width,height,center,scale,pan,...basis()},components,-1,null,true);}
    catch(error){setStatus(error.message,"error");}
  }
  state.pipes.forEach((pipe,index)=>{
    const prefix=state.stage==="edit"?"":pipe.pipe_id+" ",active=index===inspected;
    pointMark(pipe.start,"#2c8876",active?5:3,active?prefix+"起点":null);pointMark(pipe.goal,"#b46b4d",active?5:3,active?prefix+"终点":null);
    for(const guardian of pipe.guardians)pointMark(guardian.point,"#61849a",3);
  });
  if(state.stage==="edit"&&mode==="route"&&picked){pointMark(selectedPointPosition(),"#d29b36",6);drawMoveAxes();}
  if(state.stage==="edit"&&mode==="hose"&&state.routes[selected]){
    for(const handle of hoseHandles()){pointMark(handle.point,handle.kind==="move"?"#ad6553":"#d39e39",6,handle.kind==="move"?"整体滑动":handle.kind==="start"?"拉动起端":"拉动末端");}
  }
  const b=basis(),origin=[width-60,height-55];
  for(const axis of moveAxes){const x=origin[0]+dot(axis.vector,b.right)*30,y=origin[1]-dot(axis.vector,b.up)*30;ctx.beginPath();ctx.moveTo(...origin);ctx.lineTo(x,y);ctx.strokeStyle=axis.color;ctx.lineWidth=2;ctx.stroke();ctx.fillStyle=axis.color;ctx.font="11px sans-serif";ctx.fillText(axis.name,x+3,y);}
}
function pick(x,y,onlyIndex=null){
  let best=null;
  state.routes.forEach((route,index)=>{
    if(onlyIndex!==null&&onlyIndex!==index)return;
    const st=stations(route.path);
    for(let i=1;i<route.path.length;i++){
      const a=project(route.path[i-1]),b=project(route.path[i]),d=[b[0]-a[0],b[1]-a[1]],dd=dot(d,d);
      const t=dd?Math.max(0,Math.min(1,dot([x-a[0],y-a[1]],d)/dd)):0;
      const distance=Math.hypot(x-a[0]-t*d[0],y-a[1]-t*d[1]),depth=a[2]+t*(b[2]-a[2]);
      if(distance<Math.max(10,route.radius*scale+5)&&(!best||distance<best.distance-1||(Math.abs(distance-best.distance)<=1&&depth>best.depth))){
        const station=st[i-1]+t*(st[i]-st[i-1]);best={index,distance,depth,station,point:at(route.path,station,st)};
      }
    }
  });return best;
}
function slicePath(path,start,end){const st=stations(path);return [at(path,start,st),...path.filter((p,i)=>st[i]>start&&st[i]<end),at(path,end,st)];}
function makePreviewComponents(route,path,hose){
  const total=stations(path).at(-1),L=hose.hose_length_mm,S=hose.sleeve_length_mm,s=hose.position_s_mm;
  const cuts=[0,s-L/2-S,s-L/2,s+L/2,s+L/2+S,total],kinds=["aluminum","sleeve","hose","sleeve","aluminum"];
  return kinds.map((kind,i)=>{const source=route.components.find(c=>c.kind===kind);return {kind,color:source.color,radius:source.radius,path:slicePath(path,Math.max(0,cuts[i]),Math.min(total,cuts[i+1]))};}).filter((c,i)=>cuts[i+1]-cuts[i]>1e-7);
}
function displayedHose(){const route=state.routes[selected];if(!route)return null;const edit=gesture?.hose||pending;
  return edit?.kind==="hose"?{...route.hose,position_s_mm:edit.center,hose_length_mm:edit.length}:route.hose;}
function hoseHandles(){const route=state.routes[selected],hose=displayedHose();if(!route||!hose)return[];const s=hose.position_s_mm,L=hose.hose_length_mm;
  return [{kind:"start",station:s-L/2},{kind:"move",station:s},{kind:"end",station:s+L/2}].map(h=>({...h,point:at(route.path,h.station)}));}
function pickHoseHandle(x,y){let best=null;for(const handle of hoseHandles()){const p=project(handle.point),distance=Math.hypot(x-p[0],y-p[1]);if(distance<11&&(!best||distance<best.distance))best={...handle,distance};}return best;}
function position(event){const rect=canvas.getBoundingClientRect();return[event.clientX-rect.left,event.clientY-rect.top];}
canvas.addEventListener("contextmenu",event=>event.preventDefault());
canvas.addEventListener("pointerdown",event=>{
  if(busy||gesture||![0,1,2].includes(event.button))return;
  event.preventDefault();const [x,y]=position(event);let handle=null,axis=null;
  if(!pending&&state.stage==="edit"&&event.button===0){if(mode==="hose")handle=pickHoseHandle(x,y);else axis=pickAxis(x,y);}
  canvas.setPointerCapture(event.pointerId);
  gesture={kind:handle?"hose":axis?"edit":event.button===1?"pan":event.button===0?"orbit":"context",axis:axis?.name,handle:handle?.kind,
    start:[x,y],last:[x,y],pan:[...pan],pointer:event.pointerId,moved:false,delta:[0,0,0],
    hoseBase:handle?{...state.routes[selected].hose}:null,hit:pick(x,y)};
});
canvas.addEventListener("pointermove",event=>{
  if(busy)return;const[x,y]=position(event);
  if(!gesture){const hit=pick(x,y);if(hit){const i=state.pipes.findIndex(p=>p.pipe_id===state.routes[hit.index].pipe_id);if(inspected!==i)inspect(i);}
    if(state.stage==="configure")state.pipes.forEach((pipe,i)=>{if([pipe.start,pipe.goal].some(point=>{const p=project(point);return Math.hypot(x-p[0],y-p[1])<8;}))inspect(i);});
    canvas.style.cursor=!pending&&state.stage==="edit"&&(mode==="hose"?pickHoseHandle(x,y):pickAxis(x,y))?"grab":hit?"pointer":"default";return;}
  if(event.pointerId!==gesture.pointer)return;
  const dx=x-gesture.start[0],dy=y-gesture.start[1];gesture.moved||=Math.hypot(dx,dy)>3;if(!gesture.moved)return;
  if(gesture.kind==="orbit"){orbitBy(x-gesture.last[0],y-gesture.last[1]);gesture.last=[x,y];}
  else if(gesture.kind==="pan")pan=[gesture.pan[0]+dx,gesture.pan[1]+dy];
  else if(gesture.kind==="edit"){
    try{gesture.delta=dragDelta(dx,dy,gesture.axis);preview=bump(state.routes[selected].path,picked.station,gesture.delta,Number($("influence").value));
      previewComponents=makePreviewComponents(state.routes[selected],preview,state.routes[selected].hose);
      $("pickInfo").textContent=`${gesture.axis} 方向移动 ${norm(gesture.delta).toFixed(1)} mm · 拖动预览`;}
    catch(error){gesture.delta=[0,0,0];preview=null;previewComponents=null;setStatus(error.message,"error");}
  }else if(gesture.kind==="hose"){
    const route=state.routes[selected],hit=pick(x,y,selected);if(hit){
      const base=gesture.hoseBase,S=base.sleeve_length_mm,L=base.hose_length_mm,total=route.stations.at(-1);
      let start=base.position_s_mm-L/2,end=base.position_s_mm+L/2;
      if(gesture.handle==="move"){const s=Math.max(S+L/2,Math.min(total-S-L/2,hit.station));start=s-L/2;end=s+L/2;}
      else if(gesture.handle==="start")start=Math.max(S,Math.min(end-.1,hit.station));
      else end=Math.min(total-S,Math.max(start+.1,hit.station));
      gesture.hose={kind:"hose",center:(start+end)/2,length:end-start};
      previewComponents=makePreviewComponents(route,route.path,{...base,position_s_mm:(start+end)/2,hose_length_mm:end-start});
      $("pickInfo").hidden=false;$("pickInfo").textContent=`胶管长度 ${(end-start).toFixed(1)} mm · 中心距起点 ${((start+end)/2).toFixed(1)} mm · 预览`;
    }
  }schedule();
});
canvas.addEventListener("pointerup",event=>{
  if(!gesture||gesture.pointer!==event.pointerId)return;
  const last=gesture;gesture=null;canvas.releasePointerCapture(event.pointerId);
  if(last.kind==="edit"&&last.moved&&norm(last.delta)>.05){pending={kind:"route",station:picked.station,delta:last.delta,influence:Number($("influence").value)};}
  else if(last.kind==="hose"&&last.moved&&last.hose){pending=last.hose;}
  else if(!last.moved&&!pending&&state.stage==="edit"){
    if(last.hit){selected=last.hit.index;picked=mode==="route"?last.hit:null;inspect(state.pipes.findIndex(p=>p.pipe_id===state.routes[selected].pipe_id));
      $("pickInfo").hidden=false;$("pickInfo").textContent=mode==="route"?`${state.routes[selected].pipe_id} · 拖动 X / Y / Z 轴调整路径`:`${state.routes[selected].pipe_id} · 拖动胶管两端改长度，拖动中部滑动`;}
    else{picked=null;$("pickInfo").hidden=true;}
  }
  if(pending)setStatus("拖动预览已准备好，点击“应用微调”进行局部重建与约束检查");
  renderControls();renderParameters();schedule();
});
function cancelGesture(){gesture=null;preview=null;previewComponents=null;pending=null;$("pickInfo").hidden=!picked;renderControls();renderParameters();schedule();}
canvas.addEventListener("pointercancel",cancelGesture);
document.addEventListener("keydown",event=>{if(event.key==="Escape"&&!busy)cancelGesture();});
canvas.addEventListener("wheel",event=>{event.preventDefault();const[x,y]=position(event),factor=Math.exp(-event.deltaY*.001);pan=[x-width/2-(x-width/2-pan[0])*factor,y-height/2-(y-height/2-pan[1])*factor];scale=Math.max(.005,Math.min(50,scale*factor));schedule();},{passive:false});
$("discardEdit").onclick=()=>{cancelGesture();setStatus("已取消本次预览");};
$("applyEdit").onclick=()=>job("edit",{pipe_id:state.routes[selected].pipe_id,edit:pending},"正在应用微调并检查约束…");
$("plan").onclick=()=>job("plan",{specs:Object.fromEntries(state.pipes.map(p=>[p.pipe_id,p.spec])),parameters:Object.fromEntries(state.pipes.map(p=>[p.pipe_id,{...(p.edits||{})}]))},"正在生成规划，请等待最终工程路线…");
$("exportCatia").onclick=()=>job("export",{},"正在生成 CATPart，请保持 CATIA 可用…");
$("showFolder").onclick=()=>job("show_folder",{},"正在打开结果目录…");
$("openProject").onclick=()=>job("open_project",{},"请选择已保存工程或当前工程路线结果…");
$("configureStep").onclick=()=>{
  if(state.stage==="edit")job("configure",{},"正在保存当前路线并返回模型与规格…");
  else resetPanelScroll();
};
$("resultStep").onclick=()=>{
  if(state.stage!=="edit")job("resume_result",{},"正在返回已完成的规划结果…");
  else resetPanelScroll();
};
$("chooseModel").onclick=()=>job("choose_model",{},"请选择环境模型…");
$("readCatia").onclick=async()=>{const result=await job("catia_sources",{},"正在识别 CATIA 管路点组…");if(!result?.sources)return;sourceChoices=result.sources;$("sourceList").replaceChildren();
  sourceChoices.forEach((source,i)=>{const label=element("label"),check=element("input");check.type="checkbox";check.value=i;check.checked=source.document===sourceChoices[0].document;label.append(check,document.createTextNode(" "+source.document+" / "+source.body),element("small",source.pipe_ids.join("、")));$("sourceList").append(label);});$("sourceDialog").showModal();};
$("sourceConfirm").onclick=()=>{const sources=[...$("sourceList").querySelectorAll("input:checked")].map(input=>sourceChoices[Number(input.value)]);$("sourceDialog").close();job("import_catia",{sources},"正在读取管路起终点与固定约束…");};
$("sourceCancel").onclick=()=>$("sourceDialog").close();
$("hangerDemo").onclick=()=>$("demoDialog").showModal();$("demoClose").onclick=()=>$("demoDialog").close();
for(const name of ["route","hose"]){$(name+"Mode").onclick=()=>{mode=name;picked=null;$("routeMode").classList.toggle("active",name==="route");$("hoseMode").classList.toggle("active",name==="hose");$("influenceLabel").hidden=name==="hose";$("pickInfo").hidden=true;
  $("editHint").textContent=name==="route"?"点击管路选点，再拖动 X、Y、Z 轴。起终点、接口方向和挂点固定。":"点击选择一根管。拖动胶管端部改变长度，拖动中部沿管滑动；铝套长度固定并跟随。";renderControls();schedule();};}
$("influence").oninput=()=>$("influenceValue").textContent=$("influence").value+" mm";
$("fit").onclick=fitView;for(const view of ["iso","xy","xz","yz"])$(view).onclick=()=>{setView(view);fitView();};

async function loadModels(){
  if(!modelView){$("modelStatus").textContent="三维显示不可用：请检查浏览器硬件加速后刷新。";return;}
  try{
    const data=await request("/api/models");
    $("modelName").textContent=data.items.length?data.items.map(item=>item.name).join("、"):"尚未选择模型";
    $("modelName").title=data.base_dir;
    $("modelSummary").textContent=$("modelName").textContent;
    $("modelSummary").title=$("modelName").textContent;
    for(const item of data.items){
      if(item.status!=="ready"||loadedModels.has(item.id))continue;
      const response=await fetch(item.url);if(!response.ok)throw Error("环境网格读取失败");
      const binary=await response.arrayBuffer();modelView.addModel(item,binary);modelView.models.get(item.id).opacity=.18;loadedModels.add(item.id);
    }
    modelReady=data.status==="done"&&data.items.length>0&&data.items.every(i=>i.status==="ready");
    const errors=data.items.filter(i=>i.status==="error");
    $("modelStatus").textContent=errors.length?errors.map(i=>i.error).join("；"):modelReady?"":`正在读取环境模型${data.current?" · "+data.current:""}…`;
    if(data.status==="error")$("modelStatus").textContent=data.current;
    if(modelReady&&!modelFit){modelFit=true;fitView();}
    renderControls();schedule();
  }catch(error){$("modelStatus").textContent=error.message;}
  setTimeout(loadModels,1800);
}
async function init(){try{
  adoptState(await request("/api/state"));busy=state.job.status==="running";activeAction=busy?state.job.action:null;currentJob=state.job;renderAll();resize();fitView();loadModels();
  if(busy){setStatus("后台操作仍在进行，正在恢复进度…","busy");let current;
    do{await new Promise(resolve=>setTimeout(resolve,700));current=await request("/api/job");updateProgress(current);}while(current.status==="running");
    adoptState(await request("/api/state"));busy=false;renderAll();fitView();
    setStatus(current.status==="error"?current.message:(current.result?.message||"后台操作完成"),current.status==="error"?"error":"");
  }else setStatus(state.stage==="edit"?"已打开工程路线，可以微调或导出":"已读取当前工程输入，请确认规格后生成规划");
}catch(error){busy=false;setStatus(error.message,"error");}}
init();
