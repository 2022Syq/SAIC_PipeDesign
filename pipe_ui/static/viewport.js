// Migrated from the supplied hvwire pipe editor; material colors retained for every pipe.
/* Local WebGL2 renderer; shares the editor's orthographic camera exactly. */
"use strict";
class ModelViewport {
  constructor(canvas) {
    this.canvas = canvas;
    const gl = canvas.getContext("webgl2", {alpha: true, antialias: true});
    if (!gl) throw Error("浏览器未启用 WebGL2，无法显示数模。请启用硬件加速后重新打开页面。");
    this.gl = gl;
    this.models = new Map();
    this.routes = [];
    this.routeOutlines = [];
    const shader = (type, source) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, source); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
      return s;
    };
    const vertex = shader(gl.VERTEX_SHADER, `#version 300 es
      in vec3 position; in vec3 normal;
      uniform vec3 center, right, up, depth, translation;
      uniform vec2 extent, pan; uniform float scale, farPlane;
      out float lighting;
      void main() {
        vec3 p = position+translation-center;
        gl_Position = vec4((vec2(dot(p,right),dot(p,up))*scale+pan)/extent,
                          -dot(p,depth)/farPlane,1.0);
        lighting = .4+.6*abs(dot(normal,normalize(depth+up*.55+right*.35)));
      }`);
    const fragment = shader(gl.FRAGMENT_SHADER, `#version 300 es
      precision highp float; in float lighting; uniform vec4 color; out vec4 outColor;
      void main() { outColor=vec4(color.rgb*lighting,color.a); }`);
    this.program = gl.createProgram();
    gl.attachShader(this.program, vertex); gl.attachShader(this.program, fragment); gl.linkProgram(this.program);
    gl.deleteShader(vertex); gl.deleteShader(fragment);
    if (!gl.getProgramParameter(this.program, gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(this.program));
    this.uniforms = {};
    for (const key of ["center","right","up","depth","extent","pan","scale","farPlane","color","translation"])
      this.uniforms[key] = gl.getUniformLocation(this.program, key);
    this.attributes = [gl.getAttribLocation(this.program,"position"), gl.getAttribLocation(this.program,"normal")];
    canvas.addEventListener("webglcontextlost", e => {e.preventDefault(); this.lost = true;});
    canvas.addEventListener("webglcontextrestored", () => location.reload());
  }
  upload(vertices, indices) {
    const gl = this.gl, vao = gl.createVertexArray(), vb = gl.createBuffer(), ib = gl.createBuffer();
    gl.bindVertexArray(vao);
    gl.bindBuffer(gl.ARRAY_BUFFER,vb); gl.bufferData(gl.ARRAY_BUFFER,vertices,gl.STATIC_DRAW);
    this.attributes.forEach((attribute,i) => {
      gl.enableVertexAttribArray(attribute); gl.vertexAttribPointer(attribute,3,gl.FLOAT,false,24,i*12);
    });
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,ib); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER,indices,gl.STATIC_DRAW);
    return {vao,vb,ib,count:indices.length};
  }
  dispose(mesh) {
    const gl=this.gl;
    gl.deleteVertexArray(mesh.vao); gl.deleteBuffer(mesh.vb); gl.deleteBuffer(mesh.ib);
  }
  addModel(item, binary) {
    const counts=new Uint32Array(binary,0,2), n=counts[0], t=counts[1];
    if (binary.byteLength !== 8+n*24+t*12) throw Error("数模网格数据不完整："+item.name);
    const vertices=new Float32Array(binary,8,n*6), indices=new Uint32Array(binary,8+n*24,t*3);
    const mesh=this.upload(vertices,indices);
    this.models.set(item.id,{...item,...mesh,vertices,indices,visible:true,opacity:.45,offset:[0,0,0]});
  }
  isOccluded(point,camera) {
    // A click ray begins at the centreline and points toward the camera.
    // Transparent parts intentionally permit selecting the geometry behind them.
    const d=camera.depth, cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
      dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
    for(const m of this.models.values()) {
      if(!m.visible||m.opacity<.999) continue;
      const localPoint=point.map((v,i)=>v-(m.offset?.[i]||0));
      let near=0,far=Infinity;
      for(let k=0;k<3;k++) {
        if(Math.abs(d[k])<1e-10) {if(localPoint[k]<m.bounds[0][k]||localPoint[k]>m.bounds[1][k]) far=-1;}
        else {const a=(m.bounds[0][k]-localPoint[k])/d[k],b=(m.bounds[1][k]-localPoint[k])/d[k];near=Math.max(near,Math.min(a,b));far=Math.min(far,Math.max(a,b));}
      }
      if(far<near) continue;
      const vertices=m.vertices, ids=m.indices;
      for(let i=0;i<ids.length;i+=3) {
        const read=j=>{const k=ids[j]*6;return [vertices[k],vertices[k+1],vertices[k+2]];};
        const a=read(i),b=read(i+1),c=read(i+2),e1=b.map((v,k)=>v-a[k]),e2=c.map((v,k)=>v-a[k]);
        const p=cross(d,e2),det=dot(e1,p);
        if(Math.abs(det)<1e-10) continue;
        const t=localPoint.map((v,k)=>v-a[k]),u=dot(t,p)/det;
        if(u<0||u>1) continue;
        const q=cross(t,e1),v=dot(d,q)/det;
        if(v>=0&&u+v<=1&&dot(e2,q)/det>1e-4) return true;
      }
    }
    return false;
  }
  pickComponent(name, point, camera) {
    const m=[...this.models.values()].find(m=>m.name===name&&m.visible&&m.opacity>0);
    if(!m) return null;
    let near=-Infinity,far=Infinity;
    for(let k=0;k<3;k++) {
      const p=point[k]-(m.offset?.[k]||0),d=camera.depth[k];
      if(Math.abs(d)<1e-10) {
        if(p<m.bounds[0][k]||p>m.bounds[1][k]) return null;
      } else {
        const a=(m.bounds[0][k]-p)/d,b=(m.bounds[1][k]-p)/d;
        near=Math.max(near,Math.min(a,b));far=Math.min(far,Math.max(a,b));
      }
    }
    if(far<near) return null;
    const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
    let best=null;
    // Orthographic hit test against the actual triangles, including translation.
    for(let i=0;i<m.indices.length;i+=3) {
      const vertices=[0,1,2].map(j=>{const k=m.indices[i+j]*6;
        return [0,1,2].map(a=>m.vertices[k+a]+(m.offset?.[a]||0));});
      const p=vertices.map(v=>{const d=v.map((x,k)=>x-point[k]);return [dot(d,camera.right),dot(d,camera.up)];});
      const [a,b,c]=p, det=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1]);
      if(Math.abs(det)<1e-10) continue;
      const u=((b[1]-c[1])*(-c[0])+(c[0]-b[0])*(-c[1]))/det;
      const v=((c[1]-a[1])*(-c[0])+(a[0]-c[0])*(-c[1]))/det,w=1-u-v;
      if(Math.min(u,v,w)<0) continue;
      const hit=[0,1,2].map(k=>u*vertices[0][k]+v*vertices[1][k]+w*vertices[2][k]);
      const depth=dot(hit,camera.depth);
      if(!best||depth>best.depth) best={point:hit,depth,component:m.name};
    }
    return best;
  }
  tube(path,radius) {
    const vertices=[], indices=[], sides=12;
    const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
    const unit=a=>{const n=Math.hypot(...a)||1;return a.map(v=>v/n);};
    for(let i=0;i<path.length;i++) {
      const a=path[Math.max(0,i-1)], b=path[Math.min(path.length-1,i+1)];
      const tangent=unit(b.map((v,j)=>v-a[j]));
      const u=unit(cross(tangent,Math.abs(tangent[2])<.9?[0,0,1]:[0,1,0])), v=cross(tangent,u);
      for(let j=0;j<sides;j++) {
        const angle=j*2*Math.PI/sides, normal=u.map((x,k)=>x*Math.cos(angle)+v[k]*Math.sin(angle));
        vertices.push(...path[i].map((x,k)=>x+radius*normal[k]),...normal);
        if(i) {const c=i*sides+j, d=i*sides+(j+1)%sides;
          indices.push(c,d,c-sides,d,d-sides,c-sides);}
      }
    }
    return this.upload(new Float32Array(vertices),new Uint32Array(indices));
  }
  draw(camera,routes,selected,preview,xray) {
    if(this.lost) throw Error("三维显示上下文已丢失，请刷新页面恢复数模");
    const gl=this.gl, c=this.canvas, dpr=Math.min(window.devicePixelRatio||1,2);
    const w=Math.round(camera.width*dpr), h=Math.round(camera.height*dpr);
    if(c.width!==w||c.height!==h) {c.width=w;c.height=h;}
    gl.viewport(0,0,w,h); gl.clearColor(0,0,0,0); gl.depthMask(true);
    gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT); gl.useProgram(this.program);
    const u=this.uniforms;
    for(const k of ["center","right","up","depth"]) gl.uniform3fv(u[k],camera[k]);
    gl.uniform2f(u.extent,camera.width/2,camera.height/2);
    gl.uniform2f(u.pan,camera.pan[0],-camera.pan[1]); gl.uniform1f(u.scale,camera.scale);
    let far=10000;
    for(const m of this.models.values()) for(const p of m.bounds)
      far=Math.max(far,Math.hypot(...p.map((v,i)=>v+(m.offset?.[i]||0)-camera.center[i]))*2);
    for(const r of routes) for(const p of r.path) far=Math.max(far,Math.hypot(...p.map((v,i)=>v-camera.center[i]))*2);
    gl.uniform1f(u.farPlane,far);
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE); gl.disable(gl.BLEND);
    const colors={guide_surface:[.32,.66,.82],obstacle:[.64,.68,.72],existing_hv:[.94,.61,.27],pipe:[.6,.78,.58],ignore:[.5,.5,.5]};
    const render=(mesh,color,alpha)=>{gl.uniform3fv(u.translation,mesh.offset||[0,0,0]);gl.uniform4f(u.color,...color,alpha);gl.bindVertexArray(mesh.vao);gl.drawElements(gl.TRIANGLES,mesh.count,gl.UNSIGNED_INT,0);};
    const models=[...this.models.values()].filter(m=>m.visible&&m.opacity>0);
    for(const m of models.filter(m=>m.opacity>=.999)) render(m,colors[m.role]||[.69,.58,.76],1);
    // Opaque routes participate in CAD depth testing; x-ray routes are drawn last.
    const renderRoutes=()=>{
      // A display-only shell adds a constant screen-width selection outline.
      // It never enters route data, hit testing, constraints or CAD export.
      gl.depthMask(false);
      routes.forEach((r,i)=>{
        const path=preview&&i===selected?preview:r.path;
        if(!r.highlighted){
          if(this.routeOutlines[i])this.dispose(this.routeOutlines[i]);
          this.routeOutlines[i]=null;return;
        }
        const radius=r.radius+2/camera.scale;
        if(this.routeOutlines[i]?.path!==path||this.routeOutlines[i]?.radius!==radius){
          if(this.routeOutlines[i])this.dispose(this.routeOutlines[i]);
          this.routeOutlines[i]={...this.tube(path,radius),path,radius};
        }
        render(this.routeOutlines[i],[.08,.48,.43],1);
      });
      gl.depthMask(true);
      routes.forEach((r,i)=>{
        const path=preview&&i===selected?preview:r.path;
        if(this.routes[i]?.path!==path||this.routes[i]?.radius!==r.radius) {
          if(this.routes[i]) this.dispose(this.routes[i]);
          this.routes[i]={...this.tube(path,r.radius),path,radius:r.radius};
        }
        const hex=preview&&i===selected?"#ffbb62":r.color;
        const color=[1,3,5].map(k=>parseInt(hex.slice(k,k+2),16)/255);
        render(this.routes[i],color,1);
      });
      while(this.routes.length>routes.length) this.dispose(this.routes.pop());
      while(this.routeOutlines.length>routes.length){const mesh=this.routeOutlines.pop();if(mesh)this.dispose(mesh);}
    };
    if(!xray) renderRoutes();
    // Per-component back-to-front transparency, with depth preserved for solids.
    // Intersecting translucent components are approximate (not order independent).
    // Preserve accumulated alpha for composition over the white workbench.
    // Using SRC_ALPHA for alpha itself squares it and washes translucent CAD white.
    gl.enable(gl.BLEND); gl.blendFuncSeparate(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA,gl.ONE,gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
    const depth=m=>m.bounds[0].reduce((s,v,i)=>s+((v+m.bounds[1][i])*.5+(m.offset?.[i]||0))*camera.depth[i],0);
    for(const m of models.filter(m=>m.opacity<.999).sort((a,b)=>depth(a)-depth(b)))
      render(m,colors[m.role]||[.69,.58,.76],m.opacity);
    gl.depthMask(true); gl.disable(gl.BLEND);
    if(xray) {gl.clear(gl.DEPTH_BUFFER_BIT);renderRoutes();}
    gl.bindVertexArray(null);
  }
}
window.ModelViewport=ModelViewport;
