(function(){
 var INK='#f5f5f2',INK2='#b9b8ae',LINE='#2f2f2d',C={blue:'#3987e5',orange:'#d95926',aqua:'#199e70',red:'#e66767',grey:'#8f8e84'};
 var D=function(id){return JSON.parse(document.getElementById(id).textContent);};
 var cfg3={displaylogo:false,responsive:true},cfg={displaylogo:false,responsive:true,displayModeBar:false};
 function ax(t,o){var a={title:{text:t,font:{size:12,color:INK2},standoff:6},gridcolor:LINE,zerolinecolor:'#5a5a54',linecolor:LINE,tickfont:{size:11,color:INK2},automargin:true};for(var k in (o||{})){a[k]=o[k];}return a;}
 function base(o){var l={paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{family:"'IBM Plex Sans',system-ui,sans-serif",color:INK2,size:12},margin:{l:56,r:12,t:30,b:40},hoverlabel:{bgcolor:'#141413',bordercolor:'#55554f',font:{color:INK,size:12}},showlegend:true,legend:{orientation:'h',x:0,y:1.02,yanchor:'bottom',font:{color:INK,size:12},bgcolor:'rgba(0,0,0,0)'}};for(var k in (o||{})){l[k]=o[k];}return l;}
 function col(a,i){return a.map(function(r){return r[i];});}
 function vline(x,txt,yref,color){return {s:{type:'line',xref:'x',yref:yref||'paper',x0:x,x1:x,y0:0,y1:1,line:{color:color||'#77766d',width:1,dash:'dot'}},a:{xref:'x',yref:'paper',x:x,y:1,yanchor:'bottom',text:txt,showarrow:false,font:{size:10.5,color:INK2},textangle:-30,xanchor:'left'}};}
 if(typeof Plotly==='undefined'){[].forEach.call(document.querySelectorAll('.plot-missing'),function(e){e.hidden=false;});return;}

 /* ---------- Fig. 1: 3-D trajectories ---------- */
 var d3=D('data-3d'),names={p1:'PP-1',p2:'PP-2',p3:'PP-3'},plots3=[];
 function tr3(P,t,name,color,width,dash,key){return {type:'scatter3d',mode:'lines',x:col(P,0),y:col(P,1),z:col(P,2),customdata:t,name:name,meta:key,line:{color:color,width:width,dash:dash||'solid'},hovertemplate:'t = %{customdata:.1f} s<br>x %{x:.2f}, y %{y:.2f}, z %{z:.2f} m<extra>'+name+'</extra>'};}
 ['p1','p2','p3'].forEach(function(nm){
  var e=d3[nm],tr=[tr3(e.ref,e.t,'planned claw path',C.grey,3,'dash','ref'),tr3(e.base,e.t,'airframe',C.blue,3,null,'base'),tr3(e.claw,e.t,'claw',C.orange,4,null,'claw')];
  if(e.basket){tr.push(tr3(e.basket,e.tb,'basket',C.aqua,5,null,'basket'));}
  if(e.flyaway){tr.push(tr3(e.flyaway,e.tf,'after the link froze',C.red,5,null,'fly'));}
  tr.push({type:'scatter3d',mode:'markers+text',x:[e.pick_goal[0],e.place_goal[0]],y:[e.pick_goal[1],e.place_goal[1]],z:[e.pick_goal[2],e.place_goal[2]],text:['pick','place'],textposition:'top center',textfont:{color:INK,size:11},marker:{size:4,color:INK,symbol:'diamond'},name:'claw targets',meta:'ref',hovertemplate:'%{text} target<br>x %{x:.2f}, y %{y:.2f}, z %{z:.2f} m<extra></extra>'});
  [[e.capture[0],e.capture[1],e.capture[2]-0.03],[e.place_goal[0]-0.02,e.place_goal[1],e.place_goal[2]-0.16-0.03]].forEach(function(p){tr.push({type:'scatter3d',mode:'lines',x:[p[0],p[0]],y:[p[1],p[1]],z:[0,p[2]],line:{color:'#6b6a62',width:8},hoverinfo:'skip',showlegend:false,meta:'pillar'});});
  var sc={xaxis:ax('x [m]',{backgroundcolor:'rgba(0,0,0,0)',showbackground:false}),yaxis:ax('y [m]',{showbackground:false}),zaxis:ax('z [m]',{showbackground:false,range:[0,1.6]}),aspectmode:'data',camera:{eye:{x:-1.3,y:-1.65,z:1.0}}};
  var el=document.getElementById('f3d-'+nm);
  Plotly.newPlot(el,tr,base({scene:sc,showlegend:false,margin:{l:0,r:0,t:0,b:0}}),cfg3);plots3.push(el);
 });
 [].forEach.call(document.querySelectorAll('#f3d-tog button'),function(b){b.addEventListener('click',function(){
   var on=b.getAttribute('aria-pressed')!=='true';b.setAttribute('aria-pressed',on?'true':'false');var key=b.dataset.k;
   plots3.forEach(function(el){var idx=[];el.data.forEach(function(t,i){if(t.meta===key){idx.push(i);}});if(idx.length){Plotly.restyle(el,{visible:on},idx);}});});});
 document.getElementById('f3d-reset').addEventListener('click',function(){plots3.forEach(function(el){Plotly.relayout(el,{'scene.camera':{eye:{x:-1.3,y:-1.65,z:1.0},up:{x:0,y:0,z:1}}});});});
 document.getElementById('f3d-top').addEventListener('click',function(){plots3.forEach(function(el){Plotly.relayout(el,{'scene.camera':{eye:{x:0,y:-0.001,z:2.4},up:{x:0,y:1,z:0}}});});});

 /* ---------- Fig. 2: tracking errors over time ---------- */
 var de=D('data-err'),elE=document.getElementById('ferr');
 function errPlot(nm){
  var e=de[nm],t=e.t,tr=[],rows=5,gap=0.035,h=(1-gap*(rows-1))/rows,lay=base({hovermode:'x unified',hoversubplots:'axis',margin:{l:58,r:10,t:86,b:40},showlegend:true});
  function dom(i){var top=1-i*(h+gap);return [top-h,top];}
  function add(row,y,name,color,dash){tr.push({type:'scatter',mode:'lines',x:t,y:y,name:name,xaxis:'x',yaxis:'y'+(row?row+1:''),legend:'legend'+(row?row+1:''),line:{color:color,width:1.6,dash:dash||'solid'},hovertemplate:'%{y:.1f}<extra>'+name+'</extra>'});}
  ['x','y','z'].forEach(function(c,i){add(0,col(e.e_com,i),'airframe '+c,[C.blue,C.orange,C.aqua][i]);});
  ['x','y','z'].forEach(function(c,i){add(1,col(e.e_task,i),'claw vs airframe '+c,[C.blue,C.orange,C.aqua][i]);});
  add(2,e.e_head,'claw heading error',C.blue);add(2,e.e_yaw,'airframe yaw error',C.orange);add(2,e.tilt,'tilt',C.aqua);
  add(3,e.u1,'collective thrust',C.blue);
  add(4,col(e.tau,0),'joint 2 torque',C.blue);add(4,col(e.tau,1),'joint 3 torque',C.orange);
  var titles=['airframe error [mm]','claw vs airframe [mm]','angles [deg]','thrust [N]','arm torque [N·m]'];
  for(var i=0;i<rows;i++){var k=i?i+1:'';lay['yaxis'+k]=ax(titles[i],{domain:dom(i),anchor:'x'});lay['legend'+k]={orientation:'h',x:1,xanchor:'right',y:dom(i)[1],yanchor:'bottom',font:{size:11,color:INK},bgcolor:'rgba(0,0,0,0.55)'};}
  lay.xaxis=ax('time in the bag [s]',{anchor:'y5',range:[t[0],t[t.length-1]]});
  lay.shapes=[];lay.annotations=[];var span=t[t.length-1]-t[0];
  e.phases.forEach(function(p,i){lay.shapes.push({type:'rect',xref:'x',yref:'paper',x0:p[0],x1:p[1],y0:0,y1:1,fillcolor:'#ffffff',opacity:(p[2]==='wait')?0.0:0.055,line:{width:0},layer:'below'});
    if((p[1]-p[0])/span>0.018){lay.annotations.push({xref:'x',yref:'paper',x:(p[0]+p[1])/2,y:1.0,yanchor:'bottom',yshift:26,text:p[2],showarrow:false,font:{size:10.5,color:INK2},textangle:-35});}});
  Plotly.react(elE,tr,lay,cfg);
 }
 errPlot('p1');
 [].forEach.call(document.querySelectorAll('#ferr-pick button'),function(b){b.addEventListener('click',function(){
   [].forEach.call(document.querySelectorAll('#ferr-pick button'),function(o){o.setAttribute('aria-pressed',o===b?'true':'false');});errPlot(b.dataset.k);});});

 /* ---------- Fig. 3: claw hover error, without and with the basket ---------- */
 (function(){
  var dh=D('data-hover'),tr=[],sh=[],an=[],K=[['none','without payload',C.blue,'x','y',0.24],['payload','with the basket',C.orange,'x2','y2',0.76]];
  K.forEach(function(k){
   var x=[],y=[],cd=[],n=0;dh.pts[k[0]].forEach(function(w){n++;w.xy.forEach(function(p,i){x.push(p[0]);y.push(p[1]);cd.push(w.run+', '+w.t[i].toFixed(1)+' s');});});
   tr.push({type:'scatter',mode:'markers',x:x,y:y,customdata:cd,name:k[1],marker:{size:4.5,color:k[2],opacity:0.5},xaxis:k[3],yaxis:k[4],showlegend:false,hovertemplate:'%{x:.0f}, %{y:.0f} mm<br>%{customdata}<extra>'+k[1]+'</extra>'});
   [20,50].forEach(function(r){sh.push({type:'circle',xref:k[3],yref:k[4],x0:-r,x1:r,y0:-r,y1:r,line:{color:'#8f8e84',width:1,dash:'dot'}});
     an.push({xref:k[3],yref:k[4],x:0,y:r,text:r+' mm',showarrow:false,yanchor:'bottom',font:{size:10.5,color:INK2}});});
   var q=dh.pooled[k[0]];sh.push({type:'circle',xref:k[3],yref:k[4],x0:-q.p95,x1:q.p95,y0:-q.p95,y1:q.p95,line:{color:k[2],width:1.6}});
   an.push({xref:k[3],yref:k[4],x:0,y:-q.p95,text:'95 % of the time inside '+q.p95.toFixed(0)+' mm',showarrow:false,yanchor:'top',yshift:-3,font:{size:11,color:INK},bgcolor:'rgba(0,0,0,0.7)'});
   an.push({xref:'paper',yref:'paper',x:k[5],y:1,yanchor:'bottom',xanchor:'center',text:'<b>'+(k[0]==='none'?'Without payload':'With the basket')+'</b>: '+q.seconds.toFixed(0)+' s in '+q.windows+' hovers, '+q.rms.toFixed(0)+' mm rms',showarrow:false,font:{size:12.5,color:INK}});
  });
  Plotly.newPlot('fhover',tr,base({margin:{l:52,r:10,t:34,b:42},showlegend:false,hovermode:'closest',
   xaxis:ax('error along world x [mm]',{domain:[0,0.48],range:[-105,105],anchor:'y'}),yaxis:ax('error along world y [mm]',{range:[-105,105],anchor:'x',scaleanchor:'x',scaleratio:1}),
   xaxis2:ax('error along world x [mm]',{domain:[0.52,1],range:[-105,105],anchor:'y2',matches:'x'}),yaxis2:ax('',{range:[-105,105],anchor:'x2',scaleanchor:'x2',scaleratio:1,matches:'y',showticklabels:false}),shapes:sh,annotations:an}),cfg);
 })();

 /* ---------- Fig. 4: what the vehicle carried ---------- */
 var dld=D('data-load'),elL=document.getElementById('fload');
 function loadPlot(nm){
  var e=dld[nm],t=e.t,tr=[],rows=3,gap=0.05,h=(1-gap*(rows-1))/rows,lay=base({hovermode:'x unified',hoversubplots:'axis',margin:{l:58,r:10,t:40,b:40},showlegend:true});
  function dom(i){var top=1-i*(h+gap);return [top-h,top];}
  function add(row,y,name,color,fmt){tr.push({type:'scatter',mode:'lines',x:t,y:y,name:name,xaxis:'x',yaxis:'y'+(row?row+1:''),legend:'legend'+(row?row+1:''),line:{color:color,width:1.8},hovertemplate:'%{y:'+fmt+'}<extra>'+name+'</extra>'});}
  add(0,e.u1,'thrust the controller commands, in its own newtons',C.blue,'.2f');add(0,e.T_fit,'thrust delivered (motor commands and battery voltage)',C.orange,'.2f');
  add(1,e.P,'battery power',C.blue,'.0f');add(2,e.V,'battery voltage',C.blue,'.2f');
  var titles=['thrust [N]','power [W]','voltage [V]'];
  for(var i=0;i<rows;i++){var k=i?i+1:'';lay['yaxis'+k]=ax(titles[i],{domain:dom(i),anchor:'x'});lay['legend'+k]={orientation:'h',x:0,xanchor:'left',y:dom(i)[1],yanchor:'bottom',font:{size:11,color:INK},bgcolor:'rgba(0,0,0,0.55)'};}
  lay.yaxis.range=[33.5,42.8];lay.yaxis2.range=[440,530];lay.yaxis3.range=[22.9,24.0];
  lay.xaxis=ax('time in the bag [s]',{anchor:'y3',range:[t[0],t[t.length-1]]});lay.shapes=[];lay.annotations=[];
  if(e.load){lay.shapes.push({type:'rect',xref:'x',yref:'paper',x0:e.load[0],x1:e.load[1],y0:0,y1:1,fillcolor:C.aqua,opacity:0.13,line:{width:0},layer:'below'});
   lay.annotations.push({xref:'x',yref:'paper',x:(e.load[0]+e.load[1])/2,y:dom(0)[0],yanchor:'bottom',text:'basket on the claw',showarrow:false,font:{size:11.5,color:INK}});}
  Plotly.react(elL,tr,lay,cfg);
 }
 loadPlot('p2');
 [].forEach.call(document.querySelectorAll('#fload-pick button'),function(b){b.addEventListener('click',function(){
   [].forEach.call(document.querySelectorAll('#fload-pick button'),function(o){o.setAttribute('aria-pressed',o===b?'true':'false');});loadPlot(b.dataset.k);});});

 /* ---------- Fig. 5 and 6: the pick and the place from above and from the side ---------- */
 var dv=D('data-views'),SC={ready:C.blue,pick:C.orange,place:C.orange,exit:C.aqua};
 var LAB={pick:{ready:'Ready To Pick',pick:'Pick',exit:'Exit To Pick'},place:{ready:'Ready To Place',place:'Place',exit:'Exit To Place'}};
 var EVT={close:['gripper closes','circle'],liftoff:['basket lifts off','diamond'],touchdown:['basket touches down','diamond'],open:['gripper opens','square']};
 var HAT=100.3,BX=55,BY=57.5,BH=65,STEM=-27.5,ARCH=233.4;
 function viewPlot(task,nm){
  var e=dv[nm][task],t=e.t,top=[],side=[],shT=[],shS=[],anT=[],anS=[];
  function seg(P,a,b){var o={x:[],y:[],z:[],t:[]};for(var i=0;i<t.length;i++){if(t[i]>=a&&t[i]<=b){o.x.push(P[i][0]);o.y.push(P[i][1]);o.z.push(P[i][2]);o.t.push(t[i]);}}return o;}
  function ln(x,y,tt,name,color,w,dash,ht){return {type:'scatter',mode:'lines',x:x,y:y,customdata:tt,name:name,line:{color:color,width:w,dash:dash||'solid'},showlegend:false,hovertemplate:'t = %{customdata:.1f} s<br>'+ht+'<extra>'+name+'</extra>'};}
  var HT='along %{x:.0f} mm, across %{y:.0f} mm',HS='along %{x:.0f} mm, height %{y:.0f} mm';
  var r=seg(e.ref,t[0],t[t.length-1]);top.push(ln(r.x,r.y,r.t,'planned claw path',C.grey,1.4,'dash',HT));side.push(ln(r.x,r.z,r.t,'planned claw path',C.grey,1.4,'dash',HS));
  e.stages.forEach(function(s){var c=SC[s[0]],n=LAB[task][s[0]],a=seg(e.claw,s[1],s[2]);
   if(e.basket){var b=seg(e.basket,s[1],s[2]);top.push(ln(b.x,b.y,b.t,'basket, '+n,c,2.2,'dot',HT));side.push(ln(b.x,b.z,b.t,'basket, '+n,c,2.2,'dot',HS));}
   top.push(ln(a.x,a.y,a.t,'claw, '+n,c,3.2,null,HT));side.push(ln(a.x,a.z,a.t,'claw, '+n,c,3.2,null,HS));});
  function imax(a){var k=0;for(var i=1;i<a.length;i++){if(a[i]>a[k]){k=i;}}return k;}
  var xs=e.stages[2],cx=seg(e.claw,xs[1],xs[2]),kc=imax(cx.z),kr=imax(r.z),LB={size:11.5,color:INK},BG='rgba(0,0,0,0.65)',pk=task==='pick';
  anS.push({x:cx.x[kc],y:cx.z[kc],text:'claw',showarrow:false,xanchor:pk?'center':'left',yanchor:'bottom',xshift:pk?0:8,yshift:5,font:LB,bgcolor:BG});
  if(pk){anS.push({x:r.x[kr],y:r.z[kr],text:'planned claw path: straight up',showarrow:false,xanchor:'right',yanchor:'bottom',xshift:-6,font:{size:11,color:INK2},bgcolor:BG});}
  if(e.basket){var bx=seg(e.basket,xs[1],xs[2]),kb=imax(bx.x);
   anS.push({x:bx.x[kb],y:bx.z[kb],text:'basket centre',showarrow:false,xanchor:pk?'right':'left',yanchor:'top',xshift:pk?0:8,yshift:-6,font:LB,bgcolor:BG});}
  if(!pk){var pl=seg(e.ref,e.stages[1][1],e.stages[1][2]),sd=pl.x[pl.x.length-1],sx=r.x[r.x.length-1],gg=e.goal,AR={showarrow:true,arrowcolor:INK2,arrowwidth:1,arrowhead:2,arrowsize:0.9,font:{size:11,color:INK},bgcolor:BG};
   shS.push({type:'line',x0:gg[0],x1:gg[0],y0:gg[2],y1:gg[2]+200,line:{color:INK,width:1.4,dash:'dot'},layer:'below'});
   var a1={x:gg[0],y:gg[2]+150,text:'descent without the trim',ax:95,ay:-28},a2={x:sd,y:gg[2]+95,text:'planned descent, trimmed '+Math.abs(sd-gg[0]).toFixed(0)+' mm',ax:112,ay:-14},a3={x:sx,y:gg[2]+60,text:'planned exit: back out, then up',ax:-80,ay:-86};
   [a1,a2,a3].forEach(function(a){for(var k in AR){a[k]=AR[k];}anS.push(a);});}
  function at(tq){var i=0,best=0,dm=1e9;for(i=0;i<t.length;i++){var d=Math.abs(t[i]-tq);if(d<dm){dm=d;best=i;}}return best;}
  var OFF={close:[-70,-34],liftoff:[60,-40],touchdown:[-75,36],open:[78,-62]};
  Object.keys(EVT).forEach(function(k){var tv=e.ev[k];if(tv===null||tv===undefined){return;}var i=at(tv),p=e.claw[i],m={size:11,symbol:EVT[k][1],color:INK,line:{color:'#000',width:2}};
   top.push({type:'scatter',mode:'markers',x:[p[0]],y:[p[1]],marker:m,showlegend:false,hovertemplate:EVT[k][0]+', t = '+tv.toFixed(1)+' s<br>claw along %{x:.0f}, across %{y:.0f} mm<extra></extra>'});
   side.push({type:'scatter',mode:'markers',x:[p[0]],y:[p[2]],marker:m,showlegend:false,hovertemplate:EVT[k][0]+', t = '+tv.toFixed(1)+' s<br>claw along %{x:.0f}, height %{y:.0f} mm<extra></extra>'});
   var o=OFF[k];anT.push({x:p[0],y:p[1],text:EVT[k][0],showarrow:true,arrowcolor:INK2,arrowwidth:1,arrowhead:0,ax:o[0],ay:o[1],font:{size:11,color:INK},bgcolor:'rgba(0,0,0,0.6)'});
   anS.push({x:p[0],y:p[2],text:EVT[k][0],showarrow:true,arrowcolor:INK2,arrowwidth:1,arrowhead:0,ax:o[0],ay:o[1],font:{size:11,color:INK},bgcolor:'rgba(0,0,0,0.6)'});
   if(e.basket){var q=e.basket[i],mb={size:9,symbol:EVT[k][1]+'-open',color:INK,line:{color:INK,width:2}};
    top.push({type:'scatter',mode:'markers',x:[q[0]],y:[q[1]],marker:mb,showlegend:false,hovertemplate:EVT[k][0]+'<br>basket along %{x:.0f}, across %{y:.0f} mm<extra></extra>'});
    side.push({type:'scatter',mode:'markers',x:[q[0]],y:[q[2]],marker:mb,showlegend:false,hovertemplate:EVT[k][0]+'<br>basket along %{x:.0f}, height %{y:.0f} mm<extra></extra>'});}});
  var g=e.goal,mx={size:12,symbol:'x-thin',line:{color:INK,width:2.5}};
  top.push({type:'scatter',mode:'markers',x:[g[0]],y:[g[1]],marker:mx,showlegend:false,hovertemplate:'claw target: along %{x:.0f}, across %{y:.0f} mm<extra></extra>'});
  side.push({type:'scatter',mode:'markers',x:[g[0]],y:[g[2]],marker:mx,showlegend:false,hovertemplate:'claw target: along %{x:.0f}, height %{y:.0f} mm<extra></extra>'});
  var c0=task==='pick'&&e.rest?e.rest:[0,0,32.5],dsh=task==='pick'?'solid':'dash',fill=task==='pick'?'rgba(255,255,255,0.07)':'rgba(255,255,255,0.02)';
  shT.push({type:'circle',x0:-HAT,x1:HAT,y0:-HAT,y1:HAT,line:{color:'#8f8e84',width:1.6},fillcolor:'rgba(255,255,255,0.04)',layer:'below'});
  shT.push({type:'rect',x0:c0[0]-BX,x1:c0[0]+BX,y0:c0[1]-BY,y1:c0[1]+BY,line:{color:INK2,width:1.2,dash:dsh},fillcolor:fill,layer:'below'});
  shT.push({type:'line',x0:c0[0]+STEM,x1:c0[0]+STEM,y0:c0[1]-40,y1:c0[1]+40,line:{color:INK2,width:3,dash:dsh},layer:'below'});
  anT.push({x:0,y:HAT,text:'hat, radius 100 mm',showarrow:false,yanchor:'bottom',font:{size:11,color:INK2}});
  anT.push({x:c0[0],y:c0[1]-BY,text:task==='pick'?'basket at rest':'basket on the place point',showarrow:false,xanchor:'center',yanchor:'top',yshift:-2,font:{size:11,color:INK2}});
  shS.push({type:'rect',x0:-HAT,x1:HAT,y0:-5,y1:0,line:{width:0},fillcolor:'#8f8e84',layer:'below'});
  shS.push({type:'rect',x0:-61,x1:61,y0:-57,y1:-5,line:{width:0},fillcolor:'#55554f',layer:'below'});
  shS.push({type:'rect',x0:c0[0]-BX,x1:c0[0]+BX,y0:0,y1:BH,line:{color:INK2,width:1.2,dash:dsh},fillcolor:fill,layer:'below'});
  shS.push({type:'line',x0:c0[0]+STEM,x1:c0[0]+STEM,y0:BH,y1:ARCH,line:{color:INK2,width:2,dash:dsh},layer:'below'});
  shS.push({type:'line',x0:c0[0]+STEM-9,x1:c0[0]+STEM+9,y0:ARCH,y1:ARCH,line:{color:INK2,width:4},layer:'below'});
  anS.push({x:c0[0]+STEM+9,y:ARCH,text:'arch',showarrow:false,xanchor:'left',xshift:4,font:{size:11,color:INK2}});
  anS.push({x:-HAT,y:-5,text:'hat top, z = '+e.z_world0.toFixed(3)+' m',showarrow:false,xanchor:'right',yanchor:'top',xshift:-6,font:{size:11,color:INK2}});
  if(task==='pick'){shS.push({type:'line',x0:-340,x1:290,y0:195.5,y1:195.5,line:{color:'#c98500',width:1.2,dash:'dot'},layer:'below'});
}
  var world=task==='pick'?'world +x':'world −x',wy=task==='pick'?'world +y':'world −y';
  Plotly.react('fv-'+task+'-top',top,base({margin:{l:56,r:10,t:16,b:44},showlegend:false,hovermode:'closest',shapes:shT,annotations:anT,
   xaxis:ax('along the approach ('+world+') [mm]',{range:[-340,290]}),yaxis:ax('across ('+wy+') [mm]',{range:[-190,190],scaleanchor:'x',scaleratio:1})}),cfg);
  Plotly.react('fv-'+task+'-side',side,base({margin:{l:56,r:10,t:16,b:44},showlegend:false,hovermode:'closest',shapes:shS,annotations:anS,
   xaxis:ax('along the approach ('+world+') [mm]',{range:[-340,290]}),yaxis:ax('height above the hat [mm]',{range:[-70,450],scaleanchor:'x',scaleratio:1})}),cfg);
 }
 ['pick','place'].forEach(function(task){viewPlot(task,'p2');
  [].forEach.call(document.querySelectorAll('#fv-'+task+'-run button'),function(b){b.addEventListener('click',function(){
    [].forEach.call(document.querySelectorAll('#fv-'+task+'-run button'),function(o){o.setAttribute('aria-pressed',o===b?'true':'false');});viewPlot(task,b.dataset.k);});});});

 /* ---------- Fig. 7: the link freeze in flight 2 ---------- */
 var df=D('data-freeze'),tf=df.t,px=col(df.pos,0),py=col(df.pos,1),pz=col(df.pos,2),mk=[],i;
 for(i=0;i<tf.length;i+=25){mk.push(i);}
 Plotly.newPlot('ffreeze-map',[
   {type:'scatter',mode:'lines',x:px,y:py,customdata:tf,name:'airframe (mocap)',line:{color:C.blue,width:2.5},hovertemplate:'t = %{customdata:.2f} s<br>x %{x:.2f}, y %{y:.2f} m<extra></extra>'},
   {type:'scatter',mode:'markers+text',x:mk.map(function(i){return px[i];}),y:mk.map(function(i){return py[i];}),text:mk.map(function(i){return tf[i].toFixed(1);}),textposition:'top right',textfont:{size:10,color:INK2},marker:{size:6,color:C.blue,line:{color:'#000',width:1.5}},name:'every 0.5 s',hoverinfo:'skip'},
   {type:'scatter',mode:'markers+text',x:[1.0,-1.03],y:[1.02,-1.03],text:['pick pillar','place pillar'],textposition:'bottom center',textfont:{size:11,color:INK2},marker:{size:13,color:'#6b6a62',symbol:'circle'},name:'pillars',hoverinfo:'skip',showlegend:false}],
  base({margin:{l:52,r:10,t:34,b:42},xaxis:ax('x [m]',{range:[-3.1,1.6]}),yaxis:ax('y [m]',{range:[-2.6,1.6],scaleanchor:'x',scaleratio:1}),
   shapes:[{type:'rect',xref:'x',yref:'y',x0:-2.25,x1:2.25,y0:-2.1,y1:2.1,line:{color:C.grey,width:1.2,dash:'dash'}}],
   annotations:[{xref:'x',yref:'y',x:-2.2,y:1.45,text:'field 4.5 × 4.2 m',showarrow:false,xanchor:'left',font:{size:11,color:INK2}}],hovermode:'closest'}),cfg);
 var sh3=[],an3=[];[[df.ev.freeze,'Pixhawk stream stops'],[df.ev.revert,'node reverts to SAFETY'],[df.ev.touchdown,'touchdown']].forEach(function(e){var v=vline(e[0],e[1],null,C.red);sh3.push(v.s);an3.push(v.a);});
 Plotly.newPlot('ffreeze-ts',[
   {type:'scatter',mode:'lines',x:tf,y:df.roll,name:'roll',line:{color:C.blue,width:2},hovertemplate:'%{y:.1f}°<extra>roll</extra>'},
   {type:'scatter',mode:'lines',x:tf,y:df.pitch,name:'pitch',line:{color:C.orange,width:2},hovertemplate:'%{y:.1f}°<extra>pitch</extra>'},
   {type:'scatter',mode:'lines',x:tf,y:df.yaw.map(function(v){return v-df.yaw[0];}),name:'yaw change',line:{color:C.aqua,width:2},hovertemplate:'%{y:.1f}°<extra>yaw change</extra>'},
   {type:'scatter',mode:'lines',x:tf,y:df.speed,name:'speed',line:{color:C.blue,width:2},yaxis:'y2',showlegend:false,hovertemplate:'%{y:.2f} m/s<extra>speed</extra>'},
   {type:'scatter',mode:'lines',x:tf,y:pz,name:'height',line:{color:C.blue,width:2},yaxis:'y3',showlegend:false,hovertemplate:'%{y:.2f} m<extra>height</extra>'}],
  base({hovermode:'x unified',hoversubplots:'axis',margin:{l:56,r:10,t:84,b:42},legend:{orientation:'h',x:0,xanchor:'left',y:0.56,yanchor:'bottom',font:{color:INK,size:11},bgcolor:'rgba(0,0,0,0.55)'},
   xaxis:ax('time in the bag [s]',{anchor:'y3'}),yaxis:ax('attitude [deg]',{domain:[0.56,1],range:[-60,20]}),yaxis2:ax('speed [m/s]',{domain:[0.29,0.50]}),yaxis3:ax('height [m]',{domain:[0,0.22]}),shapes:sh3,annotations:an3}),cfg);

 /* ---------- Fig. 8: the two manual touchdowns ---------- */
 var dn=D('data-land');
 [['p1','fland-p1',[[1.94,'planner abort']]],['p3','fland-p3',[]]].forEach(function(q){var e=dn[q[0]],sh=[],an=[];
  [[e.t_stab,'pilot takes STAB'],[e.t_disarm,'disarmed']].concat(q[2]).forEach(function(v,i){var l=vline(v[0],v[1]);if(i===2){l.a.yshift=26;}sh.push(l.s);an.push(l.a);});
  Plotly.newPlot(q[1],[
   {type:'scatter',mode:'lines',x:e.t,y:e.z,name:'height',line:{color:C.blue,width:2},showlegend:false,hovertemplate:'%{y:.2f} m<extra>height</extra>'},
   {type:'scatter',mode:'lines',x:e.t,y:e.pitch,name:'pitch (+ nose down)',line:{color:C.blue,width:2},yaxis:'y2',legend:'legend',hovertemplate:'%{y:.0f}°<extra>pitch</extra>'},
   {type:'scatter',mode:'lines',x:e.t,y:e.roll,name:'roll',line:{color:C.orange,width:2},yaxis:'y2',legend:'legend',hovertemplate:'%{y:.0f}°<extra>roll</extra>'},
   {type:'scatter',mode:'lines',x:e.t,y:e.tau2,name:'joint 2',line:{color:C.blue,width:2},yaxis:'y3',legend:'legend2',hovertemplate:'%{y:.2f} N·m<extra>joint 2</extra>'},
   {type:'scatter',mode:'lines',x:e.t,y:e.tau3,name:'joint 3',line:{color:C.orange,width:2},yaxis:'y3',legend:'legend2',hovertemplate:'%{y:.2f} N·m<extra>joint 3</extra>'}],
  base({hovermode:'x unified',hoversubplots:'axis',margin:{l:56,r:10,t:88,b:42},
   legend:{orientation:'h',x:1,xanchor:'right',y:0.68,yanchor:'bottom',font:{color:INK,size:11},bgcolor:'rgba(0,0,0,0.55)'},legend2:{orientation:'h',x:1,xanchor:'right',y:0.30,yanchor:'bottom',font:{color:INK,size:11},bgcolor:'rgba(0,0,0,0.55)'},
   xaxis:ax('seconds from '+e.t0.toFixed(1)+' s in the bag',{anchor:'y3'}),yaxis:ax('height [m]',{domain:[0.76,1]}),yaxis2:ax('attitude [deg]',{domain:[0.38,0.68]}),yaxis3:ax('applied arm torque [N·m]',{domain:[0,0.30]}),shapes:sh,annotations:an}),cfg);});
})();
