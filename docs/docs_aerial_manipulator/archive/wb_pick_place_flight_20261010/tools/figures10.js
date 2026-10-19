(function(){
 /* Figures of the 2026-10-10 flight (PP-4): the descent trim, the set-down, the carry. Same palette and helpers as figures.js. */
 var INK='#f5f5f2',INK2='#b9b8ae',LINE='#2f2f2d',C={blue:'#3987e5',orange:'#d95926',aqua:'#199e70',red:'#e66767',grey:'#8f8e84',amber:'#c98500'};
 var el=document.getElementById('data-p4');if(!el||typeof Plotly==='undefined'){return;}
 var P=JSON.parse(el.textContent),cfg={displaylogo:false,responsive:true,displayModeBar:false};
 function ax(t,o){var a={title:{text:t,font:{size:12,color:INK2},standoff:6},gridcolor:LINE,zerolinecolor:'#5a5a54',linecolor:LINE,tickfont:{size:11,color:INK2},automargin:true};for(var k in (o||{})){a[k]=o[k];}return a;}
 function base(o){var l={paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{family:"'IBM Plex Sans',system-ui,sans-serif",color:INK2,size:12},margin:{l:58,r:12,t:30,b:40},hoverlabel:{bgcolor:'#141413',bordercolor:'#55554f',font:{color:INK,size:12}},showlegend:true,hovermode:'x unified',hoversubplots:'axis'};for(var k in (o||{})){l[k]=o[k];}return l;}
 function col(a,i){return a.map(function(r){return r[i];});}
 /* rows: [{title, traces:[{y,name,color,dash,fmt,width}], range, lines:[{y,text,color}]}], events: [[t,label]] */
 function rowsPlot(id,t,rows,events,opt){
  opt=opt||{};var box=document.getElementById(id),narrow=box&&box.clientWidth<640;
  var n=rows.length,gap=(opt.gap||0.07)+(narrow?0.035:0),h=(1-gap*(n-1))/n,tr=[],lay=base({margin:{l:narrow?46:58,r:narrow?6:12,t:narrow?74:(opt.top||92),b:40}});lay.shapes=[];lay.annotations=[];
  function dom(i){var top=1-i*(h+gap);return [top-h,top];}
  rows.forEach(function(r,i){var k=i?i+1:'';
   r.traces.forEach(function(s){tr.push({type:'scatter',mode:'lines',x:t,y:s.y,name:s.name,xaxis:'x',yaxis:'y'+k,legend:'legend'+k,line:{color:s.color,width:s.width||2,dash:s.dash||'solid'},hovertemplate:'%{y:'+(s.fmt||'.0f')+'}<extra>'+s.name+'</extra>'});});
   lay['yaxis'+k]=ax(r.title,{domain:dom(i),anchor:'x',range:r.range||undefined});
   lay['legend'+k]={orientation:'h',x:0,xanchor:'left',y:dom(i)[1],yanchor:'bottom',font:{size:11,color:INK},bgcolor:'rgba(0,0,0,0.55)'};
   (r.lines||[]).forEach(function(L){lay.shapes.push({type:'line',xref:'paper',yref:'y'+k,x0:0,x1:1,y0:L.y,y1:L.y,line:{color:L.color||C.amber,width:1.2,dash:'dot'},layer:'below'});
    if(L.text){lay.annotations.push({xref:'paper',yref:'y'+k,x:L.left?0:1,y:L.y,xanchor:L.left?'left':'right',xshift:L.left?4:-2,yanchor:L.below?'top':'bottom',text:L.text,showarrow:false,font:{size:11,color:INK2},bgcolor:'rgba(0,0,0,0.6)'});}});
   (r.bands||[]).forEach(function(B){lay.shapes.push({type:'rect',xref:'x',yref:'y'+k+' domain',x0:B.x0,x1:B.x1,y0:0,y1:1,fillcolor:B.color||'#ffffff',opacity:B.opacity||0.07,line:{width:0},layer:'below'});
    if(B.text){lay.annotations.push({xref:'x',yref:'y'+k+' domain',x:(B.x0+B.x1)/2,y:B.ty===undefined?0.02:B.ty,yanchor:'bottom',text:B.text,showarrow:false,font:{size:11,color:INK2}});}});
  });
  lay.xaxis=ax('time in the bag [s]',{anchor:'y'+(n>1?n:''),range:[t[0],t[t.length-1]]});
  (events||[]).forEach(function(e,i){lay.shapes.push({type:'line',xref:'x',yref:'paper',x0:e[0],x1:e[0],y0:0,y1:1,line:{color:'#77766d',width:1,dash:'dot'}});
   var late=(e[0]-t[0])>0.86*(t[t.length-1]-t[0]);
   if(narrow){lay.annotations.push({xref:'x',yref:'paper',x:e[0],y:1,yanchor:'bottom',xanchor:'center',yshift:50,text:String(i+1),showarrow:false,font:{size:11,color:INK},bgcolor:'#2f2f2d',borderpad:2});}
   else{lay.annotations.push({xref:'x',yref:'paper',x:e[0],y:1,yanchor:'bottom',xanchor:late?'right':'left',yshift:(e[2]||0)+(opt.legendGap||22),text:e[1],showarrow:false,font:{size:10.5,color:INK2},textangle:late?0:-30});}});
  Plotly.newPlot(id,tr,lay,cfg);
 }

 /* ---------- the descent trim at the place ---------- */
 if(document.getElementById('ftrim')){var T=P.trim;
  rowsPlot('ftrim',T.t,[
   {title:'along the approach [mm]',range:[-110,110],traces:[{y:T.claw,name:'claw',color:C.orange,width:2.4},{y:T.ref,name:'planned claw path',color:C.grey,dash:'dash',width:1.6}],
    lines:[{y:0,text:'typed target',color:'#8f8e84',left:true,below:true},{y:T.mean5,text:'5 s average: '+T.mean5.toFixed(0)+' mm beyond'},{y:T.settled,text:'last 2 s: '+T.settled.toFixed(0)+' mm',below:true}],
    bands:[{x0:T.window[0],x1:T.window[1],text:'averaged by the planner',ty:0.02}]},
   {title:'across [mm]',range:[-70,70],traces:[{y:T.claw_across,name:'claw',color:C.orange,width:2.4},{y:T.ref_across,name:'planned claw path',color:C.grey,dash:'dash',width:1.6}],
    lines:[{y:0,text:'typed target',color:'#8f8e84',left:true,below:true}],bands:[{x0:T.window[0],x1:T.window[1],text:''}]}],
   [[T.ev.arrive,'claw arrives'],[T.ev.press,'Place pressed'],[T.ev.touchdown,'touchdown']],{top:84});}

 /* ---------- the set-down ---------- */
 if(document.getElementById('fsetdown')){var S=P.setdown,e=S.ev;
  rowsPlot('fsetdown',S.t,[
   {title:'height above the hat [mm]',range:[-20,330],traces:[{y:S.claw_h,name:'claw',color:C.orange,width:2.4},{y:S.ref_h,name:'planned claw path',color:C.grey,dash:'dash',width:1.6},
     {y:S.arch_h,name:'arch contact height',color:C.amber,width:1.8},{y:S.basket_h,name:"basket's underside",color:C.aqua,width:2}]},
   {title:'moved toward the vehicle [mm]',range:[-30,260],traces:[{y:S.veh_back,name:'airframe',color:C.blue,width:2.4},{y:S.basket_back,name:'basket',color:C.aqua,width:2}],
    lines:[{y:S.rim,text:"hat's rim, "+S.rim.toFixed(0)+' mm away'}]},
   {title:'pitch change [deg]',range:[-7,10],traces:[{y:S.pitch,name:'airframe pitch, negative is nose up',color:C.blue,width:2}],lines:[{y:0,text:'',color:'#5a5a54'}]}],
   [[e.touchdown,'touchdown'],[e.vehicle,'airframe starts back'],[e.rim,'over the rim'],[e.open_cmd,'open command'],[e.open,'fingers open']],{top:92,gap:0.085});}

 /* ---------- the carry: joint 2 sticks and slips ---------- */
 if(document.getElementById('fcarry')){var K=P.carry;
  rowsPlot('fcarry',K.t,[
   {title:'joint 2 angle [deg]',range:[26,48],traces:[{y:K.q2,name:'joint 2',color:C.blue,width:2.2},{y:K.q2_ref,name:'its reference',color:C.grey,dash:'dash',width:1.6}],lines:[{y:K.guard,text:'position guard, 45°',color:C.red}]},
   {title:'joint 2 torque [N·m]',range:[0,2.7],traces:[{y:K.tau2,name:'commanded by the controller',color:C.blue,width:2.2,fmt:'.2f'}],lines:[{y:K.caps[0],text:'cap, 2.44 N·m',color:C.red}]},
   {title:'airframe error [mm]',range:[-220,220],traces:[{y:col(K.e_com,0),name:'x',color:C.blue,width:2},{y:col(K.e_com,1),name:'y',color:C.orange,width:2}]}],
   [[K.ev.arrive,'at the place start'],[K.ev.ready,'Ready To Place']],{top:84});}

 /* ---------- how the basket hangs under the claw, seen from the side: PP-2 (10-09) against PP-4 (10-10) ---------- */
 if(P.hang&&document.getElementById('fhang-'+P.hang.drawn[0])){var H=P.hang,BXh=H.box[0]/2,BZh=H.box[1]/2,SX=H.stem_x;
  var mn=function(v){return (v<0?'−':'+')+Math.abs(v).toFixed(0);};
  /* a point (x forward, z up) of a basket with centre c tilted th degrees, top forward */
  var rot=function(c,th,x,z){var t=th*Math.PI/180;return [c[0]+x*Math.cos(t)+z*Math.sin(t),c[1]-x*Math.sin(t)+z*Math.cos(t)];};
  var basket=function(c,th,h){var o={bx:[],by:[],hx:[],hy:[]};
   [[-BXh,-BZh],[BXh,-BZh],[BXh,BZh],[-BXh,BZh],[-BXh,-BZh]].forEach(function(q){var r=rot(c,th,q[0],q[1]);o.bx.push(r[0]);o.by.push(r[1]);});
   [[SX,BZh],[SX,h],null,[SX-11,h],[SX+11,h]].forEach(function(q){if(!q){o.hx.push(null);o.hy.push(null);return;}var r=rot(c,th,q[0],q[1]);o.hx.push(r[0]);o.hy.push(r[1]);});return o;};
  H.drawn.forEach(function(key){
   var a=H.runs.p2.stages[key],b=H.runs.p4.stages[key],tr=[],an=[],sh=[],YB=-236,p4left=b.along[0]<=a.along[0];
   var ca=[H.assumed.along,H.assumed.up],A=basket(ca,0,H.assumed.contact);
   tr.push({type:'scatter',mode:'lines',x:A.bx.concat([null]).concat(A.hx),y:A.by.concat([null]).concat(A.hy),line:{color:C.grey,width:1.4,dash:'dash'},hoverinfo:'skip',showlegend:false});
   tr.push({type:'scatter',mode:'markers',x:[ca[0]],y:[ca[1]],marker:{size:8,color:'rgba(0,0,0,0)',line:{color:C.grey,width:2}},showlegend:false,
     hovertemplate:'What the planner assumes<br>centre '+ca[0].toFixed(0)+' mm ahead of the claw, '+(-ca[1]).toFixed(0)+' mm below it, level<extra></extra>'});
   sh.push({type:'line',x0:ca[0],x1:ca[0],y0:ca[1],y1:YB,line:{color:C.grey,width:1,dash:'dot'}});
   an.push({x:ca[0],y:YB,text:mn(ca[0]),showarrow:false,yanchor:'top',xanchor:'left',xshift:-3,font:{size:11.5,color:INK2}});
   [[a,H.runs.p2,C.blue,'PP-2, 9 October','rgba(57,135,229,0.16)',p4left?17:0],[b,H.runs.p4,C.orange,'PP-4, 10 October','rgba(217,89,38,0.16)',p4left?0:17]].forEach(function(r){
    var s=r[0],c=[s.along[0],s.up[0]],G=basket(c,s.tilt_side,r[1].contact);
    tr.push({type:'scatter',mode:'lines',x:G.bx,y:G.by,fill:'toself',fillcolor:r[4],line:{color:r[2],width:2},hoverinfo:'skip',showlegend:false});
    tr.push({type:'scatter',mode:'lines',x:G.hx,y:G.hy,line:{color:r[2],width:2},hoverinfo:'skip',showlegend:false});
    tr.push({type:'scatter',mode:'markers',x:[c[0]],y:[c[1]],marker:{size:9,color:r[2],line:{color:'#000',width:2}},showlegend:false,
      hovertemplate:r[3]+'<br>centre '+Math.abs(c[0]).toFixed(0)+' mm '+(c[0]<0?'behind':'ahead of')+' the claw, '+(-c[1]).toFixed(0)+' mm below it<br>tilted '+s.tilt_side.toFixed(0)+'°, top forward<extra></extra>'});
    sh.push({type:'line',x0:c[0],x1:c[0],y0:c[1],y1:YB-r[5],line:{color:r[2],width:1,dash:'dot'}});
    an.push({x:c[0],y:YB-r[5],text:mn(c[0]),showarrow:false,yanchor:'top',xanchor:r[5]?'center':'right',xshift:r[5]?0:3,font:{size:11.5,color:INK}});
   });
   var pd=0.5*(a.claw_pitch+b.claw_pitch)*Math.PI/180,fx=Math.cos(pd),fz=-Math.sin(pd);
   tr.push({type:'scatter',mode:'lines',x:[-52*fx,44*fx],y:[-52*fz,44*fz],line:{color:'#d8d7cf',width:6},hoverinfo:'skip',showlegend:false});
   tr.push({type:'scatter',mode:'markers',x:[0],y:[0],marker:{size:9,color:INK,line:{color:'#000',width:2}},showlegend:false,hovertemplate:"the claw's grasp point<extra></extra>"});
   an.push({x:-52*fx,y:-52*fz,text:'fingers',showarrow:false,xanchor:'right',yanchor:'bottom',xshift:-2,font:{size:11,color:INK2}});
   an.push({xref:'paper',yref:'paper',x:0,y:1,text:'← vehicle',showarrow:false,xanchor:'left',yanchor:'top',font:{size:11,color:INK2}});
   an.push({xref:'paper',yref:'paper',x:1,y:1,text:'ahead →',showarrow:false,xanchor:'right',yanchor:'top',font:{size:11,color:INK2}});
   Plotly.newPlot('fhang-'+key,tr,{paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{family:"'IBM Plex Sans',system-ui,sans-serif",color:INK2,size:12},
     margin:{l:48,r:8,t:8,b:40},showlegend:false,hovermode:'closest',hoverlabel:{bgcolor:'#141413',bordercolor:'#55554f',font:{color:INK,size:12}},shapes:sh,annotations:an,
     xaxis:ax('along the heading, from the claw [mm]',{range:[-118,98],dtick:50}),yaxis:ax('height from the claw [mm]',{range:[-278,48],dtick:50,scaleanchor:'x',scaleratio:1})},cfg);
  });}

 /* ---------- does the hanging basket pivot on its arch, or turn with the claw? (pitch of the fingers against the basket's tilt) ---------- */
 if(P.hang&&P.hang.follow){['p2','p4'].forEach(function(nm){var F=P.hang.follow[nm],id='ffollow-'+nm;if(!document.getElementById(id)){return;}
  rowsPlot(id,F.t,[{title:'angle [deg]',range:[0,34],traces:[
     {y:F.pitch_s,name:'fingers, below the horizontal',color:'#d8d7cf',width:2,fmt:'.1f'},
     {y:F.side_s,name:'basket, top toward the fingertips',color:nm==='p2'?C.blue:C.orange,width:2.4,fmt:'.1f'}],
    lines:[{y:F.free_hang,text:'free hang, '+F.free_hang.toFixed(0)+'°',color:C.grey,left:true,below:true}]}],
   [[F.ev.leave,'arm leaves the pick pose'],[F.ev.jump,'joint 2 jumps '+F.ev.jump_deg.toFixed(0)+'°'],[F.ev.back,'arm to the place pose']],{top:84});});}
})();
