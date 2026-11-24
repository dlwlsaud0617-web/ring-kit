// node render.js <out_base_without_ext> [metal]  → <base>_preview.png (정면/측면/윗면/투시 4컷)
const {chromium}=require('playwright');const http=require('http'),fs=require('fs'),path=require('path');
const base=path.resolve(process.argv[2]); const metal=(process.argv[3]||'').toLowerCase();
const mc=metal.includes('pg')||metal.includes('kp')?'#e8b9a0':metal.includes('yg')?'#e6c66e':'#d4d4d4';
const viewer=path.join(__dirname,'viewer');
const srv=http.createServer((q,s)=>{let u=decodeURIComponent(q.url.split('?')[0]);
 let p=u.startsWith('/__m/')?u.slice(4):path.join(viewer,u);
 if(!fs.existsSync(p)){s.writeHead(404);return s.end()}
 const t=p.endsWith('.js')?'text/javascript':p.endsWith('.json')?'application/json':p.endsWith('.html')?'text/html':'application/octet-stream';
 s.writeHead(200,{'Content-Type':t});fs.createReadStream(p).pipe(s)}).listen(0);
(async()=>{const port=srv.address().port;
 const b=await chromium.launch({args:['--use-gl=swiftshader','--enable-webgl','--ignore-gpu-blocklist']});
 const pg=await b.newPage({viewport:{width:800,height:800}});pg.on('pageerror',e=>console.log('pageerror',e.message));
 const shots=[];
 for(const v of ['front','side','top','persp']){
  await pg.goto(`http://localhost:${port}/view.html?b=${encodeURIComponent('/__m'+base)}&v=${v}&mc=${encodeURIComponent(mc)}`);
  await pg.waitForFunction('window.DONE',null,{timeout:120000});
  shots.push((await (await pg.$('canvas')).screenshot()).toString('base64'));}
 const html=`<body style="margin:0;display:flex">${shots.map(s=>`<img src="data:image/png;base64,${s}" width=400>`).join('')}</body>`;
 await pg.setViewportSize({width:1600,height:400});await pg.setContent(html);
 await pg.screenshot({path:base+'_preview.png'});
 console.log('preview:',base+'_preview.png');await b.close();srv.close()})();
