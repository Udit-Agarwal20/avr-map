import { chromium, expect } from '@playwright/test';
import AxeBuilder from '@axe-core/playwright';
import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import http from 'node:http';
import path from 'node:path';
process.chdir(new URL('..', import.meta.url).pathname);
// Serve the production artifact, including SPA routes, with no external assets.
const root=path.resolve('dist');
const server=http.createServer(async(req,res)=>{
 const requested=path.resolve(root, '.'+new URL(req.url,'http://localhost').pathname);
 if(!requested.startsWith(root+path.sep)&&requested!==root){res.writeHead(403).end();return;}
 let file=requested;
 try { if(!(await fs.stat(file)).isFile()) file=path.join(root,'index.html'); } catch {file=path.join(root,'index.html');}
 const mime={'.html':'text/html','.js':'text/javascript','.css':'text/css','.woff2':'font/woff2','.woff':'font/woff','.svg':'image/svg+xml'};
 res.setHeader('Content-Type',mime[path.extname(file)]??'application/octet-stream');res.end(await fs.readFile(file));
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const base=`http://127.0.0.1:${server.address().port}`;
const browser=await chromium.launch({headless:true});
const context=await browser.newContext({viewport:{width:1920,height:1080},deviceScaleFactor:1,reducedMotion:'reduce'});
const page=await context.newPage();
const errors=[],warnings=[],network=[];
page.on('pageerror',e=>errors.push(String(e)));page.on('console',m=>{if(m.type()==='error')errors.push(m.text());if(m.type()==='warning')warnings.push(m.text());});
page.on('request',r=>network.push(r.url()));
const routes=[['overview','/mission?demo=ppt'],['resolution','/mission?demo=ppt&view=resolution'],['dynamics','/mission?demo=ppt&view=dynamics'],['lidar','/mission?demo=ppt&view=raw'],['semantic','/mission?demo=ppt&view=semantic'],['compare','/compare?demo=ppt'],['benchmarks','/benchmarks'],['diagnostics','/diagnostics'],['config','/config']];
await fs.mkdir('.screenshots',{recursive:true});
const audits=[];
for(const [name,path] of routes){
 await page.goto(`${base}${path}`);await page.evaluate(()=>document.fonts.ready);await page.waitForTimeout(700);
 await page.screenshot({path:`.screenshots/final-freeze-${name}.png`});
 await page.screenshot({path:`.screenshots/astra-final-${name}.png`});
 const axe=await new AxeBuilder({page}).withTags(['wcag2a','wcag2aa','wcag21aa']).analyze();
 audits.push({name,violations:axe.violations.map(v=>({id:v.id,impact:v.impact,nodes:v.nodes.map(n=>({html:n.html,summary:n.failureSummary}))})),overflow:await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth)});
 console.log(name, 'axe',axe.violations.length);
}
await fs.writeFile('.screenshots/audit.json',JSON.stringify({errors,warnings,externalRequests:network.filter(u=>!u.startsWith('http://127.0.0.1')&&!u.startsWith('data:')),audits},null,2));
const checks=[];
async function check(name,fn){try{await fn();checks.push({name,pass:true});console.log('PASS',name);}catch(e){checks.push({name,pass:false,error:String(e)});console.log('FAIL',name,String(e));}}
await check('Root redirects; five primary destinations',async()=>{await page.goto(base+'/');await page.waitForURL('**/mission');assert.equal(await page.getByRole('navigation',{name:'Main Navigation'}).getByRole('link').count(),5);});
await check('Presentation shortcut hides and restores navigation',async()=>{await page.keyboard.press('p');assert.equal(await page.getByRole('navigation',{name:'Main Navigation'}).count(),0);await page.keyboard.press('p');assert.equal(await page.getByRole('navigation',{name:'Main Navigation'}).count(),1);});
await check('Layers keyboard shortcut and Escape',async()=>{await page.keyboard.press('l');assert.equal(await page.getByText('DISPLAY LAYERS',{exact:true}).count(),1);await page.keyboard.press('Escape');assert.equal(await page.getByText('DISPLAY LAYERS',{exact:true}).count(),0);});
await check('Command palette keyboard search and routing',async()=>{await page.keyboard.press('Control+k');await page.getByRole('textbox',{name:'Search commands'}).fill('Go to Benchmarks');await page.keyboard.press('Enter');await page.waitForURL('**/benchmarks');assert.equal(await page.getByRole('dialog').count(),0);});
await check('Command palette traps focus and restores its trigger on Escape',async()=>{await page.goto(base+'/mission');const trigger=page.getByRole('navigation',{name:'Main Navigation'}).getByRole('button',{name:'Open command palette'});await trigger.focus();await page.keyboard.press('Control+k');await page.getByRole('dialog').waitFor();for(let i=0;i<20;i++)await page.keyboard.press('Tab');assert.ok(await page.getByRole('dialog').evaluate(el=>el.contains(document.activeElement)));await page.keyboard.press('Escape');await expect(trigger).toBeFocused();});
await check('Golden cell DOM inspector; Escape clears selection',async()=>{await page.goto(base+'/mission?demo=ppt&view=resolution');await page.getByText('WHY THIS RESOLUTION?',{exact:true}).waitFor();assert.match(await page.getByRole('complementary').innerText(),/10.82 m · XY/);assert.match(await page.getByRole('complementary').innerText(),/NO STATIC-FUSABLE ELEVATION SUPPORT/);await page.keyboard.press('Escape');assert.equal(await page.getByRole('complementary').count(),0);});
await check('All four fixture scenarios render',async()=>{await page.goto(base+'/mission');for(const id of ['A','B','C','D']){await page.getByRole('combobox',{name:'Fixture scenario'}).selectOption(id);assert.equal(await page.getByRole('combobox',{name:'Fixture scenario'}).inputValue(),id);}});
await check('Comparison modes, pan, zoom and accessible wipe',async()=>{await page.goto(base+'/compare');await page.getByRole('button',{name:'WIPE',exact:true}).click();await page.getByRole('slider',{name:'Comparison wipe position',exact:true}).fill('27');assert.equal(await page.getByRole('slider',{name:'Comparison wipe position',exact:true}).inputValue(),'27');const handle=page.getByRole('slider',{name:'Drag comparison divider',exact:true});await handle.focus();await page.keyboard.press('ArrowRight');assert.equal(await handle.getAttribute('aria-valuenow'),'28');const box=await handle.boundingBox();await page.mouse.move(box.x+box.width/2,box.y+box.height/2);await page.mouse.down();await page.mouse.move(box.x+100,box.y+box.height/2);await page.mouse.up();assert.ok(Number(await handle.getAttribute('aria-valuenow'))>28);await page.getByRole('button',{name:'Zoom in',exact:true}).click();await page.mouse.move(600,400);await page.mouse.down();await page.mouse.move(650,450);await page.mouse.up();await page.getByRole('button',{name:'DIFFERENCE',exact:true}).click();assert.equal(await page.getByRole('button',{name:'DIFFERENCE',exact:true}).getAttribute('aria-pressed'),'true');await page.getByRole('button',{name:'SPLIT',exact:true}).click();});
await check('Config edits require review and apply; Escape cancels',async()=>{await page.goto(base+'/config');await page.getByRole('checkbox',{name:'LiDAR points Mission renderer layer'}).uncheck();await page.getByRole('button',{name:'Review changes'}).click();await page.getByRole('dialog').waitFor();await page.keyboard.press('Escape');assert.equal(await page.getByRole('dialog').count(),0);assert.ok(await page.getByRole('button',{name:'Review changes'}).evaluate(el=>el===document.activeElement));await page.getByRole('button',{name:'Review changes'}).click();await page.getByRole('button',{name:'Apply locally'}).click();await page.getByText('Applied to local visualization.').waitFor();await page.getByRole('checkbox',{name:'LiDAR points Mission renderer layer'}).check();await page.getByRole('button',{name:'Review changes'}).click();await page.getByRole('button',{name:'Apply locally'}).click();});
await check('Architecture remains available through commands',async()=>{await page.goto(base+'/architecture');await page.getByText('AVR-MAP PIPELINE ARCHITECTURE',{exact:true}).waitFor();});
const responsive=[];
for(const [width,height] of [[1920,1080],[1600,900],[1440,900],[1366,768],[1280,720],[900,700],[390,844]]){
 await page.setViewportSize({width,height});
 for(const route of ['/mission','/compare','/benchmarks','/diagnostics','/config']){
  await page.goto(base+route);await page.evaluate(()=>document.fonts.ready);await page.waitForTimeout(200);
  const overflow=await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth);
  const clipped=await page.evaluate(()=>[...document.querySelectorAll('button,a,input,select')].filter(el=>{const r=el.getBoundingClientRect();return r.width>0&&r.height>0&&(r.right>innerWidth+1||r.left<-1)&&getComputedStyle(el).visibility!=='hidden';}).map(el=>el.getAttribute('aria-label')||el.textContent));
  responsive.push({width,height,route,overflow,clipped});
  if(route==='/mission') {
   await page.screenshot({path:`.screenshots/final-freeze-mission-${width}x${height}.png`});
   await page.screenshot({path:`.screenshots/astra-mission-${width}x${height}.png`});
  }
 }
 console.log('Responsive',width,height);
}
await page.setViewportSize({width:1920,height:1080});
await check('All three fonts render from production assets with internet blocked',async()=>{
 await context.route('**/*',route=>route.request().url().startsWith(base)?route.continue():route.abort());
 await page.goto(base+'/mission');
 const fonts=await page.evaluate(async()=>{const names=['IBM Plex Sans','IBM Plex Sans Condensed','IBM Plex Mono'];for(const name of names)for(const weight of [400,500,600])await document.fonts.load(`${weight} 13px "${name}"`);await document.fonts.ready;return names.map(name=>({name,loaded:[...document.fonts].some(f=>f.family.replaceAll('"','')===name&&f.status==='loaded')}));});
 assert.ok(fonts.every(f=>f.loaded),JSON.stringify(fonts));
 await context.setOffline(true);assert.ok(await page.evaluate(()=>document.fonts.check('400 13px "IBM Plex Mono"')));await page.getByRole('button',{name:'RESOLUTION',exact:true}).click();await page.screenshot({path:'.screenshots/astra-offline-fonts.png'});await context.setOffline(false);
});
await fs.writeFile('.screenshots/audit.json',JSON.stringify({errors,warnings,externalRequests:network.filter(u=>!u.startsWith('http://127.0.0.1')&&!u.startsWith('data:')),audits,checks,responsive},null,2));
await browser.close();await new Promise(resolve=>server.close(resolve));
if(errors.length||checks.some(c=>!c.pass)||audits.some(a=>a.violations.length)||responsive.some(r=>r.overflow||r.clipped.length))process.exitCode=1;
