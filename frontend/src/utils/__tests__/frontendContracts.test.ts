import assert from 'node:assert/strict';
import { describe, it } from 'node:test';
import { readFileSync } from 'node:fs';
import { SCENARIOS } from '../../demo/scenarios';
import { containsPoint, compareTopology } from '../topology';
import { explainCellResolution } from '../resolution';
import { validatePerceptionFrame } from '../../demo/fixtureValidation';
import { PRIMARY_ROUTES } from '../../components/navigation/routes';
import { pipelineStatusSeverity } from '../status';

describe('Frontend product contracts',()=>{
 it('exposes exactly the five required primary routes',()=>assert.deepEqual(PRIMARY_ROUTES.map(r=>r.path),['/mission','/compare','/benchmarks','/diagnostics','/config']));
 for(const [id,frame] of Object.entries(SCENARIOS)) {
  it(`${id}: return counts derive from actual half-open footprints`,()=>{
   for(const c of frame.cells) assert.equal(c.pointCount,frame.points.filter(p=>containsPoint(c,p)).length,c.cellId);
   assert.equal(validatePerceptionFrame(frame).valid,true);
  });
  it(`${id}: active leaves do not overlap`,()=>{
   for(let i=0;i<frame.cells.length;i++)for(let j=i+1;j<frame.cells.length;j++){
    const a=frame.cells[i],b=frame.cells[j];assert.ok(Math.abs(a.x-b.x)>=(a.sizeMeters+b.sizeMeters)/2-1e-7 || Math.abs(a.y-b.y)>=(a.sizeMeters+b.sizeMeters)/2-1e-7,`${a.cellId} overlaps ${b.cellId}`);
   }
  });
  it(`${id}: compare uses exactly the same area, no rounded bounding-box inflation`,()=>{
   const t=compareTopology(frame.cells);assert.ok(Math.abs(t.area-frame.cells.reduce((s,c)=>s+c.sizeMeters**2,0))<1e-6);assert.equal(t.adaptiveCount,frame.cells.length);
  });
 }
 it('half-open ownership assigns boundary points to one owner',()=>{
  const a={cellId:'a',x:0,y:0,sizeMeters:.1,pointCount:0,level:1 as const,resolutionReasons:[]};const b={...a,x:.1};
  assert.equal(containsPoint(a,{x:.05,y:0}),false);assert.equal(containsPoint(b,{x:.05,y:0}),true);
 });
 it('does not infer semantics from an ID or fabricate returns for an empty region',()=>{
  const cell={cellId:'D_L0_P_empty',x:900,y:900,sizeMeters:.05,pointCount:999,level:0 as const,resolutionReasons:['SEMANTIC_RELEVANCE' as const]};
  const e=explainCellResolution(cell,SCENARIOS.D);assert.equal(e.pointCount,0);assert.equal(e.semanticClass,undefined);assert.equal(e.staticElevationStats,undefined);
 });
 it('mixed dynamic/static cells keep routing authorization per point',()=>{
  const frame={...SCENARIOS.D,points:[{sourceIndex:1,x:0,y:0,z:0,semanticClass:'ROAD' as const,routing:'STATIC_FUSABLE' as const},{sourceIndex:2,x:0,y:0,z:1,semanticClass:'VEHICLE' as const,routing:'DYNAMIC_EXCLUDED' as const}]};
  const e=explainCellResolution({cellId:'mixed',x:0,y:0,level:0,sizeMeters:.05,pointCount:2,resolutionReasons:['DYNAMIC_RELEVANCE']},frame);
  assert.equal(e.routing,'MIXED');assert.equal(e.staticFusion,'MIXED');assert.match(e.decisionEvidence ?? '',/vehicle/);
 });
 it('rejects duplicate noise and orphan dynamic observations',()=>{
  const frame=SCENARIOS.C;
  assert.equal(validatePerceptionFrame({...frame,noiseSourceIndices:[999,999]}).valid,false);
  assert.equal(validatePerceptionFrame({...frame,dynamicPoints:[...frame.dynamicPoints,{...frame.dynamicPoints[0],sourceIndex:999}]}).valid,false);
 });
 it('rejects padded cluster bounds and rounded source statistics',()=>{
  const frame=SCENARIOS.D;
  const cluster=frame.dynamicClusters[1];
  const check=(changed:typeof cluster)=>validatePerceptionFrame({...frame,dynamicClusters:[frame.dynamicClusters[0],changed]});
  assert.equal(check({...cluster,aabbMin:{...cluster.aabbMin,x:cluster.aabbMin.x-.1}}).valid,false);
  assert.equal(check({...cluster,centroid:{...cluster.centroid,z:Math.round(cluster.centroid.z*100)/100}}).valid,false);
  assert.equal(check({...cluster,meanSemanticConfidence:Math.round(cluster.meanSemanticConfidence*100)/100}).valid,false);
 });
 it('classifies policy commit failure as terminal, semantic frame rejection as recoverable',()=>{
  assert.equal(pipelineStatusSeverity('POLICY_COMMIT_INTERNAL_ERROR'),'terminal');assert.equal(pipelineStatusSeverity('FRAME_REJECTED_SEMANTIC_ERROR'),'rejected');
 });
 it('bundles typography without external runtime font links',()=>{
  const html=readFileSync('index.html','utf8');assert.doesNotMatch(html,/fonts\.googleapis|fonts\.gstatic/);
  const main=readFileSync('src/main.tsx','utf8');for(const family of ['ibm-plex-sans','ibm-plex-sans-condensed','ibm-plex-mono'])assert.ok(main.includes(`@fontsource/${family}/latin-400.css`));
 });
});
