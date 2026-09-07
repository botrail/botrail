import {test} from 'node:test';
import assert from 'node:assert/strict';
import {declaredMass, bomDifference, overviewConnection, parseProposal} from '../src/mounting-edit.ts';

test('missing mass is distinct from zero and partial totals remain partial', () => {
  assert.equal(declaredMass({known_kg: null, missing: ['adapter']}), 'Unknown');
  assert.equal(declaredMass({known_kg: 12, missing: ['adapter']}), '12.000 kg + unknown');
  assert.equal(declaredMass({known_kg: 0, missing: []}), '0.000 kg');
});

test('application comes from the shared checker independently of strict ready', () => {
  const inspection = {schema_version:'1',report:{ready:false,input_hash:'test',items:[]}};
  const raw = {schema_version:'1',robot:'arm',scene:{robots:[{name:'arm'}]},before_scene:{robots:[{name:'arm'}]},
    base_revision:'a',candidate_revision:'b',can_apply:true,mounting_blockers:[],inspection,before_inspection:inspection};
  const allowed = parseProposal(raw);
  assert.equal(allowed.can_apply, true);
  assert.equal(allowed.inspection.ready, false);
  const blocked = parseProposal({...raw,can_apply:false,mounting_blockers:['mounting:arm/tool:pose']});
  assert.equal(blocked.can_apply, false);
  assert.deepEqual(blocked.mounting_blockers, ['mounting:arm/tool:pose']);
  assert.equal(parseProposal({...raw,can_apply:undefined}).can_apply, false);
});

test('BOM differences keep catalog revisions distinct and use purchase quantities', () => {
  const kit = {model: 'Kit', names:['arm/tool'], catalog:{id:'vendor/kit/r1',revision:'a'}, qty:1};
  const b = {bom: [kit]}, a = {bom: [{...kit,catalog:{...kit.catalog,revision:'b'}}]};
  assert.deepEqual(bomDifference(b,a), [{name:'Kit',before:1,after:0},{name:'Kit',before:0,after:1}]);
});

test('whole-assembly framing cannot invent mating faces or evidence', () => {
  const c = overviewConnection({connections:[]}, {name:'arm',base_pose:{position:[0,0,0],quaternion:[0,0,0,1]},links:[{name:'root'},{name:'tool'}]});
  assert.deepEqual(c.movingLinks,['root','tool']);
  assert.equal(c.flange.pose,null);
  assert.equal(c.mount.mapped,false);
  assert.deepEqual(c.mount.holes,[]);
});

import {declaredCandidates} from '../src/mounting-edit.ts';
test('only explicit unmet tool requirements suggest product IDs, including unknown alternatives', () => {
  const finding = {id:'arm/tool:required:adapter',target:'arm/tool',status:'unknown',raw:{evidence:{order_requirement:{catalog:'vendor/adapter/r1'},requirement:{catalog_alternatives:[{catalog:'vendor/other/r1'}]}}}};
  assert.deepEqual(declaredCandidates({findings:[finding]},'arm').map(s=>s.catalog), ['vendor/adapter/r1','vendor/other/r1']);
  assert.deepEqual(declaredCandidates({findings:[{...finding,status:'pass'}]},'arm'), []);
  assert.deepEqual(declaredCandidates({findings:[finding]},'other_robot'), []);
});
