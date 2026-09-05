"""Display refinements must keep the verified mechanical cell intact."""
import json

import pytest
import botrail as bt


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('door', ['manual', 'servo'])
def test_glazed_machine_keeps_collision_switches_and_moving_trim(tmp_path, side, door):
    plain, full = bt.Scene(), bt.Scene()
    options = dict(door=door, door_side=side, yaw=0.37, position=(1.3, -0.4))
    a = bt.parts.machine_tool(plain, 'vmc', detail='plain', **options)
    b = bt.parts.machine_tool(full, 'vmc', detail='full', **options)
    def snapshot(scene, filename):
        scene.save_project(tmp_path / filename)
        return json.loads((tmp_path / filename).read_text())
    pa, pb = snapshot(plain, 'plain.json'), snapshot(full, 'full.json')
    # Enabled shapes, poses and sensors keep their exact contracts.
    geometry = lambda p: {o['name']: (o['geometry'], o['pose']) for o in p['obstacles'] if o['enabled']}
    assert geometry(pa) == geometry(pb)
    assert pa['sensors'] == pb['sensors']
    assert plain.frames == full.frames
    assert plain.bom().rows == full.bom().rows
    by = {o['name']: o for o in pb['obstacles']}
    for group in ['front_door', 'side_door']:
        assert by[f'vmc/{group}/leaf']['enabled']
        assert not by[f'vmc/{group}/leaf']['visible']
        assert full.obstacle_opacity(f'vmc/{group}/window') == pytest.approx(.24)
    added = set(by) - {o['name'] for o in pa['obstacles']}
    assert all(not by[n]['enabled'] for n in added)
    # Every side-door panel and seal follows the door, except the fixed rails.
    assert {n for n in added if '/side_door/' in n and '/rail_' not in n} <= set(b.door_objects)
    assert set(a.door_objects) <= set(b.door_objects)
    loaded = bt.Scene.load_project(tmp_path / 'full.json')
    namespace = {}
    exec(loaded.generate_python().replace('bt.studio(scene)', ''), namespace)
    for s in [loaded, namespace['scene']]:
        assert s.obstacle_opacity('vmc/front_door/window') == pytest.approx(.24)
    output = tmp_path / 'machine.usdc'
    full.export_usd(output)
    from pxr import Usd, UsdShade
    stage = Usd.Stage.Open(str(output))
    windows = [p for p in stage.Traverse() if p.GetName() == 'window']
    assert len(windows) == 2
    for pane in windows:
        mat, _ = UsdShade.MaterialBindingAPI(pane).ComputeBoundMaterial()
        assert mat.ComputeSurfaceSource()[0].GetInput('opacity').Get() == pytest.approx(.24)


def test_visual_replacement_preserves_motion_and_survives_portable_project(tmp_path):
    # The visual model uses a rotated frame; installation compensates that
    # basis while retaining the original joint, collision and tip transforms.
    from test_visual_assets import write_visual_assets, project_json
    _, _, hand = write_visual_assets(tmp_path / 'source')
    urdf = '''<robot name="fixture"><link name="base">
      <collision><geometry><box size=".1 .1 .1"/></geometry></collision>
      </link><link name="tip"/>
      <joint name="tip" type="fixed"><parent link="base"/><child link="tip"/>
        <origin xyz="0 0 .1"/></joint></robot>'''
    original = bt.Robot.from_urdf_string(urdf)
    visual = bt.Robot.from_usd(hand)
    refined = original.with_visuals(visual)
    scene = bt.Scene(refined)
    assert refined.link_names == original.link_names
    assert refined.joint_names == original.joint_names
    assert refined.tcp_link == original.tcp_link
    assert scene.link_pose('tip') == bt.Scene(original).link_pose('tip')
    scene.add_box('probe', (.01,.01,.01), (.12,0,0))
    control = bt.Scene(original)
    control.add_box('probe', (.01,.01,.01), (.12,0,0))
    assert scene.min_obstacle_distance() == control.min_obstacle_distance()
    project = tmp_path / 'refined.botrail'
    scene.save_project(project)
    assert project_json(project)['robots'][0]['source']['kind'] == 'visuals'
    (tmp_path / 'source').rename(tmp_path / 'source-unavailable')
    restored = bt.Scene.load_project(project)
    namespace = {}
    exec(restored.generate_python().replace('bt.studio(scene)', ''), namespace)
    for candidate in [restored, namespace['scene']]:
        assert candidate.robot.link_names == original.link_names
        assert candidate.link_pose('tip') == scene.link_pose('tip')
        assert candidate.export_usd(tmp_path / 'refined.usdc') == []


def test_opacity_validation_and_clear():
    scene = bt.Scene()
    scene.add_box('pane', (.1,.1,.01), (0,0,0))
    for bad in [float('nan'), float('inf'), -.01, 1.01]:
        with pytest.raises(ValueError, match='opacity'):
            scene.set_obstacle_material('pane', opacity=bad)
    scene.set_obstacle_material('pane', opacity=.3)
    assert scene.obstacle_opacity('pane') == pytest.approx(.3)
    scene.set_obstacle_material('pane')
    assert scene.obstacle_material('pane') is None
    assert scene.obstacle_opacity('pane') is None


def test_urdf_relative_mesh_keeps_its_source_directory_after_save(tmp_path):
    from test_visual_assets import project_json
    package = tmp_path / 'vendor & mesh'
    package.mkdir()
    mesh = package / 'body.obj'
    mesh.write_text('v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    urdf = package / 'model.urdf'
    urdf.write_text('''<robot name="mesh"><link name="body">
      <visual><geometry><mesh filename="body.obj"/></geometry></visual>
      <collision><geometry><mesh filename="body.obj"/></geometry></collision>
      </link></robot>''')
    scene = bt.Scene(bt.Robot.from_urdf(urdf))
    project = tmp_path / 'saved.botrail'
    scene.save_project(project)
    xml = project_json(project)['robots'][0]['source']['xml']
    assert str(mesh).replace('&', '&amp;') in xml
    restored = bt.Scene.load_project(project)
    assert mesh in restored._asset_paths()
    namespace = {}
    exec(restored.generate_python().replace('bt.studio(scene)', ''), namespace)
    assert mesh in namespace['scene']._asset_paths()
