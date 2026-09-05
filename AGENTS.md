## How to work on this project

- Save internal design documents in the `.internal` folder.
- Avoid overengineering. Develop features based on actual needs and requirements.
- When adding a feature, aim for the optimal change. Consider which approach fits best: writing something new from scratch, extending existing code, refactoring existing functionality to add it, or removing unnecessary code before adding.
- Check the appearance of the UI in a browser: the `/chrome` command, or the headless capture `scripts/docs_screenshots.py` uses (Playwright + SwiftShader).
- When a demo program needs geometry the catalog does not have — a single object or a whole environment — build it in Python from `bt.parts` and `add_box`, collected under an appropriate group name. Never freeze a dimension the cell verifies (aisle width, riser, storey height) into a baked asset.
- Reach for `Node.js + three-usd-robot` only for a *shape* boxes cannot express: a curved facade, a moulded counter, a spiral stair. Model it, check the appearance in the browser, export to USD, bring it in with `scene.add_mesh` / `scene.load_usd`, and pin its identity with `set_part`. A USD import carries geometry, `displayColor` and leaf `Xform`s (as frames), and nothing else — `walkable`, `enabled`, `visible` and materials are botrail state and have to be re-applied by name after loading. `set_obstacle_walkable` takes an upright box only, so the mesh is the picture and a box is what the machine stands on.
- Bake an environment to USD only when there is no Python at run time: the browser demo (`scripts/bake_demo_equipment.py`).
