# CARLA Cooperative V2X Intersection Demo (Scripts Snapshot)

This README captures the custom Python scripts we built for a cooperative intersection controller on CARLA 0.9.16. Save them under `PythonAPI/examples/coop_v2x/` in your CARLA tree to mirror the expected structure before publishing to GitHub.

## Files (expected locations)
- `PythonAPI/examples/coop_v2x/run_simulation.py`  
  Entry point: loads a map (e.g., Town05), enables synchronous + fixed-delta stepping, spawns vehicles, applies FCFS-style intersection permissions, and logs timing metrics.
- `PythonAPI/examples/coop_v2x/intersection_manager.py`  
  Minimal FCFS intersection manager: tracks arrival, permission, entry, and exit times; allows one vehicle in the intersection box at a time.
- `PythonAPI/examples/coop_v2x/vehicle_agent.py`  
  Wraps CARLA’s `BasicAgent`; brakes on red, follows route to destination on green.
- `PythonAPI/examples/coop_v2x/logging_utils.py`  
  CSV logger for arrival/permission/enter/exit timestamps.

## Usage (PowerShell example, with `CarlaUE4.exe` running and venv active)
```powershell
python run_simulation.py --map Town05 `
  --spawn-indices "43,151,177,137" --dest-indices "151,43,137,177" `
  --center-x -45.5 --center-y 4.4 --center-z 0.45 `
  --approach 35 --box 12 --vehicles 4 --logfile metrics.csv
```
Key flags:
- `--map <Town>`: load map (e.g., Town05).
- `--spawn-indices "a,b,c"`: spawn points to use.
- `--dest-indices "x,y,z"`: destinations per vehicle.
- `--center-x/--center-y/--center-z`: intersection center.
- `--approach`: approach radius (m) for queueing.
- `--box`: half-extent (m) of the intersection box.
- `--vehicles`: number of vehicles to spawn.
- `--logfile`: CSV output path.
- `--list-spawns`: print spawn points and exit.

## Notes
- Align spawn/destination indices with your chosen intersection center; otherwise vehicles may never enter the managed box.
- Adjust `--center` and `--box` based on the junction you target.
- Metrics are logged to CSV for wait/crossing time analysis.

## Publishing to GitHub (suggested minimal tree)
1) Create a clean repo directory (outside the CARLA install) and recreate the CARLA-like subpaths:
```
<repo>/PythonAPI/examples/coop_v2x/
```
2) Copy the four scripts above into that folder (and this README at repo root).  
3) Add a `.gitignore` that ignores everything except `PythonAPI/examples/coop_v2x/*.py` and the README.  
4) `git init`, commit, and push to your GitHub repo.

