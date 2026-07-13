# Data Analysis

Raw ROS bags are not tracked by Git. Keep them under `trial_data/raw/` locally.

```bash
scripts/sync_jetson_bags.sh --local-dir trial_data/raw
/usr/bin/python3 scripts/analyze_bag_runs.py --bag-dir trial_data/raw --out-dir trial_data/processed
```

Generated outputs include:

- `moving_obstacle_tracks_xy.png`
- `obstacle_speed_timeline.png`
- `control_obstacle_response.png`
- `combined_timeseries.csv`
- `obstacle_tracks.csv`
