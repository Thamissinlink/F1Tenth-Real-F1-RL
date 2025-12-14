# F1TENTH Autonomous Racer: Imitation Learning from Real F1 Telemetry + RL Fine-Tuning


Autonomous racing stack for the F1TENTH platform that learns driving behavior directly from real Formula 1 telemetry data.

## Highlights
- Behavioral cloning from real F1 quali laps (2020–2025 seasons) using FastF1 telemetry
- Policies initialized from expert drivers (e.g., Verstappen, Hamilton, Leclerc)
- Fine-tuned with reinforcement learning (PPO/SAC) in F1TENTH Gym
- Full ROS2 perception → planning → control pipeline
- Achieved [X.X seconds] lap times on [track name] (top Y% on community leaderboard)

## Demo
[Coming Soon...]

## Tech Stack
- ROS2 Humble
- F1TENTH Gym (Gazebo simulation)
- FastF1 for real F1 telemetry
- PyTorch (imitation learning)
- Stable-Baselines3 (RL)
- SLAM Toolbox, Nav2, particle filter localization

## Results
| Method              | Lap Time (Levy track) | Success Rate |
|---------------------|-----------------------|--------------|
| Pure Pursuit        | X.XX s                | 100%         |
| Imitation Learning  | X.XX s                | XX%          |
| IL + RL Fine-tune   | X.XX s                | XX%          |

## Setup & Usage
...
