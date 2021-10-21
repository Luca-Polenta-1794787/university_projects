from gym.envs.registration import register

register(
    id='PacManGame-v0',
    entry_point='Pacman.envs:PacManGameEnv',
    max_episode_steps=2000,
)