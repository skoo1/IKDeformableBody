from ruamel.yaml import YAML, dump, RoundTripDumper
from RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
from mycost import RaisimGymEnv
import numpy as np

# config
cfg = YAML().load(open("cfg.yaml", 'r'))

# create environment from the configuration file

dump_cfg = dump(cfg['environment'], Dumper=RoundTripDumper)
MyRaisimGymEnv = RaisimGymEnv("/rsc", dump_cfg)
env = VecEnv(MyRaisimGymEnv)
env.seed(cfg['seed'])
ob_dim = env.num_obs
act_dim = env.num_acts
env.reset()

env.build(np.zeros([100, 3], dtype=np.float32), np.zeros([100, 3], dtype=np.float32))


obs = env.observe(False)
reward, dones = env.step(np.zeros([10,91]))

print('complete')