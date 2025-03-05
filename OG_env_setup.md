This is guidance for install of **OmniGibson (with people extension and remote streaming)**

# OmniGibson Install

## 1. Conda Env

```bash
conda create -n omnigibson python=3.10 pytorch torchvision torchaudio pytorch-cuda=12.1 "numpy<2" -c pytorch -c nvidia
conda activate omnigibson
```
## 2. Install OmniGibson

```bash
git clone https://github.com/Gonglitian/OmniGibson.git
cd OmniGibson
pip install -e .
```

## 3. Install Isaac Sim as well as OmniGibson dataset and assets:

```bash
python -m omnigibson.install
```

## 4. Switch to 'og-people' tag to dev people extension

```bash
git switch og-people
```

# Remote Streaming
1. Run [example code](https://github.com/Gonglitian/IsaacSim-Tutorial/blob/main/OG_streaming.py) in the remote machine to start streaming

2. Download [Omniverse Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html#isaac-sim-latest-release) (shown following), input remote machine IP address and connect to see the local visualization of remote streaming graphics.

![image](https://github.com/user-attachments/assets/4f6330e2-1aa3-444b-97a9-83949d182b2d)

