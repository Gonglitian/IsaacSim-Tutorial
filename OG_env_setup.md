# 🚀 OmniGibson Installation Guide  
**(with People Extension & Remote Streaming support)**

This guide walks you through setting up **OmniGibson**. Let’s get started! 🛠️

---

## 🐍 1. Create Conda Environment

```bash
conda create -n omnigibson python=3.10 pytorch torchvision torchaudio pytorch-cuda=12.1 "numpy<2" -c pytorch -c nvidia
conda activate omnigibson
```
---

## 📦 2. Install OmniGibson
If you want to use remote streaming for local visualization or people extension in OmniGibson, please clone my repo
```bash
git clone https://github.com/Gonglitian/OmniGibson.git
cd OmniGibson
pip install -e .
```

If not, you can just use official repo for installation.

```bash
git clone https://github.com/StanfordVL/OmniGibson.git
cd OmniGibson
pip install -e .
```
Or 

```
pip install omnigibson # Source not editable
```

✅ **Explanation:**
- The `-e .` flag installs the project in **editable mode**.

---

## 📁 3. Install Isaac Sim, Dataset, and Assets

```bash
python -m omnigibson.install
```

📥 **What this does:**
- Downloads and installs **Isaac Sim**.
- Retrieves necessary **scene assets** and **datasets** in **Behavior-1K** used by OmniGibson.

### (Optional) Custom Dataset Path
Do the following in `omnigibson/macros.py`:
```python
gm.ASSET_PATH = determine_gm_path(os.path.join("your_path_here", "assets"), "OMNIGIBSON_ASSET_PATH")
gm.DATASET_PATH = determine_gm_path(os.path.join("your_path_here", "og_dataset"), "OMNIGIBSON_DATASET_PATH")
gm.KEY_PATH = determine_gm_path(os.path.join("your_path_here", "omnigibson.key"), "OMNIGIBSON_KEY_PATH")
```
Once configured, the directories will be automatically created (if they do not exist), and the corresponding data will be downloaded to the specified locations.

## 🧍‍♂️ 4. (Optional) Switch to `og-people` branch for people simulation support

```bash
git switch og-people
git clone https://github.com/sybrenstuvel/Python-RVO2.git
cd Python-RVO2/
pip install Cython
python setup.py build
python setup.py install
```

👥 **Why this matters:**
- The `og-people` branch enables **human agent simulation** using the **RVO2** (Reciprocal Velocity Obstacle) crowd motion algorithm.
- `Cython` is required to compile the native parts of RVO2.

---

## 🌐 5. (Optional) Enable Remote Streaming Visualization

### 1️⃣ Run the streaming server on the remote machine:

Use the provided [example code](https://github.com/Gonglitian/IsaacSim-Tutorial/blob/main/OG_streaming.py):

```bash
python OG_streaming.py
```

🖥️ This starts a **remote Isaac Sim server** that streams rendered frames over the network.

---

### 2️⃣ Install and use the Omniverse Streaming Client on your local machine:

🔗 [Download Omniverse Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html#isaac-sim-latest-release)
![image](https://github.com/user-attachments/assets/4f6330e2-1aa3-444b-97a9-83949d182b2d)

📡 Launch the client and input the **remote machine’s IP address** to visualize the simulation **in real time** on your local system.
![image](https://github.com/user-attachments/assets/0a61c0eb-8c9d-4f6e-b219-4d4632c10047)

---

✅ You're all set! Happy simulating with OmniGibson! 🧠🤖



