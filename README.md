# Installation Guide

To install the required packages for the congestion-aware coverage planning algorithm, follow these steps.

1. Clone the repository:

```bash
git clone https://github.com/convince-project/congestion-coverage-plan.git
```

2. Navigate to the project directory:

```bash
cd congestion-coverage-plan
```

3. (Optional) Create and activate a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

4. Install the required packages using pip (editable mode recommended for development):

```bash
pip install -e .
```


5. To test if the library works, run the following command inside the `congestion-coverage-plan` directory:

```bash
python3 main.py run --map atc_corridor_21 --algorithms lrtdp --convergence_threshold 0.01 --wait_time 20 --time_bound_lrtdp 350 --time_bound_real 3000 --heuristic mst_shortest_path --times 1351651349.547
installing the library

```
# Install dependencies
apt-get update && apt-get install -y \
python3 \
python3-pip \
python3-venv \
git 

# install the devices messages 
cd ~
git clone https://github.com/morpheus1820/occupancy_static_devices
cd occupancy_static_devices/static_devices_msgs
colcon build
source install/setup.bash
cd ~;
# clone the repo
git clone --branch fix/UC3 https://github.com/convince-project/congestion-coverage-plan.git;

# install it
cd  congestion-coverage-plan;

python3 -m venv create venv-congestion-coverage-plan;
source ./venv-congestion-coverage-plan/bin/activate;
touch ./venv-congestion-coverage-plan/COLCON_IGNORE;
python3 -m pip install --upgrade pip;
python3 -m pip install -e .;
export PYTHONPATH=$PYTHONPATH:`pwd`/venv-congestion-coverage-plan/lib/python3.12/site-packages;
```