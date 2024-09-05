# DataGenerationTool_Carla
## Install
```shell
git clone https://github.com/Jas000n/DataGenerationTool_Carla
cd DataGenerationTool_Carla
conda env create -f environment.yml
conda activate DataGeneration
chmod +x setup_carla.sh
./setup_carla.sh
```
## Run
### Start Carla with screen off
```shell
./carla/CarlaUE4.sh -RenderOffScreen
```
### Start Carla 
```shell
./carla/CarlaUE4.sh 
```
### Start Bash to collect data
```shell
./data_collect.sh
```
## Check Distribution
python check_lidar_category_distribution.py