# DataGenerationTool_Carla
## Preview of the Town Setup
![random_explore.gif](pics/random_explore.gif)
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
### Start Carla with Screen off
```shell
./carla/CarlaUE4.sh -RenderOffScreen
```
### or Start Carla with Visualization
```shell
./carla/CarlaUE4.sh 
```
### Start Bash to Collect Data
```shell
./data_collect.sh
```
## Check Distribution
```shell
python check_lidar_category_distribution.py
```
