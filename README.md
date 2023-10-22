
# Cognitive Encoding for Decision Making of Autonomous Driving

This repository is along with the paper 'Emergence of Societally Exemplary Behavior in Autonomous Driving from Bio-plausible Cognitive Encoding'. This README.md details how to reproduce the results of paper.

## System Requirements
* Operating System: Linux Ubuntu 20.04 (recommended) or MacOS (only for demo)
* Programming Language: Python 3.8

## Installation

An isolated virtual environment is recommended for installation (such as Conda). The installation of this repository is shown as follows.

1. Clone this repository with:

    `git clone https://github.com/Meta-Drive-Lab/Cog_Dec_Mak.git`

2. Navigate to the root folder of the repository (`[..]/Cof_Dec_Mak`) and install requirements:

    `pip install -r requirements.txt`

3. Download the required scenarios from [CommonRoad scenarios](https://gitlab.lrz.de/tum-cps/commonroad-scenarios) by:

    `git clone https://gitlab.lrz.de/tum-cps/commonroad-scenarios`

    Thus, you will have the following folder structure:

    ```
    ├── Cog_Dec_Mak (This repository)
      ├── commonroad-scenarios
    ```
    

## Quick Start Demo

To run the ethical planner on an exemplary default scenario, execute the following command from the root directory of this repository:
    
* `python planner/Frenet/frenet_planner.py`

![Exemplary Result](readme/running_sample.gif)

You will see a live visualization of the scenario being solved by the planner.
Now you can start with your own experiments by changing the [configs](/planner/Frenet/configs/README.md) or selecting another scenario by adding

* `--scenario <path-to-scenario>`

to the command.

By default logs are saved and can be analyzed afterwards by running:

* `python planner/Frenet/analyze_tools/analyze_log.py`


[<img src="./readme/log_analysis.png" width="450" height="250">](/readme/log_analysis.png)


## How to reproduce results

The following describes how the results from the paper can be reproduced. 

To evaluate the proposed method over all 2000 scenarios, please run:

* `python planner/Frenet/plannertools/evaluatefrenet.py --weights ethical --all`

Please note that at least 200 GB space left is required on your device for saving the log files. For better runtime, we recommend using [multiprocessing](/planner/Frenet/plannertools/evaluatefrenet.py#L46) and a [GPU](planner/Frenet/configs/prediction.json#L4) for the prediction network (RTX3090 used in our work). Evaluating all scenarios in 10 parallel threads with a GPU takes around 48 hours (sometimes longer, depends on your computational resources). Results and logfiles for each run are stored in `planner/Frenet/results`.

After tesing, standard evaluation metrics relevant to our paper such as the cummulated harm from 2000 benchmark scenarios can be seen within the results (`results/eval/harm.json`). 

## Acknowledgement

We would like to thank the planner provided in the paper '[An ethical trajectory planning algorithm for autonomous vehicles](https://doi.org/10.1038/s42256-022-00607-z)', a brilliant study that provides a benchmark for the ethical issue of different road users. Based on their benchmark and provided planner, we test our bio-plausible scheme for decision making.
