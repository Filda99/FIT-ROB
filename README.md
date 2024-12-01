# Mobile Robotic source code
This repository contains all the source files for the mobile robotic course of Polytech Angers.

## Monte Carlo Localization

This workshop aims to program a MonteCarlo Localization. The explanation of the work to do can be found in the document [documents/MCL/MCL.pdf](https://gitlab.u-angers.fr/cours/mobile_robotic_student/-/raw/master/documents/MCL/MCL.pdf)

To run the main program: `python3 main_mcl.py`

To run the unit tests: `python3 run_mcl_unit_tests.py`

Files to complete:
> - tp\_mcl/monte\_carlo.py
> - tp\_mcl/cost\_map.py

A video of the expected results: [https://youtu.be/HXVk_BERMqI](https://youtu.be/HXVk_BERMqI)


## Rapidly Explored Random Tree

This workshop aims to program a path planing algorithm based on a simple Rapidly explored Random Tree (RRT). The explanation of the work to do can be found in the document [documents/RRT/RRT.pdf](https://gitlab.u-angers.fr/cours/mobile_robotic_student/-/raw/master/documents/RRT/RRT.pdf)

To run the main program: `python3 main_rrt.py`

To run the unit tests: `python3 run_rrt_unit_tests.py`

Files to complete:
> - tp\_rrt/rrt\_star.py
> - tp\_rrt/utils.py
> 
A video of the expected results: [https://youtu.be/6R7sKZh-IEY](https://youtu.be/6R7sKZh-IEY)

## A star

This workshop aims to program a path planing algorithm based on A\*. The explanation of the work to do can be found in the document [documents/ASTAR/ASTAR.pdf](https://gitlab.u-angers.fr/cours/mobile_robotic_student/-/raw/master/documents/ASTAR/ASTAR.pdf)

To run the main program: `python3 main_astar.py`

To run the unit tests: `python3 run_astar_unit_tests.py`

Files to complete:
> - tp\_astar/node.py
> - tp\_astar/a\_star.py
> - tp\_astar/utils.py

A tool is also provided to help to understand the A\* algorithm but also the Dijskra and weighted A\* algorithms. To start the tool: `python3 tp_a_star/shortest_path_ui.py`
The help for the user interface can be found in the _Help_ menu.

## Iterative Closest Point

This workshop aims to program an Iterative Closest Point (ICP) algorithm to map two LiDAR sensor data. The explanation of the work to do can be found in the document [documents/ICP/ICP.pdf](https://gitlab.u-angers.fr/cours/mobile_robotic_student/-/raw/master/documents/ICP/ICP.pdf). A [Nelder And Mead](https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method) algorithm is considered to evaluate the best transformation.

To run the main program: `python3 main_icp.py`

To run the unit tests: `python3 run_icp_unit_tests.py`

Files to complete:
> - tp\_icp/icp.py
> - tp\_icp/simplex.py
> - tp\_icp/vertice.py

## Frontier Exploration

This workshop aims to program an frontier exploration algorithm for autonomous exploration. The explanation of the work to do can be found in the document [documents/EXPLORATION/EXPLORATION.pdf](https://gitlab.u-angers.fr/cours/mobile_robotic_student/-/raw/master/documents/EXPLORATION/EXPLORATION.pdf)

To run the main program: `python3 main_exploration.py`

To run the unit tests: `python3 run_exploration_unit_tests.py` (Work in progress... Not available yet...)

Files to complete:
> - tp\_exploration/exploration\_map.py
> - tp\_exploration/work2do.py
