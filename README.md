[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# EAGERx

Engine Agnostic Gym Environment with Reactive extension (EAGERx) is a toolkit that will allow users to apply (deep) reinforcement learning for both simulated and real robots as well as combinations thereof.
The toolkit serves as bridge between the popular reinforcement learning toolkit [OpenAI Gym](https://gym.openai.com/) and robots that can either be real or simulated.
The EAGERx toolkit makes use of the widely used ROS framework for communication and ReactiveX synchronization of actions and observations.
Nonetheless, thanks to the flexible design of the toolkit, it is possible for users to create customized bridges for robots without ROS support.

## Key Functionalities and Features


| **Functionality/Feature**                                           | **EAGERx**         |
| ------------------------------------------------------------------- | -------------------|
| User-friendly creation of Gym environments for robot control tasks  | :heavy_check_mark: |
| Synchronization of actions and observations in simulators           | :heavy_check_mark: |
| Processing of data streams                                          | :heavy_check_mark: |
| Switching between physics engines                                   | :heavy_check_mark: |
| Documentation                                                       | :heavy_check_mark: |

#### Planned Functionalities and Features
We are currently working on the following features and functionalities:
- Demos
- Increasing the number of supported simulators and hardware
- Adding reset procedures

## Documentation

Documentation is available online: [https://eagerx.readthedocs.io](https://eagerx.readthedocs.io)


## Installation

**Prerequisites**: EAGERx requires ROS Noetic and Python 3.6+ to be installed.

---
**NOTE**
Older ROS versions, such as ROS Melodic are not supported.

---

Follow the instructions [here](https://github.com/tasostefas/opendr_internal/wiki/Development-System-Setup/) to install the OpenDR toolkit.

## Example

TODO


## Citing the Project

To cite this repository in publications:
```bibtex
@misc{eagerx,
  author = {Van der Heijden, Bas and Luijkx, Jelle},
  title = {EAGERx: Engine Agnostic Gym Environment with Reactive extension},
  year = {2021},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/eager-dev/eagerx}},
}
```

## Maintainers

EAGERx is currently maintained by [Bas van der Heijden](https://github.com/bheijden) (@bheijden) and [Jelle Luijkx](https://github.com/jelledouwe) (@jelledouwe).

---


## Acknowledgments


EAGERx received funding from the [OpenDR](https://opendr.eu/) project.
