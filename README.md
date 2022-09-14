# CARLA Python Stubs

[![GitHub all releases](https://img.shields.io/github/downloads/mathiaswold/carla-python-stubs/total)](https://github.com/mathiaswold/carla-python-stubs/releases)

This repository contains [Python stub files](https://peps.python.org/pep-0484/#stub-files) for the [CARLA Python API](https://carla.readthedocs.io/en/latest/python_api/). Installing these along the CARLA Python API will allow you to use type hints and auto-completion in your code.

![type-hints](https://user-images.githubusercontent.com/45951843/189996748-ef9e27f7-c3df-4249-bb22-659edffd54ea.png)

## Installation
Download the stub files (ending with `.pyi`) for your version of CARLA from [the releases page](https://github.com/mathiaswold/carla-python-stubs/releases). Then follow the installation instructions for your editor below.

### VS Code
VS code expects by default that custom stubs are placed in the `./typings` directory in your project. Create a subdirectory named `carla` (`./typings/carla`) and place the stub files there. You should now see type hints for the `carla`-module in your code. See [VS Code docs of `stubPath`](https://code.visualstudio.com/docs/python/settings-reference#_python-language-server-settings) for more information.


### PyCharm
Create a directory in the root of your project with any name, for example `./stubs`. Right-click the directory and select **Mark directory as** --> **Sources Root**. Create a subdirectory named `carla` (`./stubs/carla`) and place the stub files there. You should now see type hints for the `carla`-module in your code. See [PyCharm docs of stubs for external implementation](https://www.jetbrains.com/help/pycharm/stubs.html#create-stub-external) for more information.

### Other editors
See if your editor supports adding custom Python stubs. If not, you can add the stub files directly to the CARLA module:
1. Run `pip show carla` in the terminal. Find the `Location` path. This should end in a directory named `site-packages`.
2. Open the `Location` directory above. Place the stub files in the `carla` subdiretory (`.../site-packages/carla`).
3. You should now see type hints for the `carla`-module in your code.


## Generating stubs
Stubs are available in [releases](https://github.com/mathiaswold/carla-python-stubs/releases). You can generate them yourself:
1. Clone this repository.
2. Install the requirements: `pip install -r requirements.txt`
3. Run `python -m src.generate_stubs` in the terminal. This will generate stubs for the latest CARLA version. You can specify a different version by running `python -m src.generate_stubs --version <version>`. See `python -m src.generate_stubs --help` for more information.
