# CARLA Python Stubs

[![GitHub all releases](https://img.shields.io/github/downloads/mathiaswold/carla-python-stubs/total)](https://github.com/mathiaswold/carla-python-stubs/releases)

This repository contains [Python stub files](https://peps.python.org/pep-0484/#stub-files) for the [CARLA Python API](https://carla.readthedocs.io/en/latest/python_api/). Installing these along the CARLA Python API will allow you to use type hints and auto-completion in your code.

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
