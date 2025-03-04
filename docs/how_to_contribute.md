# How to contribute

## Contribute Process

1. Fork the original repository from https://github.com/jsk-ros-pkg/jsk_aerial_robot.
2. Develop your own feature in your forked repository.
3. Once you finish your development, make a pull request to the original repository.
4. The original repository maintainer will review and merge your pull request.

## Code Style

We use Clang-format to format the C++ code, and use black to format the Python code. 
Pre-commit, the format plugin, will run automatically before each commit. The specific function has been written in 
.pre-commit-config.yaml. To activate this plugin, please run the following command in terminal and root directory:

```bash
    pip3 install pre-commit
    pre-commit install
```
