<h1>Robotics Labs Course, Tel Aviv University, Mechanical Engineering</h1>

## Labs Content
In each lab there is an instructions file. Be sure to follow it.
* **[Lab 1](/Labs/Lab1)**: Introduction to the Kinova Gen3 Lite arm
* **[Lab 2](/Labs/Lab2)**: Forward Kinematics
* **[Lab 3](/Labs/Lab3)**: Numerical Inverse Kinematics
* **[Lab 4](/Labs/Lab4)**: Path Planning
* **[Lab 5](/Labs/Lab5)**: Visual Servoing Control
* **[Lab 6](/Labs/Lab6)**: Motion Planning for a mobile robot

* **[Final Project](/Labs/final_project)**

<a id="markdown-setup-example-python-environment" name="setup-example-python-environment"></a>
## Installation
<a id="markdown-requested-basic-python--python-modules" name="requested-basic-python--python-modules"></a>
### Required Python version and module

- Python >= 3.5
- pip

<a id="markdown-install-python-module-kortex-api--the-needed-dependencies" name="install-python-module-kortex-api--the-needed-dependencies"></a>
### Install Kortex Python API and required dependencies  
#### (Source: [Kortex Github](https://github.com/Kinovarobotics/kortex))
Install what is needed to run the examples via a downloaded whl file (Python wheel package).

The whl file can be downloaded via the Kinova Artifactory: [kortex_api](https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.2.0/kortex_api-2.2.0.post31-py3-none-any.whl)  

On Linux:

```sh
python3 -m pip install <whl relative fullpath name>.whl
```
**Note:** root privilege is usually required to install a new module under Linux.

On Windows:

```sh
python -m pip install <whl relative fullpath name>.whl
```

<a id="markdown-how-to-use-examples-with-your-robot" name="how-to-use-examples-with-your-robot"></a>

**Note**: Robot default IP address: ``192.168.1.10``

