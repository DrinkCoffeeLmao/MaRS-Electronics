#ROS2 executing python package
###Make sure to instal pip3,and source colcon build and etc..
Do you wanna run teleop keyboard control on your computer?? Then do the following
- First you might wanna code your python file
- Second create a python package folder using the following command
```ros2 pkg create \folder_name\ --build-type ament_python --dependencies rclpy```
You can also code cpp files btw, you just have to build it with ament_cmake, but python is easier :thumbs_up:
-You should be able to `setup.py`,`resources`,`\pkg_name`, and some other folders too...if not then re-install
-You see this folder called `\pkg_name\`, that is where you store your codes...ie you store your teleop_drive.py or some other .py files there, so you should navigate into that folder, your path should look something like this
```...\pkg_name\pkg_name```
-Then you should do ```touch file.py``` and ```chmod +x file.py```
- Make sure to colcon build after these process
-This is important step, now under the 'pkg_name' file, you see setup.py, open that in VS code and you should see 'entry_points={}', add the follwing inside
```entry_points={
        'console_scripts': [
            "node_name:pkg_name.file.py:main"
        ],
    },
```
- Now your python pkg is ready to go
