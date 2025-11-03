# ROS2 Executing Python Package

### Make sure to install `pip3`, and source `colcon build`, etc.

---

## 🚀 Do you wanna run teleop keyboard control on your computer??

Then do the following 👇

---

### 1️⃣ First you might wanna code your python file

---

### 2️⃣ Create a Python package folder

Use the following command:

```bash
ros2 pkg create <folder_name> --build-type ament_python --dependencies rclpy
```

> 💡 You can also code **C++ files** btw, you just have to build it with `ament_cmake`, but **Python is easier 👍**

---

### 3️⃣ You should be able to see the following files/folders:

- `setup.py`  
- `resource/`  
- `<pkg_name>/`  
- and some other folders too...

> If not, then **re-install** your package.

---

### 4️⃣ Store your Python codes

You see this folder called `<pkg_name>/` — that’s where you store your codes.  
For example, store your `teleop_drive.py` or other `.py` files there.

So you should navigate into that folder:

```bash
cd <pkg_name>/<pkg_name>
```

Your path should look something like this:
```
.../<pkg_name>/<pkg_name>
```

Then create your Python file and make it executable:

```bash
touch file.py
chmod +x file.py
```

---

### 5️⃣ Build your package

After these processes, make sure to **colcon build**:

```bash
colcon build
```

---

### 6️⃣ Setup `entry_points` in `setup.py`

This is an important step!

Now, under the `<pkg_name>` folder, open the `setup.py` file in VS Code.  
Inside it, you should see a section that looks like this:

```python
entry_points = {
    'console_scripts': [
        "node_name = pkg_name.file:main"
    ],
}
```

---

### ✅ Now your Python package is ready to go!
