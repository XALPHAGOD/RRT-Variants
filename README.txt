Each of the M1-M5 folders has an env.py file which is the obstacle map.

Replace the env.py in the current folder with the respective env.py from M1-M5 folders to work on a different map.

To run any algorithm execute the algorithm.py files in this folder.
eg.
python extended_rrt.py
python quick_rrt_star.py
python rrt.py
python rrt_connect.py
python rrt_star.py
python rrt_star_fn.py
python rrt_star_smart.py

Do not modify or delete any of the following files in this folder:
plotting.py
utils.py

To change the position of start or goal nodes for any algorithm.py file, change the x_start and x_goal values present in the main method of that algorithm.py file

In each env.py file the x_start and x_goal values are mentioned, so change them accordingly in the main method of every alogrithm.py before executing.