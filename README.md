reem_simulation
===============

Packages for running REEM in the Gazebo simulator.

We are using the standalone version of Gazebo (1.5+), so you'll need to do some environment setup for Gazebo to pick up the REEM model and worlds:

```
# Gazebo - atlas_msgs
export GAZEBO_PLUGIN_PATH=`rospack find atlas_msgs`/lib:$GAZEBO_PLUGIN_PATH                       # plugins

# Gazebo - PAL
export GAZEBO_PLUGIN_PATH=`rospack find pal_gazebo_plugins`/lib:$GAZEBO_PLUGIN_PATH               # plugins

# Gazebo - REEM-H3
export GAZEBO_MODEL_PATH=`rospack find reem_gazebo`/models:$GAZEBO_MODEL_PATH                     # models
export GAZEBO_RESOURCE_PATH=`rospack find reem_gazebo`/reem_gazebo/worlds:$GAZEBO_RESOURCE_PATH   # resources
```

If you plan to use REEM on a regular basis, consider adding the above snippet to your ~/.bahsrc or equivalent.
