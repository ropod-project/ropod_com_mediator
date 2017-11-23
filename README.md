
[Message definitions](doc/ropod_msgs.md)

#### Launch

```
roslaunch ropod_com_mediator com_mediator.launch
```

#### Topics
`/ropod_commands`: list of commands

`/ropod_zyre_debug`: output of entire zyre message

#### Test TF Bridge

```
rosrun ropod_com_mediator ropod_comm_mediator_test test
```

```
./ropod_0_tf_broadcaster.py
```
