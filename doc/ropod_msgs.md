ROPOD Messages for Demo 1
=========================

# Envelope structure

- header
 - type: STRING (required); shortcut to avoid validation of payload against metamodel 
 - version: STRING; optional version string e.g. "0.1.0"
 - metamodel: STRING; URI to payload schema
 - msg_id: choice = UUID; unique ID for traceability (required); 
- payload: JSON subpart (required) 
 - with required "metamodel" tag

# Waypoint Messages

Full message example:

```
{
  "header": {
    "type": "CMD",
    "version": "0.1.0",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "0d05d0bc-f1d2-4355-bd88-edf44e2475c8",
    "timestamp": "2017-11-11T11:11:00Z"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "GOTO",
        "location": "START"
      },
      { 
        "command": "GOTO",
        "location": "MOBIDIK"
      }
    ]
  }
}
```

## START

```
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "a5339f9e-5cc4-454d-a0d3-383163dc7b45"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "GOTO",
        "location": "START"
      }
    ]
  }
}
```

## MOBIDIK

```
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "12b24ce8-db2e-4d15-a393-dded5b55e5eb"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "GOTO",
        "location": "MOBIDIK"
      }
    ]
  }
}
```

## ELEVATOR

```
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "8066f827-d297-4b8f-8939-22ffd0659f38"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "GOTO",
        "location": "ELEVATOR"
      }
    ]
  }
}
```

```
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "49525ac1-c09f-4cef-8d64-65976c80370c"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "ENTER_ELEVATOR"
      }
    ]
  }
}
```

``ENTER_ELEVATOR`` is supposed to be triggered by CCU once elevator control signals the elevator has arrived. For the demo a human will trigger this."


```
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "8d5ca928-d086-41b8-b48a-c1ab9f6b9e89"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "EXIT_ELEVATOR"
      }
    ]
  }
}
```

``EXIT_ELEVATOR``: Go to an area in front of the elevator.


# Coordination Messages

```
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "7b682bd9-41a2-4f88-a485-0fb8e6e861fb"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "PAUSE"
      }
    ]
  }
}
```

```
{
  "header": {
    "type": "CMD",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "200663fa-a659-48bc-b295-b01c1680a81d"
  },
  "payload": {
    "metamodel": "ropod-demo-cmd-schema.json",
    "commandList": [
      { 
        "command": "RESUME"
      }
    ]
  }
}
```

# Progress Messages

```
                .... 
              .  x  . docking_area
              .......     
   --------+  ......c2 +---------------
           |  .  x  .  |
-----------+  ......   +---------------
            .......  
            .   x  ....   ......................
            .       x .   .   x          x wp1 .
junction j1 .         .   ......................
            ...........           corriddor c1
---------------------------------------       


```



```
{
  "header": {
    "type": "TaskProgress",
    "metamodel": "ropod-msg-schema.json",
    "msg_id": "200663fa-a659-48bc-b295-b01c1680a81d"
  },
  "payload": {
    "metamodel": "ropod-demo-task-progress-schema.json",
    "command": "GOTO",
    "location": "MOBIDIK",
    "status": "approaching",
    "reachedArea": {
      "areaName": "c1",
      "sequenceNumber": 1,
      "totalNumber": 4 
    }, 
    "reachedWaypoint": {
      "position": {
          "rencferenceId": "basement_map",
          "x": 10,
          "y": 20
      },
      "sequenceNumber": 2,
      "totalNumber": 5 
    }
  }
}
```



# Robot Pose Message

```
{
  "header":{
    "type":"RobotPose2D",
    "metamodel":"ropod-msg-schema.json",
    "msg_id":"5073dcfb-4849-42cd-a17a-ef33fa7c7a69"
  },
  "payload":{
    "metamodel":"ropod-demo-robot-pose-2d-schema.json",
    "robotId":"ropod_0",
    "pose":{
      "rencferenceId":"basement_map",
      "x":10,
      "y":20,
      "theta":3.1415
    }
  }
}
```
# Unit convention

Unit convention (will be added to message models in a new iteration) are *SI* units:

* position in **m**
* orientation in **RAD**

The values relate to  the *frame* as specified by ``rencferenceId``. For the demo this is the basement map as used by gmapping.


