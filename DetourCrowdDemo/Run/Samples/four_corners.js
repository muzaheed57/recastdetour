{
    "scene":
    {
        "file": "Meshes/square_20.obj"
    },
    "agents":
    [
        {
            "position": [-10, 0, -10],
            "destination": [10, 0, 10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pathOptimizationRange": 6,
                "updateFlags":
                [
                    "DT_CROWD_OBSTACLE_AVOIDANCE"
                ]
            }
        },
        {
            "position": [10, 0, 10],
            "destination": [-10, 0, -10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pathOptimizationRange": 6,
                "updateFlags":
                [
                    "DT_CROWD_OBSTACLE_AVOIDANCE"
                ]
            }
        },
        {
            "position": [-10, 0, 10],
            "destination": [10, 0, -10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pathOptimizationRange": 6,
                "updateFlags":
                [
                    "DT_CROWD_OBSTACLE_AVOIDANCE"
                ]
            }
        },
        {
            "position": [10, 0, -10],
            "destination": [-10, 0, 10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4,
                "pathOptimizationRange": 6,
                "updateFlags":
                [
                    "DT_CROWD_OBSTACLE_AVOIDANCE"
                ]
            }
        }
    ]
}