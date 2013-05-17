{
    "scene":
    {
        "file": "Meshes/square_20.obj"
    },
    "agents":
    [
        {
            "position": [-10, 0, -10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                    {
                        "behavior":
                        {
                            "type": "pathFollowing",
                            "destination": [10, 0, 10],
                            "pathOptimizationRange": 6
                        }
                    },
                    {                       
                        "behavior":
                        {
                            "type": "collisionAvoidance"
                        }
                    }
                ]
            }
        },
        {
            "position": [10, 0, 10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                    {
                        "behavior":
                        {
                            "type": "pathFollowing",
                            "destination": [-10, 0, -10],
                            "pathOptimizationRange": 6
                        }
                    },
                    {                       
                        "behavior":
                        {
                            "type": "collisionAvoidance"
                        }
                    }
                ]
            }
        },
        {
            "position": [-10, 0, 10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4.0,
                "pipeline":
                [
                    {
                        "behavior":
                        {
                            "type": "pathFollowing",
                            "destination": [10, 0, -10],
                            "pathOptimizationRange": 6
                        }
                    },
                    {                       
                        "behavior":
                        {
                            "type": "collisionAvoidance"
                        }
                    }
                ]
            }
        },
        {
            "position": [10, 0, -10],
            "parameters": 
            {
                "maxSpeed": 2,
                "maxAcceleration": 10,
                "radius": 0.2,
                "height": 1.7,
                "collisionQueryRange": 4,
                "pipeline":
                [
                    {
                        "behavior":
                        {
                            "type": "pathFollowing",
                            "destination": [-10, 0, 10],
                            "pathOptimizationRange": 6
                        }
                    },
                    {                       
                        "behavior":
                        {
                            "type": "collisionAvoidance"
                        }
                    }
                ]
            }
        }
    ]
}