{
	"scene":
	{
		"file": "Meshes/corridor_120.obj"
	},
	"agents":
	[
		{
			"position": [-4, 0, 0],
			"destination": [4, 0, 0],
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
			"position": [4, 0, 0],
			"destination": [-4, 0, 0],
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
		}
	]
}