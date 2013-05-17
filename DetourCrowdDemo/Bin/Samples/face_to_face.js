{
	"scene":
	{
		"file": "Meshes/corridor_120.obj"
	},
	"agents":
	[
		{
			"position": [-4, 0, 0],
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
		                    "destination": [4, 0, 0],
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
			"position": [4, 0, 0],
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
		                    "destination": [-4, 0, 0],
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