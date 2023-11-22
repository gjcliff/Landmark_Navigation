# landmark_manager
This package provides services for saving landmarks, canceling navigation, and navigating to landmarks. It also performs semantic labeling by publishing markers for detected doors and tables.
## Nodes
* `landmark_manager`: Provides functionalities for saving landmarks, canceling navigation, and navigating to landmarks
* `semantic_labeling`: Uses markers to label doors and tables
## Launch File
* `landmark_manager.launch.py`: Publishes a static transform between `base_camera` and `camera_link`, and runs `landmark_manager` and `semantic_labeling` nodes
## Parameters
The following are the parameters in `config/semantic_labeling_params.yaml`:
* `rate`: A frequency used for the timer
* `threshold`: The threshold for considering points as equal
## Message
* `SemanticPoint`: Contains a point in the map frame and the associated marker id
## Services
* `NavigateToLandmark`: Contains a name of the landmark to navigate to
* `SaveLandmark`: Contains a name, x-coordinate, and y-coordinate of the landmark to save