## Running simulation
- Step 1: Initializing the Script
Navigate to the worlds directory.
Launch the set_ball_color.py script.
- Step 2: Opening the Simulation World
Open the file mars_mines.wbt.
- Step 3: Running the Simulation
After the simulation window opens, immediately stop it by clicking the stop icon.


Hover over the element in the simulation as shown below.
![image](https://github.com/arion023/KNR-droniada/assets/70472078/597e11fb-bf21-42fd-be96-66d4ae9bf90a)


Click to select and choose the controller node.

![image](https://github.com/arion023/KNR-droniada/assets/117840525/891ea3c9-fe31-4a59-9a1e-96e5f99f8414)

Reset the simulation to its initial state by clicking the rewind button twice.

![image](https://github.com/arion023/KNR-droniada/assets/117840525/06ee3a73-be82-488b-b483-bfd140791a3f)

Start the simulation by clicking the execute button.
![image](https://github.com/arion023/KNR-droniada/assets/117840525/b099eba9-7e52-4603-aead-15bd512aad22)

### Controllers short description.
- The ManualController allows you to control the drone using the arrow keys for directional movement and shift + arrow keys for altitude adjustments. Furthermore, it allows you to control the camera up and down with the 1 and 2 keys, and control camera roll with the A and D keys.
- The PipesController allows you to control the drone with external pipes. To run the demo, you need to run simulation/controller/PipesController/pipeWrite.py after or before running the simulation, as the pipes block on opening, causing the simulation to wait for them. More information can be found in the PipeController.py file.
