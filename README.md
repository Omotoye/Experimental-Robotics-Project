<a href="https://unige.it/en/">
<img src="images/genoa_logo.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >
</a>

# Experimental Robotics Lab Project

>**Author: Omotoye Shamsudeen Adekoya**   
 **Email: adekoyaomotoye@gmail.com** </br>
 **Student ID: 5066348**

# Outline 
*   Introduction 
*   Description of the Software Architecture
    * Component Diagram 
    * State Diagram 
    * Sequence Diagram 
*   System Limitations and Possible Improvements 
*   Installation and running procedure

# Introduction 

<img src="images/cluedo.png" title="University of Genoa" alt="University of Genoa" >


**Cluedo** is a murder mystery game for three to six players (depending on editions) that was devised in 1943 by British board game designer Anthony E. Pratt.
The objective of the game is to *determine __who__ murdered the game's victim, __where__ the crime took place, and __what__ weapon was used*. Each player assumes the role of one of the six suspects and attempts to deduce the correct answer by strategically moving around a game board representing the rooms of a mansion and collecting clues about the circumstances of the murder from the other players. [Wikipedia](https://en.wikipedia.org/wiki/Cluedo)

The exact objective of this game (cluedo) is what was modeled by the **Cluedo Robot Senario** contained in this project. The Robot is supposed to: 

1. Explore the environment by entering in different rooms (e.g., randomly).
2. In each room, it should look around to find hints to make hypotheses.
3. When a consistent hypothesis is deducible, it should go in a designed location and express it in English.
4. If the hypothesis is wrong, it should keep exploring and find new hints.

This project contains packages to accomplish each of this task, the description of the working of the packages and how to install and launch them is contained in this readme.
 
>*for more information on the exact objectives of this project [click here](docs/7-Assignment.pdf) to read the assignment document.*

# Description of the Software Architecture
The images below is the component diagram the represents the software architecture of the project
## Component Diagram

<img src="UML%20Diagrams/component_diagram_v3.1.png"  title="Component Diagram Version 3.1" alt="Component Diagram Version 3.1" >

The Software Architecture contains 6 packages, each of which has a specific task to take care of:

1. Behavioural Logic 
1. Simulation Object
1. Navigation 
1. Knowledge Representation 
1. Custom Messages 
1. Armor

### Behavioural Logic 
This package contains the state machine scripts, the scripts was shared into two; the **states** and **logic** script. In the states script, this is where the state class of each of the state of the state machine are defined (*more about the states of the state machine later in the readme*). In each state, only one action is perform which is calling the action server in the **Robot Object** to perform the task. No type of computation is performed in the state script, it only calls the action to perform the required task per state. It was done this way in order to define the exact role of the state machine package which is to switch from one state to another. The logic script is where all the transtionas and the overall structure of the state machine is defined. 


### Simulation Object
This package contains the class definition of a **Robot Object**; this robot object has an action server that takes command of action to perform from the state machine. It is in charge of calling each of the components required to perform an action. It is also in charge of storing the information about the state of the robot. (ie, *where the robot is at, the new hint gotten by the robot, the new hypothesis formed by the robot etc*) 



### Navigation Package
As the name implies, it is in charge of navigating the robot to a required **point of interest**. This package has a YAML file called **poi_map_cord**. This file contains the coordinates of each of the so called point of interest. It is structured in the format shown below. 

```yaml
dinning_room: {
  loc_name: "Dinning Room",
  x: 2.5, 
  y: 10.0,
}

kitchen: {
  loc_name: "Kitchen",
  x: 2.5, 
  y: 17.5
}
# and many more...
```
The id of the point of interest (ie *dinning_room...*) is the keyword that is being received by the navigation node from the Robot Object. The navigation node uses the id to extract the target coordinate (x,y) and navigate to that coordinate. However the navigation has not been implemented in this version of the project, so the navigation only prints this coordinate to the terminal and doesnt do anything with it; for now, it simple waste time for a random amount of seconds (*from 1 to 5sec*) and returns **goal reached** to the robot object. 

### Knowledge Representation
This package is perhaps the most important part of this project; it contains the **Oracle Object** and the **Knowledge Manager**. 

From the goal objectives of the project, the oracle is in charge of generating hints for the robot to detect randomly from the rooms and most importantly, the Oracle has the information about the correct hypothesis for the **who, what** and **where** of the killer in the game. 

For managing the knowledge in (hint and hypothesis) **OWL Ontology** is used and to communicate with the Cluedo OWL Ontology, an external package called **ARMOR** is used. 
The knowledge manager helps to save and generate the required request message to the ARMOR service to perform actions such as **Update the Ontology, Query the Ontology for Complete and Consistent Hypothesis, because this is required to be done with a sequence of command (*for example to add a hint, you have to add, disjoint the hint class, and call the reasoner*) the knowledge manager serves as a medium to perform this sequence of action depending on the required task. 

When the simulation is started the Oracle which know all the consistent and complete hypothesis, selects the correct one out of them (randomly) and the robot is supposed to find out what it is by accumulating hints gotten from rooms. 
The hints are stored in a YAML file with the format shown below, it is then loaded into a parameter server which is called by both the knowledge manager and the oracle object. 
```yaml
hints:
  - HP0:
    - who: "Toye"
    - what: "Knife"
    - where: "Library"
  - HP1:
    - who: "Tom"
    - what: "Knife"
    - where: "Ball Room"
    - what: "Cutlass"
  - HP2:
    - who: "Jerry"
    - where: "Hall"
# and more ....
```

### Custom Messages

This package was created specificaly for storing all the custom message that was generated for interfacing between componnets in this project. One action message was generated: **Robot.actoin** and three service message: **Knowledge.srv, Oracle.srv, RobotNav.srv**, by the name you can tell the exact package they help interface with. 

### ARMOR

The ARMOR package is an external package used to communicate with the Cluedo OWL ontology, it is a ROS wrapper that help to make this communication possible. For more information about ARMOR [click here](https://github.com/EmaroLab/armor.git)

>__*for more information about the scripts in the project [click here](https://omotoye.github.io/Experimental-Robotics-Project/)*__


## State Diagram

<img src="UML%20Diagrams/state_diagram_v2.png"  title="Smach Viewer State Diagram Version 2" alt="Smach Viewer State Diagram Version 2.1" >

As promised, here is a diagram containing the states of the state machine of the architecture. 

There are 8 states in the architecture, the exact task of each of the states are defined below. 

1. GoTo Room 
2. Knowledge Management
   1. Search Hint 
   2. Update Knowledge
   3. Check for Consistent and Complete Hypothesis 
3. GoTo Oracle 
4. Announce Hypothesis 
5. Oracle(Hypothesis Check)

### GoTo Room 
In this state;
* The state machine sends a goal `go to room` to the robot object. 
* The robot object then randomly selects a room to go to (other than the one it is presently at). 
* The robot object sends the randomly selected room id to the navigation node. 
* The navigation node tries to reach the coordinate of the room (in this case **waste time**)
* If it reaches the room sends a response of `goal reached` to the robot object which then relays this message to the state machine. 
* If not, it send the error message (ie `invalid id`, `cannot reach point` etc ...) to the robot object which then sends it to the state machine. 
* If the message is goal reached, the outcome of the state is `at room` which causes a transition to **knowledge management**
* If not, the outcome is `not at room` which cause the state to transition back to itself to try the action again. 

### Knowledge Management 
This is a sub-statemachine that contain three states, **Search Hint, Update Knowledge and Check for Consistent and Complete Hypothesis**

**for _search hint_;**
* The state machine sends a `search hint` goal to the robot object. 
* The robot object then request the oracle object to generate a hint for it. 
* This hint is then recieve by the camera object which is a part of the robot object. 
* In later versions, the camera would process the image gotten and extract the hint from it. 
* The hint is then store as an attribute by the robot_object for later use. 
* if hint found, the robot object sends a result of `hint found` to the state machine
* if not, it sends `no hint` to the state machine. 
* if found the state machine would transition to **Update Knowledge** 
* if not found, the state machine would transition to itself to repeat the process. 
  
**for _update knowledge_;**
* The robot object recieves a request of `update knowledge` from the state machine. 
* The robot object then sends the previously saved hint to the knowledge manager to use armor to save it in the cluedo ontology. 
* If armor return `sucess` the knowledge manager send `ok` to the robot object which then sends `updated` to the state machine 
* If the state machine receives a response of `updated`, it causes an outcome of `knowledge updated` which then transitions the state to **Check for consistent and Complete Hypothesis**
* else it causes an outcome of `knowledge update failed` which then transistion back to itself to try again. 

**for _check for consistent and complete hypothesis_;**
* The robot object recieves a request of `check hypo` from the state machine.
* The robot object then sends a request of `hypo check` to the knowledge manager
* The knowledge manager uses armor to query the ontology for complete and inconsistent hypothesis, it then figures out the complete hypothesis that is not inconsistent from the response list set to it. More information from the code snippet below. 
```python 
good_hypo = [] # an empty list to save the list of complete and consistent hypothesis
_completed = self.ind_b2_class("COMPLETED") # query the ontology for COMPLETED hypothesis
_inconsistent = self.ind_b2_class("INCONSISTENT") # query the ontology for INCONSISTENT hypothesis
if len(_completed) > 0:
    # code to figure out the hypothesis that is in _completed and not in _inconsistent
    for hypothesis in _completed:
        if hypothesis not in _inconsistent:
            good_hypo.append(hypothesis[42:-1]) # using slicing to cut out unneccesary information from the hypothesis name

# this part is self explanatory :)            
if len(good_hypo) > 0:
    response.result = "hypo found"
    response.hypo_ids = good_hypo
else:
    response.result = "no hypo found"
```
* This response (`hypo found` or `no hypo found`) is then sent to the robot object
* If the response is `hypo found`, the robot object saves the new hypo in a variable/attribute called *new_hypo* for later use. 
* The robot object send a response of either `hypo found` or `not found` to the state machine.
* if hypo found, it'll cause the outcome of `complete & consistent hypo found` which will cause a break out of the sub-state machine (Knowledge Management) and transition to **GoTo Oracle**
* else it'll cause the outcome of `not found` which will cause a break out of the sub-state machine and transistion to **GoTo Room**

### GoTo Oracle 
In this state;
* The state machine send a request of `go to oracle` to the robot object.
* The robot object then takes the already known id of the oracle location and sends it to the navigation node.
* The navigation node takes the coordinates of the oracle from the map parameter server and navigate to it (in our case, waste time).
* If the navigation was successful, it returns a response of `goal reached` to the robot object, else it returns the error message. 
* If the robot object get a response of `goal reached`, it transfers the response to the state machine, else it sends `navigation failed` to the state machine. 
* If the state machine receives a response of `goal reached`, it causes an outcome of `reached oracle` which then transition to **Announce Hypothesis**
* else it causes an outcome of `failed to reach oracle` which then transition back to itself to try again. 
  
### Announce Hypothesis 
In this state;
* The state machine send a request of `announce hypothesis` to the robot object. 
* The robot object takes the already save hypotheis from the *new_hypo* variable/attribute and sends it to the knowledge manager to announce. 
* The knowledge manager uses the hypothesis id to get the hypothesis from the **cluedo_hints** parameter server, it then prints it to the command line using the code snippet below.
```python
who, what, where = self.get_hypo_data(msg.hypo_id)
print("\n\nHey there Oracle!, I have a correct hypothesis for you.\n")
print(f"{who} performed the killing at the {where} with a {what}\n\n")
response.result = "announced"
```
* The robot object takes the response and transfers it to the state machine. 
* If the response is `announced` it'll cause an outcome of `hypothesis announced` which then transistions to the **Oracle (Hypothesis Check)**
* else it'll cause an outcome of `oracle check failed` which would then transistion back to itself to try again. 

### Oracle (Hypothesis Check)
In this state; 
* The state machine sends a request of `oracle check` to the robot object. 
* The robot object takes the already saved hypothesis from the *new_hypo* variable/attribute and sends it to the oracle_object for check.
* __after sending the *new_hypo* data, it is added to a list called `checked_hypo` so the system doesnt go back to check an hypothesis that has been defined as wrong by the oracle__ 
* The Oracle object already has knowledge of the correct hypothesis since it was the one that selected it. 
* The Oracle object would check if the hypothesis is the correct one, if yes it send a response of `correct hypothesis` if no, it sends a response of `wrong hypothesis`. 
* The robot object simple transfers this response to the state machine. 
* A reponse of `correct hypothesis` to the state machine would cause an outcome of `hypothesis correct` which would ends the game by transitioning the state machine to an end point called `Game Won!!!`
* if the response is `wrong hypothesis`, it causes an outcome of `hypothesis wrong` which would transition to the **GoTo Room** state. 
* if the check failed, it'll cause an outcome of `oracle check failed` which would cause a transistion back to itself to try again.






