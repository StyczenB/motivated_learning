# Steps to perform

## Nodes:
### Main node
1. Pains calculation
   1. Get current grid coords of agent.
   2. Check known map of agent.
   3. Pains (from agent state)
      1. Low battery level pain. [DONE] test it
      2. Curiosity. [DONE] for now
      3. Going back to home. [DONE] kinda...
2. Choose pain with highest value as the main.
3. Get signals from known to agent neurons (internal map) which are associated with reducing this pain.
4. Send movement action to movement manager.
5. Update agent internal state:
   1. known fields - number of visits to fields, add new fields to internal map, 
   2. battery level, 
   3. wheels lubrication, 
   4. time spend away from home.
6. Update state of map (chargers, other fields).

#### How to calculate pain associated with curiosity having information about internal map of an agent?
Information about every field:
- string name (not needed)
- geometry_msgs/Point coords (in current implementation not needed - used for agent when assesing whether field 
  was visited previously or but giving goal coordinates)
- uint8 type (not needed)
- int32 nr_visits - the higher this value gets, the less interesting it is and visiting them increases pain because 
  agent cannot learn anything new.
This value can grow to infinity, so how to convert this values to pain? And also there are multiple fields


### Global map manager - provides information about all fields in the environment.

### Movement manager - for executing commands for moving,

### RQT - agent dashboard.
