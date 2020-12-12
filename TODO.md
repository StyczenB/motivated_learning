# Steps to perform

### Nodes:
## Main node
1. Pains calculation
   1. Get current grid coords of agent.
   2. Check known map of agent.
   3. Pains (from agent state)
      1. Low battery level pain.
      2. Curiosity.
      3. Going back to home.
2. Choose pain with highest value as the main.
3. Get signals from known to agent neurons (internal map) which are assiciated with reducing this pain.
4. Send movement action to movement manager.
5. Update agent internal state:
   1. known fields - number of visits to fields, add new fields to internal map, 
   2. battery level, 
   3. wheels lubrication, 
   4. time spend away from home.
6. Update state of map (chargers, other fields).

## Global map manager - provides information about all fields in the environment.

## Movement manager - for executing commands for moving,

## RQT - agent dashboard.
