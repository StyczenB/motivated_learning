# This message holds the description of action

# Actions as enums
# Movements
uint8 FORWARD = 1
uint8 LEFT = 2
uint8 RIGHT = 3

# Actions which affect environment's resources
uint8 CHARGE_BATTERY = 4
uint8 CARRY_BATTERIES_TO_CHARGER = 5
uint8 CHANGE_TIRES_IN_WORKSHOP = 6
uint8 CARRY_MATERIALS_TO_WAREHOUSE = 7

# Actions description as string
string FORWARD_STR = "forward"
string LEFT_STR = "left"
string RIGHT_STR = "right"

string CHARGE_BATTERY_STR = "charge_battery"
string CARRY_BATTERIES_TO_CHARGER_STR = "carry_batteries_to_charger"
string CHANGE_TIRES_IN_WORKSHOP_STR = "change_tires_in_workshop"
string CARRY_MATERIALS_TO_WAREHOUSE_STR = "carry_materials_to_warehouse"

Header header
string name      
uint8 action
