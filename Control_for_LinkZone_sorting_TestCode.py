class VirtualCAV:
    def __init__(self, ID, manhattan_distance, next_collision_point):
        self.ID = ID
        self.manhattan_distance = manhattan_distance
        self.current_collision_pt1 = next_collision_point

def calc_manhattan_distance(cav):
    return cav.manhattan_distance

def insert_cav_in_link_zone(new_cav, cavs, order_list, next_collision_point):
    # Identify the conflicting CAVs in the current order list based on the next collision point
    conflicting_cavs = [cav for cav in cavs if cav.current_collision_pt1 == next_collision_point]
    conflicting_cavs.append(new_cav)  # Include the new arriving CAV

    # Sort the conflicting CAVs by Manhattan distance
    conflicting_cavs.sort(key=lambda cav: calc_manhattan_distance(cav))

    # Create the final sorted order while maintaining non-conflicting CAV positions
    final_order = []
    conflicting_cav_iter = iter(conflicting_cavs)  # Iterator over the sorted conflicting CAVs
    conflicting_cav_ids = {cav.ID for cav in conflicting_cavs}  # Set of conflicting CAV IDs

    temp = conflicting_cavs.index(new_cav)
    
    if temp == len(conflicting_cavs) - 1:
        order_list.append(new_cav)
    else:
        temp = conflicting_cavs[temp+1]
        for index, cav in enumerate(order_list):
            if cav == temp.ID:
                temp = index
                tempVal = cav
                break
        order_list.insert(temp,new_cav.ID)

    return order_list

# Example scenario 1: Link Zone with conflicting and non-conflicting CAVs
new_cav = VirtualCAV("cav5", 7.5, "P1")  # New CAV arriving to the Link Zone with conflict
cavs = [
    VirtualCAV("cav3", 7, "P1"),
    VirtualCAV("cav1", 2, "P1"),
    VirtualCAV("cav4", 8.1, "P1"),
    VirtualCAV("cav6", 6, "P3")
]

order_list = ["cav3", "cav1", "cav4", "cav6"]  # Initial Link Zone order 5,3,1,4,6
next_collision_point = "P1"  # The collision point of interest for conflicting CAVs

# Run the sorting and insertion logic
sorted_order = insert_cav_in_link_zone(new_cav, cavs, order_list, next_collision_point)
print(f"Sorted order after inserting cav5 (Scenario 1): {sorted_order}")

# Example scenario 2: Different conflicting CAVs and new initial order
new_cav = VirtualCAV("cav8", 10, "P3")  # New CAV arriving with conflict
cavs = [
    VirtualCAV("cav1", 2, "P1"),
    VirtualCAV("cav6", 6, "P3"),
    VirtualCAV("cav3", 8, "P3"),
    VirtualCAV("cav5", 5, "P3"),
    VirtualCAV("cav4", 7, "P3")
]

order_list = ["cav1", "cav6", "cav3", "cav5", "cav4"]  # Different initial order
next_collision_point = "P3"  # Different collision point of interest

# Run the sorting and insertion logic
sorted_order = insert_cav_in_link_zone(new_cav, cavs, order_list, next_collision_point)
print(f"Sorted order after inserting cav8 (Scenario 2): {sorted_order}")
